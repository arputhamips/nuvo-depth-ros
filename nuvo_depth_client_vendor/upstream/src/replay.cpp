#include <atomic>
#include <cstring>
#include <fstream>
#include <stdexcept>

#include "nuvo/client.hpp"
#if defined(__linux__)
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif
namespace nuvo::client {
namespace {
class Replay final : public Source {
 public:
  explicit Replay(const std::filesystem::path& path) {
#if defined(__linux__)
    const int descriptor =
        ::open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NONBLOCK);
    struct stat status {};
    if (descriptor < 0)
      throw std::invalid_argument("cannot open NVR1 recording");
    if (::fstat(descriptor, &status) != 0 || !S_ISREG(status.st_mode)) {
      ::close(descriptor);
      throw std::invalid_argument("NVR1 replay requires a regular file");
    }
    input_.open("/proc/self/fd/" + std::to_string(descriptor),
                std::ios::binary);
    ::close(descriptor);
#else
    if (!std::filesystem::is_regular_file(path))
      throw std::invalid_argument("NVR1 replay requires a regular file");
    input_.open(path, std::ios::binary);
#endif
    char magic[8]{};
    input_.read(magic, 8);
    if (!input_ || std::memcmp(magic, "NVR1\0\0\0\1", 8) != 0)
      throw std::invalid_argument("not an NVR1 recording");
  }
  std::optional<Frame> next() override {
    if (stopped_ || done_) return {};
    std::array<unsigned char, 8> length{};
    input_.read(reinterpret_cast<char*>(length.data()), 8);
    if (input_.gcount() == 0 && input_.eof()) {
      done_ = true;
      return {};
    }
    if (input_.gcount() != 8)
      throw std::invalid_argument("truncated NVR1 record length");
    std::uint64_t size{};
    for (auto b : length) size = (size << 8U) | b;
    if (size < 32 || size > maximum_wire_bytes)
      throw std::invalid_argument("NVR1 record exceeds bounds");
    auto bytes = std::make_shared<std::vector<std::byte>>(size);
    input_.read(reinterpret_cast<char*>(bytes->data()),
                static_cast<std::streamsize>(size));
    if (!input_) throw std::invalid_argument("truncated NVR1 frame");
    return decode_frame(*bytes, bytes);
  }
  void cancel() noexcept override { stopped_ = true; }
  bool finished() const noexcept override { return done_; }

 private:
  std::ifstream input_;
  std::atomic_bool stopped_{}, done_{};
};
}  // namespace
std::unique_ptr<Source> replay_source(const std::filesystem::path& path) {
  return std::make_unique<Replay>(path);
}
}  // namespace nuvo::client
