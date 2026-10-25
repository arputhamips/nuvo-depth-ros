#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <nuvo/client.hpp>
#include <stdexcept>
#include <thread>
#if defined(__linux__)
#include <sys/time.h>

#include <csignal>
#endif
int main(int argc, char** argv) {
  using namespace nuvo::client;
  try {
    if (argc == 1) {
      auto sample = local_clock_sample();
      if (sample.boottime_ns <= 0 || sample.system_ns <= sample.boottime_ns ||
          sample.uncertainty_ns < 0)
        return 1;
      const std::array<std::byte, 5> unaligned{std::byte{7}, std::byte{0},
                                               std::byte{0}, std::byte{128},
                                               std::byte{63}};
      if (float32_le(unaligned.data() + 1) != 1.0F) return 1;
      try {
        Subscription s;
        s.products = {"depth", "depth"};
        validate(s);
        return 1;
      } catch (const std::invalid_argument&) {
      }
      return 0;
    }
    const std::string mode = argv[1];
    std::unique_ptr<Source> source;
    if (mode == "local" || mode == "cancel-local" || mode == "interrupt-local")
      source = local_source(argv[2]);
    else if (mode == "https" || mode == "cancel-https") {
      HttpsOptions options;
      options.url = argv[2];
      options.ca_file = argv[3];
      options.credential_file = argv[4];
      if (argc > 5) options.connect_address = argv[5];
      options.timeout = std::chrono::milliseconds(1500);
      source = https_source(options);
    } else if (mode == "replay")
      source = replay_source(argv[2]);
    else if (mode == "decode" || mode == "override") {
      std::ifstream input(argv[2], std::ios::binary);
      std::vector<char> data((std::istreambuf_iterator<char>(input)), {});
      auto frame = decode_frame(
          {reinterpret_cast<const std::byte*>(data.data()), data.size()});
      if (mode == "override") {
        const auto original_authority = frame.calibration_authority;
        apply_calibration_override(frame, load_calibration_override(argv[3]));
        if (frame.calibration_authority != original_authority) return 1;
      }
      if (!frame.matching_geometry || !frame.find("depth") ||
          float32_le(frame.find("depth")->bytes.data()) !=
              (mode == "decode" && argc > 3 ? std::stof(argv[3]) : 2.0F))
        return 1;
      std::cout << frame.identity.capture_sequence << '\n';
      return 0;
    } else
      return 1;
#if defined(__linux__)
    if (mode == "interrupt-local") {
      struct sigaction action {
      }, previous{};
      action.sa_handler = [](int) {};
      sigemptyset(&action.sa_mask);
      if (sigaction(SIGALRM, &action, &previous) != 0) return 1;
      itimerval signals{{0, 20000}, {0, 20000}}, disabled{};
      if (setitimer(ITIMER_REAL, &signals, nullptr) != 0) return 1;
      const auto started = std::chrono::steady_clock::now();
      bool rejected = false;
      try {
        (void)source->next();
      } catch (const std::exception&) {
        rejected = true;
      }
      setitimer(ITIMER_REAL, &disabled, nullptr);
      sigaction(SIGALRM, &previous, nullptr);
      const auto elapsed = std::chrono::steady_clock::now() - started;
      return rejected && elapsed >= std::chrono::milliseconds(1100) &&
                     elapsed < std::chrono::milliseconds(1800)
                 ? 0
                 : 1;
    }
#endif
    if (mode.starts_with("cancel-")) {
      std::jthread cancel([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        source->cancel();
      });
      const auto start = std::chrono::steady_clock::now();
      try {
        (void)source->next();
      } catch (const std::exception&) {
      }
      return std::chrono::steady_clock::now() - start < std::chrono::seconds(2)
                 ? 0
                 : 1;
    }
    unsigned count{};
    std::optional<Frame> retained;
    while (count < 2) {
      auto frame = source->next();
      if (!frame) {
        if (source->finished()) break;
        continue;
      }
      if (!frame->matching_geometry || !frame->find("depth") ||
          !frame->find("validity"))
        throw std::runtime_error("fixture products or geometry absent");
      if (!retained) retained = *frame;
      std::cout << frame->identity.capture_sequence << '\n';
      ++count;
    }
    source.reset();  // Frame views must survive source and mapping teardown.
    const auto expected =
        mode == "replay" && argc > 3 ? std::stof(argv[3]) : 2.0F;
    if (count == 0 || !retained ||
        float32_le(retained->find("depth")->bytes.data()) != expected)
      return 1;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
