#include "nuvo_camera/camera.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <mutex>
#include <nuvo_camera_msgs/msg/frame_metadata.hpp>
#include <optional>
#include <rclcpp_components/register_node_macro.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <thread>

#include "nuvo_camera/conversion.hpp"

namespace nuvo_camera {
class NuvoCamera::Impl {
 public:
  explicit Impl(NuvoCamera& node) : node_(node) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    auto text = [&](const char* name, const std::string& value) {
      return node_.declare_parameter<std::string>(name, value, descriptor);
    };
    source_kind_ = text("source", "local");
    endpoint_ = text("endpoint", "/run/nuvo-depth/data.sock");
    recording_ = text("recording", "");
    frame_id_ = text("frame_id", "nuvo_left_rectified_optical");
    mode_ = text("timestamp_mode", "auto");
    ca_ = text("ca_file", "");
    credential_ = text("credential_file", "");
    connect_address_ = text("connect_address", "");
    subscription_.delivery = text("delivery", "best_effort");
    subscription_.cadence_hz =
        node_.declare_parameter("cadence_hz", 10.0, descriptor);
    allow_provisional_ = node_.declare_parameter(
        "allow_provisional_calibration", false, descriptor);
    enable_points_ =
        node_.declare_parameter("publish_points", false, descriptor);
    lazy_ = node_.declare_parameter("lazy", true, descriptor);
    stride_ = node_.declare_parameter("point_stride", 2, descriptor);
    speed_ = node_.declare_parameter("replay_rate", 1.0, descriptor);
    loop_ = node_.declare_parameter("replay_loop", false, descriptor);
    publish_clock_ =
        node_.declare_parameter("publish_clock", false, descriptor);
    max_age_ = node_.declare_parameter("maximum_frame_age_s", 1.0, descriptor);
    const auto qos_depth = node_.declare_parameter("qos_depth", 1, descriptor);
    const auto reliability = text("reliability", "best_effort");
    const auto fallback = text("calibration_override_file", "");
    if (!fallback.empty())
      override_ = nuvo::client::load_calibration_override(fallback);
    if (source_kind_ != "local" && source_kind_ != "https" &&
        source_kind_ != "replay")
      throw std::invalid_argument("source must be local, https or replay");
    if (mode_ == "auto")
      mode_ = source_kind_ == "local"   ? "local_boottime"
              : source_kind_ == "https" ? "receipt"
                                        : "replay_rebased";
    if ((source_kind_ == "local" && mode_ != "local_boottime" &&
         mode_ != "receipt") ||
        (source_kind_ == "https" && mode_ != "receipt") ||
        (source_kind_ == "replay" && mode_ != "replay_rebased"))
      throw std::invalid_argument(
          "timestamp_mode incompatible with source; remote acquisition "
          "currently requires labeled receipt mode");
    if (frame_id_.empty() || frame_id_.front() == '/' || stride_ < 1 ||
        stride_ > 16 || qos_depth < 1 || qos_depth > 16 ||
        !std::isfinite(speed_) || speed_ <= 0 || speed_ > 100 ||
        !std::isfinite(max_age_) || max_age_ <= 0 || max_age_ > 30)
      throw std::invalid_argument(
          "invalid frame, queue, cloud stride, replay rate or freshness "
          "parameters");
    nuvo::client::validate(subscription_);
    if (source_kind_ == "https" && subscription_.cadence_hz < 1.0 / 60.0)
      throw std::invalid_argument(
          "HTTPS cadence_hz must be at least 1/60 Hz for its 120 s deadline");
    if (source_kind_ == "replay" && recording_.empty())
      throw std::invalid_argument("replay requires recording path");
    if (source_kind_ != "replay" &&
        (publish_clock_ || node_.get_parameter("use_sim_time").as_bool()))
      throw std::invalid_argument(
          "live acquisition requires system ROS time and cannot publish "
          "/clock");
    if (source_kind_ == "replay" &&
        node_.get_parameter("use_sim_time").as_bool() && !publish_clock_)
      throw std::invalid_argument(
          "replay use_sim_time requires publish_clock to define the playback "
          "timeline");
    const auto initial_sim_time = node_.get_parameter("use_sim_time").as_bool();
    time_parameters_ = node_.add_on_set_parameters_callback(
        [initial_sim_time](const std::vector<rclcpp::Parameter>& parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto& parameter : parameters)
            if (parameter.get_name() == "use_sim_time" &&
                (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL ||
                 parameter.as_bool() != initial_sim_time)) {
              result.successful = false;
              result.reason = "use_sim_time is fixed at acquisition startup";
            }
          return result;
        });
    if (publish_clock_) mode_ = "replay_clock";
    if (source_kind_ == "local" && mode_ == "local_boottime")
      (void)nuvo::client::local_clock_sample();
    RCLCPP_INFO(node_.get_logger(),
                "Starting Nuvo %s source in %s with %s timestamps",
                source_kind_.c_str(), node_.get_namespace(), mode_.c_str());
    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<std::size_t>(qos_depth))};
    if (reliability == "best_effort")
      qos.best_effort();
    else if (reliability == "reliable")
      qos.reliable();
    else
      throw std::invalid_argument(
          "reliability must be best_effort or reliable");
    depth_ = node_.create_publisher<sensor_msgs::msg::Image>("depth/image_rect",
                                                             qos);
    validity_ =
        node_.create_publisher<sensor_msgs::msg::Image>("depth/validity", qos);
    info_ = node_.create_publisher<sensor_msgs::msg::CameraInfo>(
        "depth/camera_info", qos);
    metadata_ = node_.create_publisher<nuvo_camera_msgs::msg::FrameMetadata>(
        "frame_metadata", qos);
    if (enable_points_)
      points_ =
          node_.create_publisher<sensor_msgs::msg::PointCloud2>("points", qos);
    diagnostics_ =
        node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", rclcpp::QoS(1).reliable());
    if (publish_clock_)
      clock_ = node_.create_publisher<rosgraph_msgs::msg::Clock>(
          "/clock", rclcpp::ClockQoS());
    timer_ = node_.create_wall_timer(std::chrono::seconds(1),
                                     [this] { diagnose(); });
    worker_ = std::thread([this] { run(); });
    RCLCPP_INFO(node_.get_logger(), "Nuvo publishers ready");
  }
  ~Impl() {
    stopped_ = true;
    wake_.notify_all();
    {
      std::lock_guard lock(source_mutex_);
      if (source_) source_->cancel();
    }
    if (worker_.joinable()) worker_.join();
  }

 private:
  void state(const std::string& message, unsigned level = 0) {
    std::lock_guard lock(status_mutex_);
    status_ = message;
    level_ = level;
  }
  bool wait(std::chrono::steady_clock::duration delay) {
    std::unique_lock lock(wait_mutex_);
    return wake_.wait_for(lock, delay, [this] { return stopped_.load(); });
  }
  std::shared_ptr<nuvo::client::Source> make_source() {
    if (source_kind_ == "local")
      return nuvo::client::local_source(endpoint_, subscription_);
    if (source_kind_ == "replay")
      return nuvo::client::replay_source(recording_);
    nuvo::client::HttpsOptions options;
    options.url = endpoint_;
    options.ca_file = ca_;
    options.credential_file = credential_;
    options.subscription = subscription_;
    options.connect_address = connect_address_;
    // Low-cadence subscriptions need a message deadline longer than their
    // interval.
    options.timeout = std::chrono::milliseconds(static_cast<int>(std::min(
        120000.0, std::max(5000.0, 2000.0 / subscription_.cadence_hz))));
    return nuvo::client::https_source(options);
  }
  void run() {
    unsigned failures{};
    while (!stopped_ &&
           rclcpp::ok(node_.get_node_base_interface()->get_context())) {
      if (lazy_ && !has_consumers()) {
        state("waiting for ROS measurement subscribers", 1);
        if (wait(std::chrono::milliseconds(200))) break;
        continue;
      }
      std::shared_ptr<nuvo::client::Source> current;
      bool idle = false;
      bool provisional_blocked = false;
      try {
        state("connecting", 1);
        current = make_source();
        {
          std::lock_guard lock(source_mutex_);
          source_ = current;
          if (stopped_) current->cancel();
        }
        while (!stopped_) {
          if (lazy_ && !has_consumers() && source_kind_ == "replay" &&
              !resume_replay())
            break;
          if (lazy_ && !has_consumers() && source_kind_ != "replay") {
            idle = true;
            current->cancel();
            break;
          }
          auto frame = current->next();
          if (!frame) {
            if (current->finished()) {
              if (loop_) {
                segment_ = false;
                previous_identity_.reset();
                break;
              }
              state("replay complete", 1);
              clear_source();
              return;
            }
            continue;
          }
          // Receipt mode labels arrival at the adapter, before calibration,
          // image conversion or publication can add processing latency.
          const auto receipt_stamp =
              mode_ == "receipt" ? node_.now().nanoseconds() : 0;
          if (previous_identity_ && frame->identity == *previous_identity_) {
            ++duplicates_;
            continue;
          }
          if (override_ && !frame->matching_geometry)
            nuvo::client::apply_calibration_override(*frame, *override_);
          if (!frame->matching_geometry)
            throw std::runtime_error(
                "source lacks matching geometry; use a matching "
                "hash/grid-checked calibration override or compatible camera "
                "software");
          if (source_kind_ != "replay" && frame->calibration_authority == 0 &&
              !allow_provisional_) {
            state("provisional calibration: publication requires explicit "
                  "allow_provisional_calibration", 1);
            // Release native matcher demand while publication is blocked.
            // Probe periodically so a validated calibration can recover.
            provisional_blocked = true;
            current->cancel();
            break;
          }
          if (publish(*frame, receipt_stamp)) {
            failures = 0;
            previous_identity_ = frame->identity;
          }
        }
      } catch (const std::exception& error) {
        if (!stopped_) {
          ++errors_;
          state(error.what(), 2);
          RCLCPP_WARN(node_.get_logger(), "Acquisition recovery: %s",
                      error.what());
        }
      }
      clear_source();
      current.reset();
      if (stopped_) break;
      if (provisional_blocked) {
        if (wait(std::chrono::seconds(5))) break;
        continue;
      }
      if (idle) {
        previous_identity_.reset();
        segment_ = false;
        continue;
      }
      if (source_kind_ == "replay" && !loop_) return;
      if (wait(std::chrono::milliseconds(
              std::min(5000U, 250U * (1U << std::min(failures++, 4U))))))
        break;
    }
  }
  void clear_source() {
    std::lock_guard lock(source_mutex_);
    source_.reset();
  }
  bool has_consumers() const {
    return depth_->get_subscription_count() +
               depth_->get_intra_process_subscription_count() +
               validity_->get_subscription_count() +
               validity_->get_intra_process_subscription_count() +
               info_->get_subscription_count() +
               info_->get_intra_process_subscription_count() +
               metadata_->get_subscription_count() +
               metadata_->get_intra_process_subscription_count() +
               (points_ ? points_->get_subscription_count() +
                              points_->get_intra_process_subscription_count()
                        : 0) >
           0;
  }
  bool resume_replay() {
    const auto paused = std::chrono::steady_clock::now();
    while (!stopped_ && !has_consumers()) {
      state("waiting for ROS measurement subscribers", 1);
      if (wait(std::chrono::milliseconds(200))) return false;
    }
    // Retain the open recording and pause its pacing, rather than rewinding
    // the capture stream or catching up in a burst when subscribers return.
    if (segment_) steady_base_ += std::chrono::steady_clock::now() - paused;
    return !stopped_;
  }
  bool publish(const nuvo::client::Frame& frame, std::int64_t receipt_stamp) {
    // builtin_interfaces/Time has a signed 32-bit seconds field.
    constexpr std::int64_t maximum_ros_stamp = 2147483647999999999LL;
    std::int64_t stamp{}, uncertainty = -1;
    bool clock_jump = false;
    if (mode_ == "local_boottime") {
      const auto sample = nuvo::client::local_clock_sample();
      const auto age = sample.boottime_ns - frame.capture_time_ns;
      if (age < -5000000LL || static_cast<double>(age) > max_age_ * 1e9) {
        ++stale_;
        state("stale or future local capture rejected", 1);
        return false;
      }
      const auto offset = sample.system_ns - sample.boottime_ns;
      clock_jump =
          clock_offset_ && std::abs(offset - *clock_offset_) > 100000000LL;
      clock_offset_ = offset;
      stamp = frame.capture_time_ns + offset;
      uncertainty = sample.uncertainty_ns;
    } else if (source_kind_ == "replay") {
      const bool new_segment = !segment_ ||
                               frame.identity.run_uuid != replay_run_ ||
                               frame.capture_time_ns <= last_source_time_;
      if (new_segment) {
        segment_ = true;
        replay_run_ = frame.identity.run_uuid;
        source_base_ = frame.capture_time_ns;
        ros_base_ =
            last_stamp_ > 0
                ? last_stamp_ + 1
                : (publish_clock_ ? source_base_ : node_.now().nanoseconds());
        steady_base_ = std::chrono::steady_clock::now();
      }
      const auto delta = frame.capture_time_ns - source_base_;
      const auto wall_delta = static_cast<double>(delta) / speed_;
      if (ros_base_ <= 0 || ros_base_ > maximum_ros_stamp ||
          delta > maximum_ros_stamp - ros_base_ ||
          wall_delta > 1e18)
        throw std::runtime_error("replay timestamp exceeds supported timeline");
      const auto pacing =
          std::chrono::nanoseconds(static_cast<std::int64_t>(wall_delta));
      while (!stopped_) {
        if (lazy_ && !has_consumers() && !resume_replay()) return false;
        const auto delay = steady_base_ + pacing -
                           std::chrono::steady_clock::now();
        if (delay <= std::chrono::steady_clock::duration::zero()) break;
        if (wait(std::min(delay, std::chrono::steady_clock::duration(
                                     std::chrono::milliseconds(200)))))
          return false;
      }
      stamp = ros_base_ + delta;
      last_source_time_ = frame.capture_time_ns;
    } else
      stamp = receipt_stamp;
    if (stopped_) return false;
    if (stamp <= 0 || stamp > maximum_ros_stamp)
      throw std::runtime_error("ROS acquisition stamp is outside supported range");
    // Rejected replay frames still advance playback pacing. Filtering them
    // before scheduling would scan a provisional recording at full CPU speed.
    if (frame.calibration_authority == 0 && !allow_provisional_) {
      state("provisional calibration: publication requires explicit "
            "allow_provisional_calibration", 1);
      return false;
    }
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(stamp);
    header.frame_id = frame_id_;
    auto images = convert(frame, header);
    if (clock_) {
      rosgraph_msgs::msg::Clock message;
      message.clock = header.stamp;
      clock_->publish(message);
    }
    if (points_ && (points_->get_subscription_count() +
                        points_->get_intra_process_subscription_count() >
                    0))
      points_->publish(cloud(images, static_cast<unsigned>(stride_)));
    info_->publish(images.camera_info);
    validity_->publish(std::move(images.validity));
    depth_->publish(std::move(images.depth));
    nuvo_camera_msgs::msg::FrameMetadata metadata;
    metadata.header = header;
    metadata.device_uuid = frame.identity.device_uuid;
    metadata.run_uuid = frame.identity.run_uuid;
    metadata.capture_epoch = frame.identity.capture_epoch;
    metadata.capture_sequence = frame.identity.capture_sequence;
    metadata.configuration_epoch = frame.identity.configuration_epoch;
    metadata.stream_sequence = frame.stream_sequence;
    metadata.source_capture_time_ns = frame.capture_time_ns;
    metadata.source_capture_clock = frame.capture_clock;
    metadata.calibration_sha256 = frame.calibration_sha256;
    metadata.calibration_authority =
        static_cast<std::uint8_t>(frame.calibration_authority);
    metadata.timestamp_mode = mode_;
    metadata.timestamp_uncertainty_ns = uncertainty;
    metadata.replay = source_kind_ == "replay";
    metadata_->publish(metadata);
    last_publication_steady_ns_ =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    if (++published_ == 1)
      RCLCPP_INFO(node_.get_logger(), "First associated depth frame published");
    last_stamp_ = stamp;
    state(clock_jump ? "streaming; system clock jump remapped"
          : frame.calibration_authority == 0
              ? "streaming with provisional calibration"
          : mode_ == "receipt" ? "streaming with receipt timestamps"
                               : "streaming",
          clock_jump || frame.calibration_authority == 0 || mode_ == "receipt"
              ? 1
              : 0);
    return true;
  }
  void diagnose() {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = node_.now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = node_.get_fully_qualified_name();
    status.hardware_id = source_kind_;
    {
      std::lock_guard lock(status_mutex_);
      status.message = status_;
      status.level = static_cast<std::uint8_t>(level_);
    }
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
    const auto last = last_publication_steady_ns_.load();
    if (last > 0 && status.message.starts_with("streaming") &&
        static_cast<double>(now - last) >
            std::max(2.0, 3.0 / subscription_.cadence_hz) * 1e9) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "waiting for fresh associated frames";
    }
    auto add = [&](const std::string& key, const std::string& value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      status.values.push_back(item);
    };
    add("source", source_kind_);
    add("timestamp_mode", mode_);
    add("published_frames", std::to_string(published_.load()));
    add("acquisition_errors", std::to_string(errors_.load()));
    add("stale_frames", std::to_string(stale_.load()));
    add("duplicate_frames", std::to_string(duplicates_.load()));
    message.status.push_back(status);
    diagnostics_->publish(message);
  }
  NuvoCamera& node_;
  std::string source_kind_, endpoint_, recording_, frame_id_, mode_, ca_,
      credential_, connect_address_;
  nuvo::client::Subscription subscription_;
  std::optional<nuvo::client::CalibrationOverride> override_;
  bool allow_provisional_{}, enable_points_{}, lazy_{}, loop_{},
      publish_clock_{}, segment_{};
  int stride_{};
  double speed_{}, max_age_{};
  std::optional<nuvo::client::Identity> previous_identity_;
  std::optional<std::int64_t> clock_offset_;
  std::int64_t source_base_{}, ros_base_{}, last_source_time_{}, last_stamp_{};
  std::string replay_run_;
  std::chrono::steady_clock::time_point steady_base_;
  std::atomic_bool stopped_{};
  std::atomic_uint64_t published_{}, errors_{}, stale_{}, duplicates_{};
  std::atomic_int64_t last_publication_steady_ns_{};
  std::mutex source_mutex_, status_mutex_, wait_mutex_;
  std::condition_variable wake_;
  std::shared_ptr<nuvo::client::Source> source_;
  std::string status_{"starting"};
  unsigned level_{1};
  std::thread worker_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_, validity_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_;
  rclcpp::Publisher<nuvo_camera_msgs::msg::FrameMetadata>::SharedPtr metadata_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      time_parameters_;
};
NuvoCamera::NuvoCamera(const rclcpp::NodeOptions& options)
    : Node("nuvo_camera", options), impl_(std::make_unique<Impl>(*this)) {}
NuvoCamera::~NuvoCamera() = default;
}  // namespace nuvo_camera
RCLCPP_COMPONENTS_REGISTER_NODE(nuvo_camera::NuvoCamera)
