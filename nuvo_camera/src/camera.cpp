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
#include "nuvo_camera/policy.hpp"

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
    right_frame_id_ = text("right_frame_id", "nuvo_right_rectified_optical");
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
    enable_left_ = node_.declare_parameter("publish_left_rectified", false, descriptor);
    enable_right_ = node_.declare_parameter("publish_right_rectified", false, descriptor);
    enable_disparity_ = node_.declare_parameter("publish_disparity", false, descriptor);
    lazy_ = node_.declare_parameter("lazy", true, descriptor);
    stride_ = node_.declare_parameter("point_stride", 2, descriptor);
    speed_ = node_.declare_parameter("replay_rate", 1.0, descriptor);
    loop_ = node_.declare_parameter("replay_loop", false, descriptor);
    publish_clock_ =
        node_.declare_parameter("publish_clock", false, descriptor);
    max_age_ = node_.declare_parameter("maximum_frame_age_s", 1.0, descriptor);
    PolicyConfig policy_config;
    policy_config.expected_device_uuid = text("expected_device_uuid", "");
    policy_config.expected_calibration_sha256 = text("expected_calibration_sha256", "");
    policy_config.acquisition_restart_policy = text("acquisition_restart_policy", "accept");
    policy_config.clock_jump_policy = text("clock_jump_policy", "remap");
    policy_config.clock_jump_threshold_s = node_.declare_parameter("clock_jump_threshold_s", 0.1, descriptor);
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
    if (frame_id_.empty() || frame_id_.front() == '/' || right_frame_id_.empty() || right_frame_id_.front() == '/' || stride_ < 1 ||
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
    policy_config.replay = source_kind_ == "replay";
    policy_config.local_boottime = mode_ == "local_boottime";
    policy_config.allow_provisional = allow_provisional_;
    policy_config.maximum_frame_age_s = max_age_;
    policy_ = std::make_unique<FramePolicy>(policy_config);
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
    if (enable_left_) {
      left_ = node_.create_publisher<sensor_msgs::msg::Image>("left/image_rect", qos);
      left_info_ = node_.create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", qos);
    }
    if (enable_right_) {
      right_ = node_.create_publisher<sensor_msgs::msg::Image>("right/image_rect", qos);
      right_info_ = node_.create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", qos);
    }
    if (enable_disparity_)
      disparity_ = node_.create_publisher<stereo_msgs::msg::DisparityImage>("disparity", qos);
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
      {
        std::lock_guard lock(policy_mutex_);
        if (policy_->snapshot(steady_ns()).latched) return;
      }
      if (lazy_ && !has_consumers()) {
        state("waiting for ROS measurement subscribers", 1);
        if (wait(std::chrono::milliseconds(200))) break;
        continue;
      }
      std::shared_ptr<nuvo::client::Source> current;
      bool idle = false;
      bool provisional_blocked = false;
      bool policy_blocked = false;
      try {
        state("connecting", 1);
        subscription_.products = demanded_products();
        current = make_source();
        {
          std::lock_guard lock(source_mutex_);
          source_ = current;
          if (stopped_) current->cancel();
        }
        while (!stopped_) {
          {
            std::lock_guard lock(policy_mutex_);
            policy_blocked = policy_->snapshot(steady_ns()).latched;
          }
          if (policy_blocked) {
            current->cancel();
            break;
          }
          if (source_kind_ != "replay" && subscription_.products != demanded_products()) {
            idle = true;
            current->cancel();
            break;
          }
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
                std::lock_guard lock(policy_mutex_);
                policy_->replay_segment();
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
          const auto clock_sample = mode_ == "local_boottime"
              ? std::optional(nuvo::client::local_clock_sample()) : std::nullopt;
          bool accepted;
          std::string reason;
          {
            std::lock_guard lock(policy_mutex_);
            accepted = policy_->observe(*frame, steady_ns(), clock_sample);
            reason = policy_->snapshot(steady_ns(), clock_sample).reject_reason;
            policy_blocked = policy_->snapshot(steady_ns(), clock_sample).latched;
          }
          if (policy_blocked) {
            state(reason, 2);
            current->cancel();
            break;
          }
          if (override_ && !frame->matching_geometry && (needs_depth() || consumers(disparity_)))
            nuvo::client::apply_calibration_override(*frame, *override_);
          if (!accepted && source_kind_ != "replay") {
            state(reason, 1);
            // Release native matcher demand while publication is blocked.
            // Probe periodically so a validated calibration can recover.
            if (reason.starts_with("provisional calibration")) {
              provisional_blocked = true;
              current->cancel();
              break;
            }
            continue;
          }
          if (publish(*frame, receipt_stamp, clock_sample, accepted)) {
            failures = 0;
          }
        }
      } catch (const std::exception& error) {
        if (!stopped_) {
          ++errors_;
          {
            std::lock_guard lock(policy_mutex_);
            policy_->reject(error.what());
          }
          state(error.what(), 2);
          RCLCPP_WARN(node_.get_logger(), "Acquisition recovery: %s",
                      error.what());
        }
      }
      clear_source();
      current.reset();
      if (stopped_) break;
      if (policy_blocked) return;
      if (provisional_blocked) {
        if (wait(std::chrono::seconds(5))) break;
        continue;
      }
      if (idle) {
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
    return needs_depth() || consumers(left_) || consumers(left_info_) ||
        consumers(right_) || consumers(right_info_) || consumers(disparity_);
  }
  template<class T> static bool consumers(const std::shared_ptr<T>& publisher) {
    return publisher && (publisher->get_subscription_count() + publisher->get_intra_process_subscription_count() > 0);
  }
  bool needs_depth() const {
    return !lazy_ || consumers(depth_) || consumers(validity_) || consumers(info_) || consumers(metadata_) || consumers(points_);
  }
  std::vector<std::string> demanded_products() const {
    std::vector<std::string> result;
    if (needs_depth()) result = {"depth", "validity"};
    if (consumers(left_) || consumers(left_info_)) result.push_back("left_rectified");
    if (consumers(right_) || consumers(right_info_)) result.push_back("right_rectified");
    if (consumers(disparity_)) {
      result.push_back("disparity");
      if (std::find(result.begin(), result.end(), "validity") == result.end()) result.push_back("validity");
    }
    // The outer lazy gate avoids acquiring with no consumers. A nonempty
    // fallback handles DDS count changes between the gate and connection.
    if (result.empty()) result = {"depth", "validity"};
    return result;
  }
  static std::int64_t steady_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  bool requested(const std::string& product) const {
    return source_kind_ == "replay" || std::find(subscription_.products.begin(), subscription_.products.end(), product) != subscription_.products.end();
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
  bool publish(const nuvo::client::Frame& frame, std::int64_t receipt_stamp,
               const std::optional<nuvo::client::ClockSample>& clock_sample,
               bool accepted) {
    // builtin_interfaces/Time has a signed 32-bit seconds field.
    constexpr std::int64_t maximum_ros_stamp = 2147483647999999999LL;
    std::int64_t stamp{}, uncertainty = -1;
    if (mode_ == "local_boottime") {
      const auto sample = *clock_sample;
      stamp = mapped_capture_stamp_ns(frame.capture_time_ns, sample);
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
    if (!accepted) return false;
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(stamp);
    header.frame_id = frame_id_;
    unsigned available = 0;
    std::optional<Images> images;
    std::optional<sensor_msgs::msg::PointCloud2> point_cloud;
    std::optional<RectifiedImage> left_image, right_image;
    std::optional<stereo_msgs::msg::DisparityImage> disparity_image;
    std::string product_error;
    auto prepare = [&](const char* name, auto convert_product) {
      try { convert_product(); }
      catch (const std::invalid_argument& error) {
        if (!product_error.empty()) product_error += "; ";
        product_error += std::string(name) + ": " + error.what();
      }
    };
    if (needs_depth() && requested("depth")) {
      prepare("depth", [&] {
        auto prepared_images = convert(frame, header);
        auto prepared_points = consumers(points_)
            ? std::optional(cloud(prepared_images, static_cast<unsigned>(stride_))) : std::nullopt;
        images = std::move(prepared_images);
        point_cloud = std::move(prepared_points);
        available |= 1;
      });
    }
    if ((consumers(left_) || consumers(left_info_)) && requested("left_rectified")) {
      prepare("left_rectified", [&] { left_image = rectified_image(frame, header); available |= 2; });
    }
    if ((consumers(right_) || consumers(right_info_)) && requested("right_rectified")) {
      auto right_header = header;
      right_header.frame_id = right_frame_id_;
      prepare("right_rectified", [&] { right_image = rectified_image(frame, right_header, true); available |= 4; });
    }
    if (consumers(disparity_) && requested("disparity")) {
      prepare("disparity", [&] { disparity_image = disparity(frame, header); available |= 8; });
    }
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
    {
      std::lock_guard lock(policy_mutex_);
      product_error_ = product_error;
      if (!available) {
        policy_->reject("no_usable_measurement_products");
        return false;
      }
      const auto sample = mode_ == "local_boottime"
          ? std::optional(nuvo::client::local_clock_sample()) : std::nullopt;
      if (!policy_->publication_allowed(frame, steady_ns(), sample)) return false;
      if (sample) {
        stamp = mapped_capture_stamp_ns(frame.capture_time_ns, *sample);
        const builtin_interfaces::msg::Time final_stamp = rclcpp::Time(stamp);
        metadata.header.stamp = final_stamp;
        metadata.timestamp_uncertainty_ns = sample->uncertainty_ns;
        if (images) {
          images->depth.header.stamp = final_stamp;
          images->validity.header.stamp = final_stamp;
          images->camera_info.header.stamp = final_stamp;
        }
        if (point_cloud) point_cloud->header.stamp = final_stamp;
        for (auto* image : {&left_image, &right_image}) if (*image) {
          (*image)->image.header.stamp = final_stamp;
          (*image)->camera_info.header.stamp = final_stamp;
        }
        if (disparity_image) {
          disparity_image->header.stamp = final_stamp;
          disparity_image->image.header.stamp = final_stamp;
        }
      }
      if (clock_) {
        rosgraph_msgs::msg::Clock message;
        message.clock = metadata.header.stamp;
        clock_->publish(message);
      }
      if (images) {
        if (point_cloud) points_->publish(std::move(*point_cloud));
        info_->publish(images->camera_info);
        validity_->publish(std::move(images->validity));
        depth_->publish(std::move(images->depth));
      }
      if (left_image) {
        left_info_->publish(left_image->camera_info);
        left_->publish(std::move(left_image->image));
      }
      if (right_image) {
        right_info_->publish(right_image->camera_info);
        right_->publish(std::move(right_image->image));
      }
      if (disparity_image) disparity_->publish(std::move(*disparity_image));
      metadata_->publish(metadata);
      policy_->mark_published(frame, steady_ns());
      available_products_ = available;
    }
    if (++published_ == 1)
      RCLCPP_INFO(node_.get_logger(), "First associated depth frame published");
    last_stamp_ = stamp;
    state(frame.calibration_authority == 0
              ? "streaming with provisional calibration"
          : mode_ == "receipt" ? "streaming with receipt timestamps"
                               : "streaming",
          frame.calibration_authority == 0 || mode_ == "receipt"
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
    PolicyState policy_state;
    unsigned available;
    std::string product_error;
    {
      std::lock_guard lock(policy_mutex_);
      const auto sample = mode_ == "local_boottime"
          ? std::optional(nuvo::client::local_clock_sample()) : std::nullopt;
      policy_state = policy_->snapshot(steady_ns(), sample);
      available = available_products_.load();
      product_error = product_error_;
    }
    if (policy_state.latched) {
      std::lock_guard lock(source_mutex_);
      if (source_) source_->cancel();
    }
    if (!policy_state.reject_reason.empty()) {
      status.level = policy_state.identity_fault || policy_state.clock_fault || policy_state.latched ? 2 : 1;
      status.message = policy_state.reject_reason;
    } else if (!product_error.empty()) {
      status.level = 1;
      status.message = "some requested products unavailable";
    } else if (policy_state.clock_fault) {
      status.level = 1;
      status.message = "system clock jump remapped";
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
    add("duplicate_frames", std::to_string(policy_state.duplicates));
    add("stale_frames", std::to_string(policy_state.stale));
    add("clock_jumps", std::to_string(policy_state.clock_jumps));
    add("rejected_frames", std::to_string(policy_state.rejected));
    add("acquisition_restarts", std::to_string(policy_state.restarts));
    add("source_age_s", std::to_string(policy_state.source_age_s));
    add("acquisition_age_s", std::to_string(policy_state.source_age_s));
    add("source_age_available", policy_state.source_age_s >= 0 ? "true" : "false");
    add("observation_age_s", std::to_string(policy_state.observation_age_s));
    add("publication_age_s", std::to_string(policy_state.publication_age_s));
    add("calibration_qualified", policy_state.calibration_qualified ? "true" : "false");
    add("calibration_sha256", policy_state.calibration_sha256);
    add("calibration_authority", std::to_string(policy_state.calibration_authority));
    if (policy_state.depth_constraints) {
      add("depth_minimum_m", std::to_string(policy_state.depth_constraints->minimum_m));
      add("depth_maximum_m", std::to_string(policy_state.depth_constraints->maximum_m));
    }
    if (policy_state.disparity_constraints) {
      add("disparity_accepted_min_px", std::to_string(policy_state.disparity_constraints->accepted_min_px));
      add("disparity_accepted_max_px", std::to_string(policy_state.disparity_constraints->accepted_max_px));
      add("disparity_quantization_px", std::to_string(policy_state.disparity_constraints->quantization_px));
    }
    add("identity_fault", policy_state.identity_fault ? "true" : "false");
    add("clock_fault", policy_state.clock_fault ? "true" : "false");
    add("fault_latched", policy_state.latched ? "true" : "false");
    add("reject_reason", policy_state.reject_reason);
    add("product_unavailable_reason", product_error);
    add("output_available", policy_state.available ? "true" : "false");
    add("depth_available", policy_state.available && (available & 1) ? "true" : "false");
    add("left_rectified_available", policy_state.available && (available & 2) ? "true" : "false");
    add("right_rectified_available", policy_state.available && (available & 4) ? "true" : "false");
    add("disparity_available", policy_state.available && (available & 8) ? "true" : "false");
    if (policy_state.observed) {
      add("observed_device_uuid", policy_state.observed->device_uuid);
      add("observed_run_uuid", policy_state.observed->run_uuid);
      add("observed_capture_sequence", std::to_string(policy_state.observed->capture_sequence));
      add("observed_capture_epoch", std::to_string(policy_state.observed->capture_epoch));
      add("observed_configuration_epoch", std::to_string(policy_state.observed->configuration_epoch));
    }
    if (policy_state.published) {
      add("published_device_uuid", policy_state.published->device_uuid);
      add("published_run_uuid", policy_state.published->run_uuid);
      add("published_capture_sequence", std::to_string(policy_state.published->capture_sequence));
    }
    message.status.push_back(status);
    diagnostics_->publish(message);
  }
  NuvoCamera& node_;
  std::string source_kind_, endpoint_, recording_, frame_id_, right_frame_id_, mode_, ca_,
      credential_, connect_address_;
  nuvo::client::Subscription subscription_;
  std::optional<nuvo::client::CalibrationOverride> override_;
  bool allow_provisional_{}, enable_points_{}, enable_left_{}, enable_right_{}, enable_disparity_{}, lazy_{}, loop_{},
      publish_clock_{}, segment_{};
  int stride_{};
  double speed_{}, max_age_{};
  std::unique_ptr<FramePolicy> policy_;
  std::int64_t source_base_{}, ros_base_{}, last_source_time_{}, last_stamp_{};
  std::string replay_run_;
  std::chrono::steady_clock::time_point steady_base_;
  std::atomic_bool stopped_{};
  std::atomic_uint64_t published_{}, errors_{};
  std::atomic_uint available_products_{};
  std::mutex source_mutex_, status_mutex_, wait_mutex_, policy_mutex_;
  std::condition_variable wake_;
  std::shared_ptr<nuvo::client::Source> source_;
  std::string status_{"starting"};
  std::string product_error_;
  unsigned level_{1};
  std::thread worker_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_, validity_, left_, right_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_, left_info_, right_info_;
  rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr disparity_;
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
