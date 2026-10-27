#pragma once
#include <nuvo/client.hpp>
#include <optional>
#include <deque>
#include <string>

namespace nuvo_camera {
inline constexpr std::int64_t maximum_ros_stamp_ns = 2147483647999999999LL;
// Checked before signed addition; builtin_interfaces/Time has int32 seconds.
std::int64_t mapped_capture_stamp_ns(std::int64_t capture_time_ns,
                                    const nuvo::client::ClockSample&);
struct PolicyConfig {
  bool replay{}, local_boottime{}, allow_provisional{};
  std::string expected_calibration_sha256, expected_device_uuid;
  std::string acquisition_restart_policy{"accept"}, clock_jump_policy{"remap"};
  double maximum_frame_age_s{1.0}, clock_jump_threshold_s{0.1};
};
struct PolicyState {
  std::optional<nuvo::client::Identity> observed, published;
  std::string calibration_sha256, reject_reason;
  unsigned calibration_authority{};
  std::optional<nuvo::client::DepthConstraints> depth_constraints;
  std::optional<nuvo::client::DisparityConstraints> disparity_constraints;
  bool calibration_qualified{}, identity_fault{}, clock_fault{}, latched{}, available{};
  double source_age_s{-1}, observation_age_s{-1}, publication_age_s{-1};
  std::uint64_t duplicates{}, rejected{}, restarts{}, clock_jumps{}, stale{};
};
// Acquisition state survives transport reconnects and demand pauses. The owner
// serializes calls; all times passed here use a steady clock except ClockSample.
class FramePolicy {
 public:
  explicit FramePolicy(PolicyConfig config);
  bool observe(const nuvo::client::Frame&, std::int64_t steady_ns,
               std::optional<nuvo::client::ClockSample> clock = {});
  bool check_clock(const nuvo::client::ClockSample&);
  bool publication_allowed(const nuvo::client::Frame&, std::int64_t steady_ns,
                           std::optional<nuvo::client::ClockSample> clock = {});
  void mark_published(const nuvo::client::Frame&, std::int64_t steady_ns);
  void reject(const std::string& reason);
  // Release output availability without resetting identity, faults or counters.
  void pause();
  void replay_segment();
  PolicyState snapshot(std::int64_t steady_ns,
                       std::optional<nuvo::client::ClockSample> clock = {});
 private:
  bool fail(const std::string&, bool identity = false, bool latch = false);
  PolicyConfig config_;
  PolicyState state_;
  std::optional<nuvo::client::Identity> continuity_;
  std::deque<nuvo::client::Identity> recent_observations_;
  std::deque<std::string> retired_runs_;
  std::optional<std::int64_t> clock_offset_;
  std::int64_t last_capture_{}, observed_capture_{}, observed_steady_{}, published_steady_{};
  bool pending_restart_{}, pending_new_run_{};
};
}  // namespace nuvo_camera
