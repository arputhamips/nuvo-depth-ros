#include "nuvo_camera/policy.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nuvo_camera {
namespace {
bool hex(const std::string& value, std::size_t size) {
  return value.size() == size && value.find_first_not_of('0') != std::string::npos &&
      std::all_of(value.begin(), value.end(), [](char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
      });
}
double age(std::int64_t now, std::int64_t then) {
  return then > 0 && now >= then ? static_cast<double>(now - then) * 1e-9 : -1.;
}
}
std::int64_t mapped_capture_stamp_ns(std::int64_t capture, const nuvo::client::ClockSample& sample) {
  const auto mapped = static_cast<long double>(capture) + sample.system_ns - sample.boottime_ns;
  if (capture <= 0 || sample.system_ns <= 0 || sample.boottime_ns <= 0 ||
      mapped <= 0 || mapped > maximum_ros_stamp_ns)
    throw std::out_of_range("ROS acquisition stamp is outside supported range");
  const auto stamp = capture + (sample.system_ns - sample.boottime_ns);
  if (stamp <= 0 || stamp > maximum_ros_stamp_ns)
    throw std::out_of_range("ROS acquisition stamp is outside supported range");
  return stamp;
}
FramePolicy::FramePolicy(PolicyConfig config) : config_(std::move(config)) {
  if ((!config_.expected_device_uuid.empty() && !hex(config_.expected_device_uuid, 32)) ||
      (!config_.expected_calibration_sha256.empty() && !hex(config_.expected_calibration_sha256, 64)))
    throw std::invalid_argument("expected identity pins must be nonzero lowercase hex (UUID 32, SHA256 64)");
  if (config_.acquisition_restart_policy != "accept" && config_.acquisition_restart_policy != "reject")
    throw std::invalid_argument("acquisition_restart_policy must be accept or reject");
  if (config_.clock_jump_policy != "remap" && config_.clock_jump_policy != "reject" && config_.clock_jump_policy != "latch")
    throw std::invalid_argument("clock_jump_policy must be remap, reject or latch");
  if (!std::isfinite(config_.maximum_frame_age_s) || config_.maximum_frame_age_s <= 0 ||
      !std::isfinite(config_.clock_jump_threshold_s) || config_.clock_jump_threshold_s <= 0)
    throw std::invalid_argument("policy timing thresholds must be finite and positive");
}
bool FramePolicy::fail(const std::string& reason, bool identity, bool latch) {
  state_.available = false;
  if (!state_.latched) state_.reject_reason = reason;
  state_.identity_fault = state_.identity_fault || identity;
  state_.latched = state_.latched || latch;
  ++state_.rejected;
  return false;
}
void FramePolicy::reject(const std::string& reason) { (void)fail(reason); }
void FramePolicy::pause() { state_.available = false; }
bool FramePolicy::check_clock(const nuvo::client::ClockSample& sample) {
  if (state_.latched) return false;
  if (!config_.local_boottime) return !state_.latched;
  if (sample.boottime_ns <= 0 || sample.system_ns <= 0)
    return fail("invalid_clock_sample");
  const auto offset = sample.system_ns - sample.boottime_ns;
  const bool jumped = clock_offset_ &&
      std::abs(static_cast<long double>(offset) - *clock_offset_) > config_.clock_jump_threshold_s * 1e9;
  if (jumped) {
    ++state_.clock_jumps;
    state_.clock_fault = true;
    if (config_.clock_jump_policy != "latch") clock_offset_ = offset;
    if (config_.clock_jump_policy != "remap")
      return fail("clock_jump", false, config_.clock_jump_policy == "latch");
  } else {
    clock_offset_ = offset;
  }
  return !state_.latched;
}
bool FramePolicy::observe(const nuvo::client::Frame& frame, std::int64_t now,
                          std::optional<nuvo::client::ClockSample> clock) {
  const auto& id = frame.identity;
  state_.observed = id;
  state_.calibration_sha256 = frame.calibration_sha256;
  state_.calibration_authority = frame.calibration_authority;
  state_.depth_constraints = frame.depth_constraints;
  state_.disparity_constraints = frame.disparity_constraints;
  state_.calibration_qualified = frame.calibration_authority == 1 || frame.calibration_authority == 2;
  // Duplicate packets do not refresh the acquisition observation deadline.
  if (std::find(recent_observations_.begin(), recent_observations_.end(), id) != recent_observations_.end()) {
    ++state_.duplicates;
    return fail("duplicate_capture");
  }
  observed_steady_ = now;
  observed_capture_ = frame.capture_time_ns;
  if (state_.latched) return fail(state_.reject_reason);
  if (!hex(id.device_uuid, 32) || !hex(id.run_uuid, 32) || !hex(frame.calibration_sha256, 64) ||
      !id.capture_epoch || !id.configuration_epoch || !id.capture_sequence || frame.capture_time_ns <= 0)
    return fail("invalid_identity", true);
  // Remember rejected observations separately. They cannot be repaired in
  // place, and cannot advance the accepted acquisition high-water mark.
  recent_observations_.push_back(id);
  if (recent_observations_.size() > 128) recent_observations_.pop_front();
  if (continuity_ && id.device_uuid != continuity_->device_uuid) return fail("device_changed", true, true);
  if (!config_.expected_device_uuid.empty() && id.device_uuid != config_.expected_device_uuid)
    return fail("device_pin_mismatch", true);
  bool restart = false, new_run = false;
  if (continuity_) {
    const auto& previous = *continuity_;
    if (id.device_uuid != previous.device_uuid) return fail("device_changed", true, true);
    if (!config_.replay) {
      const bool same_run = id.run_uuid == previous.run_uuid;
      const bool same_capture = same_run && id.capture_epoch == previous.capture_epoch;
      if ((!same_run && std::find(retired_runs_.begin(), retired_runs_.end(), id.run_uuid) != retired_runs_.end()) ||
          (same_run && (id.capture_epoch < previous.capture_epoch || id.configuration_epoch < previous.configuration_epoch)) ||
          (same_capture && id.capture_sequence <= previous.capture_sequence) ||
          ((same_run || config_.local_boottime) && frame.capture_time_ns <= last_capture_))
        return fail("acquisition_regressed", true);
      restart = !same_run || id.capture_epoch != previous.capture_epoch ||
          id.configuration_epoch != previous.configuration_epoch;
      new_run = !same_run;
    }
  }
  state_.identity_fault = false;
  if (clock && !check_clock(*clock)) return false;
  if (config_.local_boottime && clock) {
    const auto source_age = static_cast<long double>(clock->boottime_ns) - frame.capture_time_ns;
    if (source_age < -5000000.L || source_age > config_.maximum_frame_age_s * 1e9) {
      ++state_.stale;
      return fail("stale_or_future_capture");
    }
  }
  if (!config_.expected_calibration_sha256.empty() && frame.calibration_sha256 != config_.expected_calibration_sha256)
    return fail("calibration_pin_mismatch");
  if (!state_.calibration_qualified && !(frame.calibration_authority == 0 && config_.allow_provisional))
    return fail("provisional calibration: publication requires explicit allow_provisional_calibration");
  if (restart && config_.acquisition_restart_policy == "reject") return fail("acquisition_restart", true, true);
  pending_restart_ = restart;
  pending_new_run_ = new_run;
  state_.reject_reason.clear();
  state_.clock_fault = false;
  return true;
}
bool FramePolicy::publication_allowed(const nuvo::client::Frame& frame, std::int64_t now,
                                      std::optional<nuvo::client::ClockSample> clock) {
  if (state_.latched || !state_.reject_reason.empty()) return false;
  if (clock && !check_clock(*clock)) return false;
  if (config_.local_boottime && clock) {
    const auto source_age = static_cast<long double>(clock->boottime_ns) - frame.capture_time_ns;
    if (source_age < -5000000.L || source_age > config_.maximum_frame_age_s * 1e9) {
      ++state_.stale;
      return fail("stale_or_future_capture");
    }
  } else if (!config_.replay && age(now, observed_steady_) > config_.maximum_frame_age_s) {
    return fail("stale_receipt");
  }
  return true;
}
void FramePolicy::mark_published(const nuvo::client::Frame& frame, std::int64_t now) {
  if (state_.latched || !state_.reject_reason.empty()) return;
  if (pending_restart_) {
    ++state_.restarts;
    if (pending_new_run_ && continuity_) {
      retired_runs_.push_back(continuity_->run_uuid);
      if (retired_runs_.size() > 32) retired_runs_.pop_front();
    }
  }
  continuity_ = frame.identity;
  last_capture_ = frame.capture_time_ns;
  pending_restart_ = pending_new_run_ = false;
  state_.published = frame.identity;
  published_steady_ = now;
  state_.available = !state_.latched;
}
void FramePolicy::replay_segment() {
  if (config_.replay) {
    continuity_.reset();
    recent_observations_.clear();
  }
}
PolicyState FramePolicy::snapshot(std::int64_t now, std::optional<nuvo::client::ClockSample> clock) {
  if (clock) (void)check_clock(*clock);
  auto out = state_;
  out.observation_age_s = age(now, observed_steady_);
  out.publication_age_s = age(now, published_steady_);
  if (config_.local_boottime && clock) out.source_age_s = age(clock->boottime_ns, observed_capture_);
  if (out.available && (out.publication_age_s < 0 || out.publication_age_s > config_.maximum_frame_age_s ||
      (config_.local_boottime && (out.source_age_s < 0 || out.source_age_s > config_.maximum_frame_age_s)))) {
    out.available = false;
    out.reject_reason = "stale_acquisition";
  }
  return out;
}
}  // namespace nuvo_camera
