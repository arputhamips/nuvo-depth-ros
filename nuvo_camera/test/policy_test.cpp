#include <gtest/gtest.h>
#include <limits>
#include "nuvo_camera/policy.hpp"
using nuvo_camera::FramePolicy;
using nuvo_camera::PolicyConfig;
namespace {
constexpr std::int64_t second = 1000000000LL;
nuvo::client::Frame frame(std::uint64_t sequence = 1) {
  nuvo::client::Frame f;
  f.identity = {std::string(32, '1'), std::string(32, '2'), 1, sequence, 1};
  f.calibration_sha256 = std::string(64, '3');
  f.calibration_authority = 1;
  f.capture_time_ns = 10 * second + static_cast<std::int64_t>(sequence);
  f.capture_clock = "boottime";
  return f;
}
nuvo::client::ClockSample sample_clock(std::int64_t step = 0) {
  return {10 * second + 1000, 100 * second + 1000 + step, 1};
}
bool accept(FramePolicy& policy, const nuvo::client::Frame& f, std::int64_t now,
            std::optional<nuvo::client::ClockSample> clock = {}) {
  if (!policy.observe(f, now, clock) || !policy.publication_allowed(f, now, clock)) return false;
  policy.mark_published(f, now);
  return true;
}
}
TEST(FramePolicy, PinsAreOptionalValidatedAndCheckedOnEveryObservation) {
  PolicyConfig c;
  c.expected_device_uuid = "bad";
  EXPECT_THROW(FramePolicy{c}, std::invalid_argument);
  c.expected_device_uuid = std::string(32, '1');
  c.expected_calibration_sha256 = std::string(64, '3');
  FramePolicy policy(c);
  auto f = frame();
  ASSERT_TRUE(accept(policy, f, second));
  policy.mark_published(f, second);
  f = frame(2);
  f.calibration_sha256 = std::string(64, '4');
  EXPECT_FALSE(policy.observe(f, 2 * second));
  auto s = policy.snapshot(2 * second);
  EXPECT_EQ(s.reject_reason, "calibration_pin_mismatch");
  EXPECT_EQ(s.observed->capture_sequence, 2U);
  EXPECT_EQ(s.published->capture_sequence, 1U);
  EXPECT_FALSE(s.available);
  // A pin mismatch cannot be repaired in place under the same capture ID.
  f.calibration_sha256 = std::string(64, '3');
  EXPECT_FALSE(policy.observe(f, 2 * second));
  EXPECT_TRUE(accept(policy, frame(3), 3 * second));
  FramePolicy wrong_device(c);
  f = frame(); f.identity.device_uuid = std::string(32, '4');
  EXPECT_FALSE(wrong_device.observe(f, second));
  EXPECT_EQ(wrong_device.snapshot(second).reject_reason, "device_pin_mismatch");
  EXPECT_TRUE(accept(wrong_device, frame(2), 2 * second));
}
TEST(FramePolicy, TracksObservedIdentityEvenWhenCalibrationIsUnqualified) {
  FramePolicy p({});
  auto f = frame(); f.calibration_authority = 0;
  EXPECT_FALSE(p.observe(f, second));
  f.calibration_authority = 1;
  EXPECT_FALSE(p.observe(f, 2 * second));
  EXPECT_TRUE(p.observe(frame(2), 3 * second));
  EXPECT_FALSE(p.snapshot(3 * second).published.has_value());
}
TEST(FramePolicy, RejectsRegressionsAndKeepsWatermarksAcrossTransportAndIdleTime) {
  FramePolicy p({});
  ASSERT_TRUE(accept(p, frame(5), second));
  for (auto f : {frame(4), frame(5)}) EXPECT_FALSE(p.observe(f, 50 * second));
  auto f = frame(6); f.identity.capture_epoch = 0;
  EXPECT_FALSE(p.observe(f, 51 * second));
  f = frame(6); f.capture_time_ns = frame(4).capture_time_ns;
  EXPECT_FALSE(p.observe(f, 52 * second));
  EXPECT_FALSE(p.observe(frame(6), 53 * second));
  ASSERT_TRUE(accept(p, frame(7), 53 * second));
  f = frame(8); f.identity.configuration_epoch = 2;
  ASSERT_TRUE(accept(p, f, 54 * second));
  EXPECT_FALSE(p.observe(frame(9), 55 * second));
  f = frame(10); f.identity.configuration_epoch = 2;
  EXPECT_TRUE(accept(p, f, 56 * second));
}
TEST(FramePolicy, AcceptsExplicitNewGenerationsAndRejectsRetiredRuns) {
  PolicyConfig c; c.local_boottime = true;
  FramePolicy p(c);
  ASSERT_TRUE(accept(p, frame(5), second, sample_clock()));
  auto f = frame(6); f.identity.run_uuid = std::string(32, '4'); f.identity.capture_sequence = 1;
  EXPECT_TRUE(accept(p, f, 2 * second, sample_clock()));
  EXPECT_FALSE(p.observe(frame(7), 3 * second, sample_clock()));
  f = frame(8); f.identity.run_uuid = std::string(32, '4'); f.identity.capture_epoch = 2; f.identity.capture_sequence = 1;
  EXPECT_TRUE(accept(p, f, 4 * second, sample_clock()));
  EXPECT_EQ(p.snapshot(4 * second, sample_clock()).restarts, 2U);
  c.acquisition_restart_policy = "reject";
  FramePolicy strict(c);
  ASSERT_TRUE(accept(strict, frame(), second, sample_clock()));
  EXPECT_FALSE(strict.observe(f, 2 * second, sample_clock()));
  EXPECT_TRUE(strict.snapshot(2 * second, sample_clock()).latched);
  EXPECT_FALSE(strict.observe(frame(3), 3 * second, sample_clock()));
}
TEST(FramePolicy, ReceiptAllowsNewRunAfterRemoteRebootButSameRunTimeMustAdvance) {
  FramePolicy p({});
  ASSERT_TRUE(accept(p, frame(), second));
  auto f = frame(2); f.capture_time_ns = 1;
  EXPECT_FALSE(p.observe(f, 2 * second));
  f.identity.run_uuid = std::string(32, '4');
  EXPECT_TRUE(accept(p, f, 3 * second));
}
TEST(FramePolicy, DeviceReplacementLatchesUntilNodeRestart) {
  FramePolicy p({});
  ASSERT_TRUE(accept(p, frame(), second));
  auto f = frame(2); f.identity.device_uuid = std::string(32, '4');
  EXPECT_FALSE(p.observe(f, 2 * second));
  EXPECT_TRUE(p.snapshot(2 * second).latched);
  EXPECT_FALSE(p.observe(frame(3), 3 * second));
}
TEST(FramePolicy, FutureCaptureCannotBecomeFreshByRepeatingIt) {
  PolicyConfig c; c.local_boottime = true;
  FramePolicy p(c);
  auto f = frame(); f.capture_time_ns = 100 * second;
  EXPECT_FALSE(p.observe(f, second, sample_clock()));
  EXPECT_FALSE(p.observe(f, 2 * second, {nuvo::client::ClockSample{101 * second, 201 * second, 1}}));
  EXPECT_TRUE(accept(p, frame(2), 3 * second, sample_clock()));
}
TEST(FramePolicy, MalformedFutureGenerationDoesNotPoisonAcceptedContinuity) {
  PolicyConfig c; c.local_boottime = true;
  FramePolicy p(c);
  ASSERT_TRUE(accept(p, frame(), second, sample_clock()));
  auto f = frame(1000000); f.capture_time_ns = 100 * second;
  f.identity.capture_epoch = 1000000; f.identity.configuration_epoch = 1000000;
  EXPECT_FALSE(p.observe(f, 2 * second, sample_clock()));
  EXPECT_TRUE(accept(p, frame(2), 3 * second, sample_clock()));
}
TEST(FramePolicy, NoFrameWaitsAndDuplicatesNeverRefreshPublishedHealth) {
  PolicyConfig c; c.local_boottime = true; c.maximum_frame_age_s = .5;
  FramePolicy p(c);
  ASSERT_TRUE(accept(p, frame(), second, sample_clock())); p.mark_published(frame(), second);
  EXPECT_TRUE(p.snapshot(second, sample_clock()).available);
  EXPECT_FALSE(p.observe(frame(), second + 400000000, sample_clock()));
  auto later = sample_clock(); later.boottime_ns += second; later.system_ns += second;
  const auto s = p.snapshot(2 * second, later);
  EXPECT_FALSE(s.available);
  EXPECT_DOUBLE_EQ(s.publication_age_s, 1.);
  EXPECT_DOUBLE_EQ(s.observation_age_s, 1.);
  EXPECT_GT(s.source_age_s, .5);
}
TEST(FramePolicy, TimerClockCheckCanLatchWithoutAnyNewFrames) {
  PolicyConfig c; c.local_boottime = true; c.clock_jump_policy = "latch";
  FramePolicy p(c);
  ASSERT_TRUE(accept(p, frame(), second, sample_clock())); p.mark_published(frame(), second);
  auto s = p.snapshot(second + 1, sample_clock(second));
  EXPECT_TRUE(s.clock_fault); EXPECT_TRUE(s.latched); EXPECT_FALSE(s.available);
  EXPECT_EQ(s.reject_reason, "clock_jump");
  EXPECT_FALSE(p.observe(frame(2), second + 2, sample_clock()));
  FramePolicy backwards(c);
  ASSERT_TRUE(accept(backwards, frame(), second, sample_clock()));
  const auto backward_fault = backwards.snapshot(second + 1, sample_clock(-second));
  EXPECT_TRUE(backward_fault.clock_fault);
  EXPECT_TRUE(backward_fault.latched);
  EXPECT_FALSE(backward_fault.available);
}
TEST(FramePolicy, RejectClockPolicyRecoversOnlyOnANewFrame) {
  PolicyConfig c; c.local_boottime = true; c.clock_jump_policy = "reject";
  FramePolicy p(c);
  ASSERT_TRUE(accept(p, frame(), second, sample_clock())); p.mark_published(frame(), second);
  EXPECT_FALSE(p.snapshot(second + 1, sample_clock(second)).available);
  EXPECT_FALSE(p.snapshot(second + 2, sample_clock(second)).available);
  ASSERT_TRUE(accept(p, frame(2), second + 3, sample_clock(second)));
  p.mark_published(frame(2), second + 3);
  EXPECT_TRUE(p.snapshot(second + 4, sample_clock(second)).available);
}
TEST(FramePolicy, ReplaySegmentsHaveSeparateContinuityAndNoClaimOfSourceAge) {
  PolicyConfig c; c.replay = true;
  FramePolicy p(c);
  ASSERT_TRUE(accept(p, frame(5), second));
  EXPECT_FALSE(p.observe(frame(5), 2 * second));
  p.replay_segment();
  EXPECT_TRUE(accept(p, frame(1), 3 * second));
  p.mark_published(frame(1), 3 * second);
  EXPECT_EQ(p.snapshot(3 * second).source_age_s, -1.);
}
TEST(FramePolicy, FailedConversionCannotAdvanceAcceptedCursor) {
  FramePolicy p({});
  ASSERT_TRUE(accept(p, frame(), second));
  ASSERT_TRUE(p.observe(frame(1000), 2 * second));
  p.reject("invalid_geometry");
  EXPECT_EQ(p.snapshot(2 * second).published->capture_sequence, 1U);
  EXPECT_TRUE(accept(p, frame(2), 3 * second));
}
TEST(FramePolicy, TimerLatchBetweenObservationAndPublicationCannotBeOverwritten) {
  PolicyConfig c; c.local_boottime = true; c.clock_jump_policy = "latch";
  FramePolicy p(c);
  ASSERT_TRUE(accept(p, frame(), second, sample_clock()));
  ASSERT_TRUE(p.observe(frame(2), second + 1, sample_clock()));
  ASSERT_TRUE(p.snapshot(second + 2, sample_clock(second)).latched);
  EXPECT_FALSE(p.publication_allowed(frame(2), second + 3, sample_clock(second)));
  p.mark_published(frame(2), second + 3);
  EXPECT_EQ(p.snapshot(second + 4).published->capture_sequence, 1U);
  EXPECT_FALSE(p.snapshot(second + 4).available);
}
TEST(FramePolicy, CaptureMappingChecksBeforeAnySignedAddition) {
  const auto maximum = std::numeric_limits<std::int64_t>::max();
  EXPECT_THROW(nuvo_camera::mapped_capture_stamp_ns(101, {100, maximum, 1}), std::out_of_range);
  EXPECT_THROW(nuvo_camera::mapped_capture_stamp_ns(maximum, {100, maximum, 1}), std::out_of_range);
  EXPECT_EQ(nuvo_camera::mapped_capture_stamp_ns(101, {100, 1000, 1}), 1001);
  EXPECT_EQ(nuvo_camera::mapped_capture_stamp_ns(100, {100, nuvo_camera::maximum_ros_stamp_ns, 1}), nuvo_camera::maximum_ros_stamp_ns);
}
