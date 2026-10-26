#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <nuvo_camera/camera.hpp>

class CameraParameters : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  static rclcpp::NodeOptions options(
      std::vector<rclcpp::Parameter> parameters) {
    rclcpp::NodeOptions result;
    result.parameter_overrides(std::move(parameters));
    return result;
  }
};

TEST_F(CameraParameters, RejectsNonfiniteTimingParametersBeforeAcquisition) {
  for (const auto* name : {"cadence_hz", "replay_rate", "maximum_frame_age_s"})
    for (const auto value : {std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity(), 0., -1.})
      EXPECT_THROW(nuvo_camera::NuvoCamera(
                       options({rclcpp::Parameter(name, value)})),
                   std::invalid_argument)
          << name << "=" << value;
}

TEST_F(CameraParameters, RejectsIncompatibleTimeSources) {
  EXPECT_THROW(nuvo_camera::NuvoCamera(options({
                   rclcpp::Parameter("source", "https"),
                   rclcpp::Parameter("timestamp_mode", "local_boottime")})),
               std::invalid_argument);
  EXPECT_THROW(nuvo_camera::NuvoCamera(
                   options({rclcpp::Parameter("use_sim_time", true)})),
               std::invalid_argument);
  EXPECT_THROW(nuvo_camera::NuvoCamera(options({
                   rclcpp::Parameter("source", "replay"),
                   rclcpp::Parameter("recording", "unused.nvr"),
                   rclcpp::Parameter("use_sim_time", true)})),
               std::invalid_argument);
}

TEST_F(CameraParameters, KeepsSimulatedTimeFixedAfterStartup) {
  nuvo_camera::NuvoCamera node;
  const auto result = node.set_parameter(rclcpp::Parameter("use_sim_time", true));
  EXPECT_FALSE(result.successful);
  EXPECT_FALSE(node.get_parameter("use_sim_time").as_bool());
  EXPECT_TRUE(node.set_parameter(rclcpp::Parameter("use_sim_time", false))
                  .successful);
  nuvo_camera::NuvoCamera replay(options({
      rclcpp::Parameter("source", "replay"),
      rclcpp::Parameter("recording", "unused.nvr")}));
  EXPECT_FALSE(replay.set_parameter(rclcpp::Parameter("use_sim_time", true))
                   .successful);
  EXPECT_FALSE(replay.get_parameter("use_sim_time").as_bool());
}

TEST_F(CameraParameters, EnforcesRemoteCadenceWithinItsMessageDeadline) {
  for (const auto cadence : {.001, std::nextafter(1.0 / 60.0, 0.0)})
    EXPECT_THROW(nuvo_camera::NuvoCamera(options({
                     rclcpp::Parameter("source", "https"),
                     rclcpp::Parameter("cadence_hz", cadence)})),
                 std::invalid_argument);
  EXPECT_NO_THROW(nuvo_camera::NuvoCamera(options({
      rclcpp::Parameter("source", "https"),
      rclcpp::Parameter("cadence_hz", 1.0 / 60.0)})));
  EXPECT_NO_THROW(nuvo_camera::NuvoCamera(
      options({rclcpp::Parameter("cadence_hz", .001)})));
}
