#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
namespace nuvo_camera {
class NuvoCamera final : public rclcpp::Node {
 public:
  explicit NuvoCamera(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~NuvoCamera() override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace nuvo_camera
