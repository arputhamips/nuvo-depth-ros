#pragma once
#include <nuvo/client.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
namespace nuvo_camera {
struct Images {
  sensor_msgs::msg::Image depth, validity;
  sensor_msgs::msg::CameraInfo camera_info;
};
[[nodiscard]] Images convert(const nuvo::client::Frame& frame,
                             const std_msgs::msg::Header& header);
// A calibrated 32FC1 little-endian image, as returned by convert(). Padded
// depth rows are accepted; nonpositive or nonfinite XYZ values become NaN.
[[nodiscard]] sensor_msgs::msg::PointCloud2 cloud(const Images& images,
                                                unsigned stride = 2);
}  // namespace nuvo_camera
