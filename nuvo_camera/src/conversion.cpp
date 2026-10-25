#include "nuvo_camera/conversion.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace nuvo_camera {
namespace {
void write(std::vector<std::uint8_t>& bytes, std::size_t offset, float value) {
  const auto bits = std::bit_cast<std::uint32_t>(value);
  for (unsigned i = 0; i < 4; ++i)
    bytes[offset + i] = static_cast<std::uint8_t>(bits >> (8U * i));
}
float read(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
  return nuvo::client::float32_le(
      reinterpret_cast<const std::byte*>(bytes.data() + offset));
}
}  // namespace
Images convert(const nuvo::client::Frame& frame,
               const std_msgs::msg::Header& header) {
  const auto* depth = frame.find("depth");
  const auto* validity = frame.find("validity");
  if (!depth || !validity || !frame.matching_geometry)
    throw std::invalid_argument(
        "depth, validity and matching calibration geometry are required");
  const auto w = depth->grid.width, h = depth->grid.height;
  const auto& g = *frame.matching_geometry;
  const auto pixels = static_cast<std::size_t>(w) * h;
  if (!w || !h || depth->format != "float32_le" ||
      validity->format != "uint8" || validity->grid.width != w ||
      validity->grid.height != h || g.grid.width != w || g.grid.height != h ||
      depth->bytes.size() != pixels * 4 || validity->bytes.size() != pixels)
    throw std::invalid_argument(
        "measurement products disagree with calibrated grid");
  Images output;
  output.depth.header = output.validity.header = output.camera_info.header =
      header;
  output.depth.width = output.validity.width = output.camera_info.width = w;
  output.depth.height = output.validity.height = output.camera_info.height = h;
  output.depth.encoding = "32FC1";
  output.depth.is_bigendian = false;
  output.depth.step = w * 4;
  output.validity.encoding = "8UC1";
  output.validity.is_bigendian = false;
  output.validity.step = w;
  output.depth.data.resize(pixels * 4);
  output.validity.data.resize(pixels);
  for (std::size_t i = 0; i < pixels; ++i) {
    const auto code = std::to_integer<std::uint8_t>(validity->bytes[i]);
    const auto value = nuvo::client::float32_le(depth->bytes.data() + 4 * i);
    output.validity.data[i] = code;
    write(output.depth.data, 4 * i,
          code == 1 && std::isfinite(value) && value > 0
              ? value
              : std::numeric_limits<float>::quiet_NaN());
  }
  // A virtual, already rectified pinhole camera. These K/D/R values describe
  // this output, never the untransported distorted physical sensor calibration.
  const auto& p = g.left_projection;
  output.camera_info.p = p;
  output.camera_info.k = {p[0], 0, p[2], 0, p[5], p[6], 0, 0, 1};
  output.camera_info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  output.camera_info.distortion_model = "plumb_bob";
  output.camera_info.d = {0, 0, 0, 0, 0};
  return output;
}
sensor_msgs::msg::PointCloud2 cloud(const Images& images, unsigned stride) {
  if (stride < 1 || stride > 16)
    throw std::invalid_argument("point cloud stride must be 1..16");
  sensor_msgs::msg::PointCloud2 out;
  out.header = images.depth.header;
  out.width = (images.depth.width + stride - 1) / stride;
  out.height = (images.depth.height + stride - 1) / stride;
  out.point_step = 12;
  out.row_step = out.width * 12;
  out.is_bigendian = false;
  out.is_dense = false;
  for (unsigned i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = std::string(1, "xyz"[i]);
    f.offset = i * 4;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    out.fields.push_back(f);
  }
  out.data.resize(static_cast<std::size_t>(out.row_step) * out.height);
  const auto& p = images.camera_info.p;
  if (p[0] <= 0 || p[5] <= 0)
    throw std::invalid_argument("invalid XYZ focal lengths");
  for (unsigned row = 0; row < out.height; ++row)
    for (unsigned column = 0; column < out.width; ++column) {
      const auto u = column * stride, v = row * stride;
      const auto z =
          read(images.depth.data,
               (static_cast<std::size_t>(v) * images.depth.width + u) * 4);
      const auto offset =
          (static_cast<std::size_t>(row) * out.width + column) * 12;
      write(out.data, offset, static_cast<float>((u - p[2]) * z / p[0]));
      write(out.data, offset + 4, static_cast<float>((v - p[6]) * z / p[5]));
      write(out.data, offset + 8, z);
    }
  return out;
}
}  // namespace nuvo_camera
