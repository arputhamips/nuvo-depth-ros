#include "nuvo_camera/conversion.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
namespace nuvo_camera {
namespace {
void write(std::vector<std::uint8_t>& bytes, std::size_t offset, float value) {
  if constexpr (std::endian::native == std::endian::little) {
    std::memcpy(bytes.data() + offset, &value, sizeof(value));
    return;
  }
  const auto bits = std::bit_cast<std::uint32_t>(value);
  for (unsigned i = 0; i < 4; ++i)
    bytes[offset + i] = static_cast<std::uint8_t>(bits >> (8U * i));
}
float read(const std::byte* bytes) {
  if constexpr (std::endian::native == std::endian::little) {
    float value;
    std::memcpy(&value, bytes, sizeof(value));
    return value;
  }
  return nuvo::client::float32_le(bytes);
}
void validate_projection(const std::array<double, 12>& p) {
  if (!std::all_of(p.begin(), p.end(),
                   [](double value) { return std::isfinite(value); }) ||
      p[0] <= 0 || p[5] <= 0)
    throw std::invalid_argument("invalid rectified focal lengths or projection");
  for (const auto index : {1, 3, 4, 7, 8, 9, 11})
    if (std::abs(p[index]) > 1e-7)
      throw std::invalid_argument("XYZ requires a canonical left rectified projection");
  if (std::abs(p[10] - 1) > 1e-7)
    throw std::invalid_argument("invalid rectified projection scale");
}
sensor_msgs::msg::CameraInfo camera_info(
    const nuvo::client::Geometry& geometry, const std_msgs::msg::Header& header,
    bool right = false) {
  sensor_msgs::msg::CameraInfo out;
  out.header = header;
  out.width = geometry.grid.width;
  out.height = geometry.grid.height;
  const auto& p = right ? geometry.right_projection : geometry.left_projection;
  // A virtual already rectified pinhole camera, never fabricated physical
  // sensor intrinsics. The right P retains its calibrated stereo translation.
  out.p = p;
  out.k = {p[0], 0, p[2], 0, p[5], p[6], 0, 0, 1};
  out.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  out.distortion_model = "plumb_bob";
  out.d = {0, 0, 0, 0, 0};
  return out;
}
std::size_t product_size(const nuvo::client::Product& product,
                         const nuvo::client::Geometry& geometry,
                         unsigned channels) {
  const auto w = product.grid.width, h = product.grid.height;
  const auto bytes = static_cast<std::uint64_t>(w) * h * channels;
  if (!w || !h || w != geometry.grid.width || h != geometry.grid.height ||
      w > std::numeric_limits<std::uint32_t>::max() / channels ||
      bytes > std::numeric_limits<std::size_t>::max() ||
      product.bytes.size() != bytes)
    throw std::invalid_argument("product disagrees with its calibrated grid");
  return static_cast<std::size_t>(bytes);
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
      w > std::numeric_limits<std::uint32_t>::max() / 4 ||
      pixels > std::numeric_limits<std::size_t>::max() / 4 ||
      depth->bytes.size() != pixels * 4 || validity->bytes.size() != pixels)
    throw std::invalid_argument(
        "measurement products disagree with calibrated grid");
  validate_projection(g.left_projection);
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
    const auto value = read(depth->bytes.data() + 4 * i);
    output.validity.data[i] = code;
    write(output.depth.data, 4 * i,
          code == 1 && std::isfinite(value) && value > 0
              ? value
              : std::numeric_limits<float>::quiet_NaN());
  }
  output.camera_info = camera_info(g, header);
  return output;
}
RectifiedImage rectified_image(const nuvo::client::Frame& frame,
                              const std_msgs::msg::Header& header, bool right) {
  const auto* product = frame.find(right ? "right_rectified" : "left_rectified");
  if (!product)
    product = frame.find(right ? "right_rectified_rgb" : "left_rectified_rgb");
  if (!product || !frame.image_geometry)
    throw std::invalid_argument("rectified image and image-grid geometry are required");
  const auto& g = *frame.image_geometry;
  const auto channels = product->format == "gray8" ? 1U : 3U;
  if (product->format != "gray8" && product->format != "rgb8" &&
      product->format != "bgr8")
    throw std::invalid_argument("unsupported rectified image format");
  const auto size = product_size(*product, g, channels);
  RectifiedImage out;
  out.camera_info = camera_info(g, header, right);
  out.image.header = header;
  out.image.width = g.grid.width;
  out.image.height = g.grid.height;
  out.image.encoding = channels == 1 ? "mono8" : product->format;
  out.image.is_bigendian = false;
  out.image.step = g.grid.width * channels;
  out.image.data.resize(size);
  std::memcpy(out.image.data.data(), product->bytes.data(), size);
  return out;
}
stereo_msgs::msg::DisparityImage disparity(
    const nuvo::client::Frame& frame, const std_msgs::msg::Header& header) {
  const auto* product = frame.find("disparity");
  const auto* validity = frame.find("validity");
  if (!product || !validity || validity->format != "uint8" ||
      product->format != "float32_le" || !frame.matching_geometry ||
      !frame.disparity_constraints)
    throw std::invalid_argument("disparity, validity, matching geometry and producer constraints are required");
  const auto& g = *frame.matching_geometry;
  const auto& limits = *frame.disparity_constraints;
  const auto size = product_size(*product, g, 4);
  product_size(*validity, g, 1);
  const auto offset = g.left_projection[2] - g.right_projection[2];
  stereo_msgs::msg::DisparityImage out;
  out.f = static_cast<float>(g.left_projection[0]);
  out.t = static_cast<float>(g.baseline_m);
  out.min_disparity = static_cast<float>(limits.accepted_min_px - offset);
  out.max_disparity = static_cast<float>(limits.accepted_max_px - offset);
  out.delta_d = static_cast<float>(limits.quantization_px);
  // The SDK transports double metadata; these ROS fields are float32. Do not
  // publish infinities, underflowed focal/baseline/quantization, or an accepted
  // interval collapsed by narrowing to float32.
  if (!std::isfinite(out.f) || out.f <= 0 || !std::isfinite(out.t) || out.t <= 0 ||
      !std::isfinite(out.delta_d) || out.delta_d <= 0 ||
      !std::isfinite(out.min_disparity) || !std::isfinite(out.max_disparity) ||
      out.min_disparity >= out.max_disparity)
    throw std::invalid_argument("disparity metadata cannot be represented by ROS float32 fields");
  out.header = out.image.header = header;
  out.image.width = g.grid.width;
  out.image.height = g.grid.height;
  out.image.encoding = "32FC1";
  out.image.is_bigendian = false;
  out.image.step = g.grid.width * 4;
  out.image.data.resize(size);
  // Each input span is borrowed from its immutable SDK frame; this is the sole
  // owned pixel buffer. ROS disparity has no rejection-code plane: carry every
  // rejected native sample as NaN so downstream reconstruction cannot revive it.
  for (std::size_t i = 0; i < size / 4; ++i) {
    const auto value = read(product->bytes.data() + i * 4);
    const auto corrected = static_cast<float>(value - offset);
    write(out.image.data, i * 4, validity->bytes[i] == std::byte{1} &&
              std::isfinite(corrected) && corrected > 0
              ? corrected : std::numeric_limits<float>::quiet_NaN());
  }
  out.valid_window.x_offset = limits.valid_roi.x;
  out.valid_window.y_offset = limits.valid_roi.y;
  out.valid_window.width = limits.valid_roi.width;
  out.valid_window.height = limits.valid_roi.height;
  out.valid_window.do_rectify = false;
  return out;
}
sensor_msgs::msg::PointCloud2 cloud(const Images& images, unsigned stride) {
  if (stride < 1 || stride > 16)
    throw std::invalid_argument("point cloud stride must be 1..16");
  const auto& depth = images.depth;
  const auto& p = images.camera_info.p;
  validate_projection(p);
  if (!depth.width || !depth.height || depth.encoding != "32FC1" ||
      depth.is_bigendian ||
      depth.width > std::numeric_limits<std::uint32_t>::max() / 4 ||
      depth.step < depth.width * 4 ||
      static_cast<std::uint64_t>(depth.step) * depth.height != depth.data.size() ||
      images.camera_info.width != depth.width ||
      images.camera_info.height != depth.height)
    throw std::invalid_argument("XYZ requires a calibrated little-endian depth image");
  sensor_msgs::msg::PointCloud2 out;
  out.header = depth.header;
  out.width = (depth.width - 1) / stride + 1;
  out.height = (depth.height - 1) / stride + 1;
  if (out.width > std::numeric_limits<std::uint32_t>::max() / 12)
    throw std::invalid_argument("XYZ row exceeds ROS point cloud limits");
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
  const auto bytes = static_cast<std::uint64_t>(out.row_step) * out.height;
  if (bytes > std::numeric_limits<std::size_t>::max())
    throw std::invalid_argument("XYZ buffer exceeds addressable size");
  out.data.resize(static_cast<std::size_t>(bytes));
  // The column ray is shared across rows. Compute divisions once per column
  // and row instead of twice for every output point.
  std::vector<double> columns(out.width);
  for (unsigned column = 0; column < out.width; ++column)
    columns[column] = (column * stride - p[2]) / p[0];
  const auto invalid = std::numeric_limits<float>::quiet_NaN();
  for (unsigned row = 0; row < out.height; ++row) {
    const auto v = row * stride;
    const auto y_ray = (v - p[6]) / p[5];
    for (unsigned column = 0; column < out.width; ++column) {
      const auto z = read(reinterpret_cast<const std::byte*>(depth.data.data()) +
                          static_cast<std::size_t>(v) * depth.step +
                          column * stride * 4);
      const auto offset =
          (static_cast<std::size_t>(row) * out.width + column) * 12;
      const auto x = static_cast<float>(columns[column] * z);
      const auto y = static_cast<float>(y_ray * z);
      const bool valid = z > 0 && std::isfinite(z) && std::isfinite(x) &&
                         std::isfinite(y);
      write(out.data, offset, valid ? x : invalid);
      write(out.data, offset + 4, valid ? y : invalid);
      write(out.data, offset + 8, valid ? z : invalid);
    }
  }
  return out;
}
}  // namespace nuvo_camera
