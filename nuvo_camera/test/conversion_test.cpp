#include <gtest/gtest.h>

#include <bit>
#include <cmath>
#include <nuvo_camera/conversion.hpp>
namespace {
nuvo::client::Frame frame() {
  auto bytes = std::make_shared<std::vector<std::byte>>(15);
  const std::array<float, 3> values{2.F, 4.F, 7.F};
  for (unsigned j = 0; j < 3; ++j) {
    const auto bits = std::bit_cast<std::uint32_t>(values[j]);
    for (unsigned i = 0; i < 4; ++i)
      (*bytes)[j * 4 + i] = std::byte((bits >> (i * 8U)) & 255U);
  }
  (*bytes)[12] = std::byte{1};
  (*bytes)[13] = std::byte{6};
  (*bytes)[14] = std::byte{1};
  nuvo::client::Frame f;
  f.owner = bytes;
  f.products = {{"depth", "float32_le", {3, 1}, std::span(*bytes).first(12)},
                {"validity", "uint8", {3, 1}, std::span(*bytes).subspan(12)}};
  nuvo::client::Geometry g;
  g.grid = {3, 1};
  g.left_projection = {100, 0, 1, 0, 0, 200, 2, 0, 0, 0, 1, 0};
  f.matching_geometry = g;
  return f;
}
float at(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
  return nuvo::client::float32_le(
      reinterpret_cast<const std::byte*>(bytes.data() + offset));
}
}  // namespace
TEST(Conversion, PreservesAssociationUnitsAndRejectionCodes) {
  auto f = frame();
  std_msgs::msg::Header h;
  h.frame_id = "front_rectified_optical";
  h.stamp.sec = 123;
  const auto images = nuvo_camera::convert(f, h);
  EXPECT_EQ(images.depth.encoding, "32FC1");
  EXPECT_EQ(images.validity.encoding, "8UC1");
  EXPECT_EQ(images.depth.step, 12U);
  EXPECT_EQ(images.camera_info.header, h);
  EXPECT_EQ(images.validity.header, h);
  EXPECT_EQ(images.camera_info.k[0], 100);
  EXPECT_EQ(images.camera_info.k[4], 200);
  EXPECT_EQ(images.camera_info.k[2], 1);
  EXPECT_FLOAT_EQ(at(images.depth.data, 0), 2);
  EXPECT_TRUE(std::isnan(at(images.depth.data, 4)));
  EXPECT_EQ(images.validity.data[1], 6);
}
TEST(Conversion, ProjectsInRectifiedOpticalFrameAndKeepsInvalids) {
  const auto images = nuvo_camera::convert(frame(), std_msgs::msg::Header{});
  const auto points = nuvo_camera::cloud(images, 1);
  EXPECT_EQ(points.width, 3U);
  EXPECT_EQ(points.point_step, 12U);
  EXPECT_FALSE(points.is_dense);
  EXPECT_FLOAT_EQ(at(points.data, 0), -.02F);
  EXPECT_FLOAT_EQ(at(points.data, 4), -.02F);
  EXPECT_FLOAT_EQ(at(points.data, 8), 2);
  EXPECT_TRUE(std::isnan(at(points.data, 12)));
  EXPECT_FLOAT_EQ(at(points.data, 24), .07F);
  const auto decimated = nuvo_camera::cloud(images, 2);
  EXPECT_EQ(decimated.width, 2U);
  EXPECT_FLOAT_EQ(at(decimated.data, 12), .07F);
}
TEST(Conversion, RejectsMismatchedGeometry) {
  auto f = frame();
  f.matching_geometry->grid.width = 4;
  EXPECT_THROW(nuvo_camera::convert(f, std_msgs::msg::Header{}),
               std::invalid_argument);
  f.matching_geometry.reset();
  EXPECT_THROW(nuvo_camera::convert(f, std_msgs::msg::Header{}),
               std::invalid_argument);
}
