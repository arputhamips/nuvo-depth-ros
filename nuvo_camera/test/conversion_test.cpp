#include <gtest/gtest.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <nuvo_camera/conversion.hpp>
namespace {
nuvo::client::Frame frame(
    std::array<float, 3> values = {2.F, 4.F, 7.F}) {
  auto bytes = std::make_shared<std::vector<std::byte>>(15);
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
TEST(Conversion, NormalizesInvalidDepthWithoutChangingValidityCodes) {
  const auto images = nuvo_camera::convert(
      frame({-1.F, std::numeric_limits<float>::infinity(),
             std::numeric_limits<float>::quiet_NaN()}),
      std_msgs::msg::Header{});
  for (unsigned i = 0; i < 3; ++i)
    EXPECT_TRUE(std::isnan(at(images.depth.data, i * 4)));
  EXPECT_EQ(images.validity.data, (std::vector<std::uint8_t>{1, 6, 1}));
  const auto points = nuvo_camera::cloud(images, 1);
  for (unsigned i = 0; i < 9; ++i)
    EXPECT_TRUE(std::isnan(at(points.data, i * 4)));
}
TEST(Conversion, RejectsNonfiniteAndUnsupportedProjection) {
  for (const auto value : {0., -1., std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
    auto f = frame();
    f.matching_geometry->left_projection[0] = value;
    EXPECT_THROW(nuvo_camera::convert(f, std_msgs::msg::Header{}),
                 std::invalid_argument);
    auto images = nuvo_camera::convert(frame(), std_msgs::msg::Header{});
    images.camera_info.p[5] = value;
    EXPECT_THROW(nuvo_camera::cloud(images), std::invalid_argument);
  }
  auto f = frame();
  f.matching_geometry->left_projection[2] =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(nuvo_camera::convert(f, std_msgs::msg::Header{}),
               std::invalid_argument);
  auto images = nuvo_camera::convert(frame(), std_msgs::msg::Header{});
  images.camera_info.p[3] = .1;
  EXPECT_THROW(nuvo_camera::cloud(images), std::invalid_argument);
}
TEST(Conversion, ReadsPaddedDepthRowsAndDecimatesBothDimensions) {
  auto images = nuvo_camera::convert(frame(), std_msgs::msg::Header{});
  const auto row = images.depth.data;
  images.depth.height = images.camera_info.height = 3;
  images.depth.step = 16;
  images.depth.data.assign(48, 0);
  for (unsigned v = 0; v < 3; ++v)
    std::copy(row.begin(), row.end(), images.depth.data.begin() + v * 16);
  const auto points = nuvo_camera::cloud(images, 2);
  EXPECT_EQ(points.width, 2U);
  EXPECT_EQ(points.height, 2U);
  EXPECT_EQ(points.row_step, 24U);
  EXPECT_FLOAT_EQ(at(points.data, 24), -.02F);
  EXPECT_FLOAT_EQ(at(points.data, 28), 0.F);
  EXPECT_FLOAT_EQ(at(points.data, 32), 2.F);
  EXPECT_FLOAT_EQ(at(points.data, 36), .07F);
}
TEST(Conversion, RejectsMalformedCloudImageBeforeReadingOrAllocating) {
  const auto good = nuvo_camera::convert(frame(), std_msgs::msg::Header{});
  EXPECT_THROW(nuvo_camera::cloud(good, 0), std::invalid_argument);
  EXPECT_THROW(nuvo_camera::cloud(good, 17), std::invalid_argument);
  auto images = good;
  images.depth.data.pop_back();
  EXPECT_THROW(nuvo_camera::cloud(images), std::invalid_argument);
  images = good;
  images.depth.width = 0;
  EXPECT_THROW(nuvo_camera::cloud(images), std::invalid_argument);
  images = good;
  images.depth.step = 4;
  EXPECT_THROW(nuvo_camera::cloud(images), std::invalid_argument);
  images = good;
  images.depth.is_bigendian = true;
  EXPECT_THROW(nuvo_camera::cloud(images), std::invalid_argument);
  images = good;
  images.camera_info.width = 4;
  EXPECT_THROW(nuvo_camera::cloud(images), std::invalid_argument);
}
TEST(Conversion, OptionalImagesUseTheirOwnGridAndPreserveMonoAndColor) {
  auto f = frame();
  auto image_bytes = std::make_shared<std::vector<std::byte>>(16);
  for (unsigned i = 0; i < image_bytes->size(); ++i)
    (*image_bytes)[i] = std::byte(i);
  auto g = *f.matching_geometry;
  g.grid = {2, 2};
  g.left_projection = {200, 0, .5, 0, 0, 400, .75, 0, 0, 0, 1, 0};
  g.right_projection = {200, 0, 1.5, -20, 0, 400, .75, 0, 0, 0, 1, 0};
  f.image_geometry = g;
  f.products.push_back({"left_rectified", "gray8", {2, 2},
                        std::span(*image_bytes).first(4)});
  f.products.push_back({"right_rectified_rgb", "rgb8", {2, 2},
                        std::span(*image_bytes).subspan(4)});
  std_msgs::msg::Header h;
  h.stamp.sec = 123;
  h.frame_id = "image_grid_optical";
  const auto left = nuvo_camera::rectified_image(f, h);
  const auto right = nuvo_camera::rectified_image(f, h, true);
  EXPECT_EQ(left.image.header, h);
  EXPECT_EQ(right.camera_info.header, h);
  EXPECT_EQ(left.image.encoding, "mono8");
  EXPECT_EQ(left.image.step, 2U);
  EXPECT_EQ(left.image.data, (std::vector<std::uint8_t>{0, 1, 2, 3}));
  EXPECT_EQ(right.image.encoding, "rgb8");
  EXPECT_EQ(right.image.step, 6U);
  EXPECT_EQ(right.image.data.front(), 4U);
  EXPECT_EQ(right.image.data.back(), 15U);
  EXPECT_EQ(right.camera_info.p, g.right_projection);
  EXPECT_EQ(left.camera_info.width, 2U);
  EXPECT_EQ(left.camera_info.height, 2U);
  EXPECT_EQ(left.camera_info.k[0], 200);
  EXPECT_EQ(left.camera_info.k[2], .5);
  EXPECT_EQ(right.camera_info.k[2], 1.5);
  EXPECT_EQ(left.camera_info.d, (std::vector<double>{0, 0, 0, 0, 0}));
  (*image_bytes)[0] = std::byte{99};
  EXPECT_EQ(left.image.data[0], 0U);  // ROS ownership outlives borrowed input.
  f.products.back().kind = "right_rectified";
  f.products.back().format = "bgr8";
  EXPECT_EQ(nuvo_camera::rectified_image(f, h, true).image.encoding, "bgr8");
  f.image_geometry->grid.height = 3;
  EXPECT_THROW(nuvo_camera::rectified_image(f, h), std::invalid_argument);
  f.image_geometry.reset();
  EXPECT_THROW(nuvo_camera::rectified_image(f, h), std::invalid_argument);
}
TEST(Conversion, DisparityUsesProducerMatchingConstraintsAndPrincipalOffset) {
  auto f = frame({10.F, 6.F, std::numeric_limits<float>::quiet_NaN()});
  f.products.front().kind = "disparity";
  f.matching_geometry->right_projection =
      {100, 0, 3, -10, 0, 200, 2, 0, 0, 0, 1, 0};
  f.matching_geometry->baseline_m = .1;
  f.disparity_constraints = nuvo::client::DisparityConstraints{
      0., 64., .0625, {1, 0, 2, 1}};
  std_msgs::msg::Header h;
  h.stamp.sec = 123;
  h.frame_id = "matching_optical";
  const auto out = nuvo_camera::disparity(f, h);
  EXPECT_EQ(out.header, h);
  EXPECT_EQ(out.image.header, h);
  EXPECT_EQ(out.image.width, 3U);
  EXPECT_EQ(out.image.height, 1U);
  EXPECT_EQ(out.image.step, 12U);
  EXPECT_EQ(out.image.encoding, "32FC1");
  EXPECT_FLOAT_EQ(at(out.image.data, 0), 12.F);
  // ROS disparity has no rejection plane, so rejected native samples stay invalid.
  EXPECT_TRUE(std::isnan(at(out.image.data, 4)));
  EXPECT_TRUE(std::isnan(at(out.image.data, 8)));
  EXPECT_FLOAT_EQ(out.f, 100.F);
  EXPECT_FLOAT_EQ(out.t, .1F);
  EXPECT_FLOAT_EQ(out.min_disparity, 2.F);
  EXPECT_FLOAT_EQ(out.max_disparity, 66.F);
  EXPECT_FLOAT_EQ(out.delta_d, .0625F);
  EXPECT_EQ(out.valid_window.x_offset, 1U);
  EXPECT_EQ(out.valid_window.width, 2U);
  EXPECT_EQ(out.valid_window.height, 1U);
  EXPECT_FALSE(out.valid_window.do_rectify);
  f.disparity_constraints.reset();
  EXPECT_THROW(nuvo_camera::disparity(f, h), std::invalid_argument);
  f.disparity_constraints = nuvo::client::DisparityConstraints{
      0., 64., .0625, {1, 0, 2, 1}};
  f.products.pop_back();
  EXPECT_THROW(nuvo_camera::disparity(f, h), std::invalid_argument);
}
TEST(Conversion, FullSizeDepthAndCloudPreserveLastPixelAndInvalidRows) {
  constexpr unsigned w = 728, h = 544;
  constexpr std::size_t count = w * h;
  auto bytes = std::make_shared<std::vector<std::byte>>(count * 5);
  const auto bits = std::bit_cast<std::uint32_t>(2.F);
  for (std::size_t i = 0; i < count; ++i) {
    for (unsigned j = 0; j < 4; ++j)
      (*bytes)[i * 4 + j] = std::byte((bits >> (8U * j)) & 255U);
    (*bytes)[count * 4 + i] = std::byte(i / w == h / 2 ? 6 : 1);
  }
  auto f = frame();
  f.owner = bytes;
  f.products = {{"depth", "float32_le", {w, h}, std::span(*bytes).first(count * 4)},
                {"validity", "uint8", {w, h}, std::span(*bytes).subspan(count * 4)}};
  f.matching_geometry->grid = {w, h};
  const auto images = nuvo_camera::convert(f, std_msgs::msg::Header{});
  const auto points = nuvo_camera::cloud(images, 1);
  EXPECT_EQ(images.depth.data.size(), count * 4);
  EXPECT_EQ(points.data.size(), count * 12);
  EXPECT_EQ(points.width, w);
  EXPECT_EQ(points.height, h);
  EXPECT_TRUE(std::isnan(at(points.data, (w * (h / 2)) * 12)));
  EXPECT_FLOAT_EQ(at(points.data, (count - 1) * 12), (w - 2) * .02F);
  EXPECT_FLOAT_EQ(at(points.data, (count - 1) * 12 + 4), (h - 3) * .01F);
  EXPECT_FLOAT_EQ(at(points.data, (count - 1) * 12 + 8), 2.F);
}
TEST(Conversion, RejectsUnrepresentableRosDisparityMetadata) {
  auto base = frame();
  base.products.front().kind = "disparity";
  base.matching_geometry->baseline_m = .1;
  base.matching_geometry->right_projection[2] =
      base.matching_geometry->left_projection[2];
  base.disparity_constraints = nuvo::client::DisparityConstraints{
      0., 64., .0625, {0, 0, 3, 1}};
  for (const auto value : {std::numeric_limits<double>::max(),
                           std::numeric_limits<double>::denorm_min()}) {
    auto f = base;
    f.matching_geometry->left_projection[0] = value;
    EXPECT_THROW(nuvo_camera::disparity(f, std_msgs::msg::Header{}), std::invalid_argument);
    f = base;
    f.matching_geometry->baseline_m = value;
    EXPECT_THROW(nuvo_camera::disparity(f, std_msgs::msg::Header{}), std::invalid_argument);
    f = base;
    f.disparity_constraints->quantization_px = value;
    EXPECT_THROW(nuvo_camera::disparity(f, std_msgs::msg::Header{}), std::invalid_argument);
  }
  auto f = base;
  f.disparity_constraints->accepted_max_px = std::numeric_limits<double>::max();
  EXPECT_THROW(nuvo_camera::disparity(f, std_msgs::msg::Header{}), std::invalid_argument);
  f = base;
  f.disparity_constraints->accepted_min_px = 16777216.;
  f.disparity_constraints->accepted_max_px = 16777217.;
  EXPECT_THROW(nuvo_camera::disparity(f, std_msgs::msg::Header{}), std::invalid_argument);
}
