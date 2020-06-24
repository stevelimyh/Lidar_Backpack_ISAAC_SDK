/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "gtest/gtest.h"

#include "capnp/serialize.h"
#include "engine/core/tensor/tensor.hpp"
#include "messages/element_type.hpp"
#include "messages/math.hpp"
#include "messages/tensor.hpp"
#include "packages/composite/gems/measure.hpp"
#include "packages/composite/gems/parser.hpp"
#include "packages/composite/gems/serializer.hpp"

namespace isaac {

TEST(CompositeWrite, Measure) {
  // string to measure
  const auto maybe_measure = composite::FromString("rotation");
  ASSERT_TRUE(maybe_measure);
  EXPECT_EQ(*maybe_measure, composite::Measure::kRotation);
  // measure to string
  EXPECT_EQ(composite::ToString(composite::Measure::kNone), "none");
}

TEST(CompositeWrite, Basics) {
  // Source schema
  const composite::Schema source_schema({
    {"foo", composite::Measure::kSpeed, VectorXi::Constant(1, 2)},
    {"bar", composite::Measure::kSpeed, VectorXi::Constant(1, 3)},
    {"foo", composite::Measure::kPosition, VectorXi::Constant(1, 2)}
  });

  // Actual values used
  constexpr double kSpeedX = 1234.0;
  constexpr double kSpeedY = 5678.0;
  constexpr double kPositionX = 444.0;
  constexpr double kPositionY = 555.0;

  // Create a message and write data
  ::capnp::MallocMessageBuilder malloc_builder;
  std::vector<SharedBuffer> buffers;
  auto proto_builder = malloc_builder.getRoot<::CompositeProto>();
  WriteSchema(source_schema, proto_builder);
  {
    Tensor1d tensor(7);
    tensor(0) = kSpeedX;
    tensor(1) = kSpeedY;
    tensor(2) = 2.3;
    tensor(3) = -0.3;
    tensor(4) = -1.7;
    tensor(5) = kPositionX;
    tensor(6) = kPositionY;
    ToProto(std::move(tensor), proto_builder.initValues(), buffers);
  }

  // Generates serialized data
  ::capnp::ReaderOptions options;
  options.traversalLimitInWords = kj::maxValue;
  ::capnp::SegmentArrayMessageReader reader(malloc_builder.getSegmentsForOutput(), options);
  auto composite_reader = reader.getRoot<::CompositeProto>();

  // Parsing and checking
  composite::Parser parser;

  parser.requestSchema(composite::Schema({
      {std::string("foo"), composite::Measure::kSpeed, VectorXi::Constant(1, 2)}}));
  Vector2d v2d;
  const bool v2d_ok = parser.parse(composite_reader, buffers, v2d);
  EXPECT_TRUE(v2d_ok);
  EXPECT_EQ(v2d(0), kSpeedX);
  EXPECT_EQ(v2d(1), kSpeedY);

  Vector2d pos_v2d;
  parser.requestSchema(composite::Schema({
      {std::string("foo"), composite::Measure::kPosition, VectorXi::Constant(1, 2)}}));
  const bool pos_v2d_ok = parser.parse(composite_reader, buffers, pos_v2d);
  EXPECT_TRUE(pos_v2d_ok);
  EXPECT_EQ(pos_v2d(0), kPositionX);
  EXPECT_EQ(pos_v2d(1), kPositionY);

  Vector4d v4d;
  parser.requestSchema(composite::Schema({
      {std::string("foo"), composite::Measure::kSpeed, VectorXi::Constant(1, 2)},
      {std::string("foo"), composite::Measure::kPosition, VectorXi::Constant(1, 2)}}));
  const bool v4d_ok = parser.parse(composite_reader, buffers, v4d);
  EXPECT_TRUE(v4d_ok);
  EXPECT_EQ(v4d(0), kSpeedX);
  EXPECT_EQ(v4d(1), kSpeedY);
  EXPECT_EQ(v4d(2), kPositionX);
  EXPECT_EQ(v4d(3), kPositionY);
}

TEST(CompositeParser, Timeseries) {
  // Source schema
  const composite::Schema source_schema({
    composite::Quantity::Vector("foo", composite::Measure::kSpeed, 2),
    composite::Quantity::Scalar("time", composite::Measure::kTime),
    composite::Quantity::Vector("bar", composite::Measure::kSpeed, 2),
    composite::Quantity::Vector("foo", composite::Measure::kPosition, 2)
  });

  // Create a message and write data
  ::capnp::MallocMessageBuilder malloc_builder;
  std::vector<SharedBuffer> buffers;
  auto proto_builder = malloc_builder.getRoot<::CompositeProto>();
  WriteSchema(source_schema, proto_builder);
  {
    Tensor2d tensor(97, 7);
    for (int row = 0; row < tensor.dimensions()[0]; row++) {
      const double time = static_cast<double>(row) * 0.01;
      tensor(row, 0) = std::sin(time * 1.5 + 0.3);
      tensor(row, 1) = std::sin(time * 2.0 + 0.2);
      tensor(row, 2) = time;
      tensor(row, 3) = std::sin(time * 3.0 - 0.1);
      tensor(row, 4) = std::sin(time * 3.5 - 0.2);
      tensor(row, 5) = std::sin(time * 4.0 - 0.3);
      tensor(row, 6) = std::sin(time * 4.5 - 0.4);
    }
    ToProto(std::move(tensor), proto_builder.initValues(), buffers);
  }

  // Generates serialized data
  ::capnp::ReaderOptions options;
  options.traversalLimitInWords = kj::maxValue;
  ::capnp::SegmentArrayMessageReader reader(malloc_builder.getSegmentsForOutput(), options);
  auto composite_reader = reader.getRoot<::CompositeProto>();

  // Parsing and checking
  composite::Parser parser;

  parser.requestSchema(composite::Schema({
    composite::Quantity::Vector("foo", composite::Measure::kSpeed, 2),
    composite::Quantity::Vector("foo", composite::Measure::kPosition, 2)
  }));

  Timeseries<VectorXd, double> series;
  parser.parse(composite_reader, buffers, "time", series);

  EXPECT_EQ(series.size(), 97);

  for (size_t row = 0; row < series.size(); row++) {
    const auto& state = series.state(row);
    const double time = static_cast<double>(row) * 0.01;
    EXPECT_EQ(series.at(row).stamp, time);
    EXPECT_EQ(state.size(), 4);
    EXPECT_EQ(state[0], std::sin(time * 1.5 + 0.3));
    EXPECT_EQ(state[1], std::sin(time * 2.0 + 0.2));
    EXPECT_EQ(state[2], std::sin(time * 4.0 - 0.3));
    EXPECT_EQ(state[3], std::sin(time * 4.5 - 0.4));
  }
}


TEST(CompositeParser, Serializer_Basic) {
  composite::Serializer serializer;
  const composite::Schema source_schema(
      {composite::Quantity::Vector("foo", composite::Measure::kSpeed, 2),
       composite::Quantity::Scalar("time", composite::Measure::kTime)});

  serializer.setSchema(source_schema);

  // Create a message and write data
  ::capnp::MallocMessageBuilder malloc_builder;
  std::vector<SharedBuffer> buffers;
  auto proto_builder = malloc_builder.getRoot<::CompositeProto>();

  VectorXd data(3);
  constexpr double kTimestamp = 3456.7;
  constexpr double kSpeedX = 123.4;
  constexpr double kSpeedY = 567.8;

  data(0) = kSpeedX;
  data(1) = kSpeedY;
  data(2) = kTimestamp;

  EXPECT_TRUE(serializer.serialize(data, proto_builder, buffers));

  // Generates serialized data
  ::capnp::ReaderOptions options;
  options.traversalLimitInWords = kj::maxValue;
  ::capnp::SegmentArrayMessageReader reader(malloc_builder.getSegmentsForOutput(), options);
  auto composite_reader = reader.getRoot<::CompositeProto>();

  // Parsing and checking
  composite::Parser parser;

  parser.requestSchema(source_schema);
  Vector3d v3d;
  EXPECT_TRUE(parser.parse(composite_reader, buffers, v3d));
  EXPECT_EQ(v3d(0), kSpeedX);
  EXPECT_EQ(v3d(1), kSpeedY);
  EXPECT_EQ(v3d(2), kTimestamp);
}

TEST(CompositeParser, Serializer_Timeseries) {
  composite::Serializer serializer;
  const composite::Schema source_schema(
      {composite::Quantity::Vector("foo", composite::Measure::kSpeed, 2),
       composite::Quantity::Scalar("time", composite::Measure::kTime)});

  serializer.setSchema(source_schema);

  // Create a message and write data
  ::capnp::MallocMessageBuilder malloc_builder;
  std::vector<SharedBuffer> buffers;
  auto proto_builder = malloc_builder.getRoot<::CompositeProto>();

  Timeseries<Vector3d, double> series;
  constexpr double kTimestamp1 = 7654.3;
  constexpr double kTimestamp2 = 8765.4;
  constexpr double kSpeed1X = 12.3;
  constexpr double kSpeed1Y = 23.4;
  constexpr double kSpeed2X = 34.5;
  constexpr double kSpeed2Y = 45.6;

  series.push(kTimestamp1, Vector3d{kSpeed1X, kSpeed1Y, 0.0});
  series.push(kTimestamp2, Vector3d{kSpeed2X, kSpeed2Y, 0.0});

  const composite::Quantity timestamp_quantity =
      composite::Quantity::Scalar("time", composite::Measure::kTime);

  EXPECT_TRUE(serializer.serialize(series, timestamp_quantity, proto_builder, buffers));

  // Generates serialized data
  ::capnp::ReaderOptions options;
  options.traversalLimitInWords = kj::maxValue;
  ::capnp::SegmentArrayMessageReader reader(malloc_builder.getSegmentsForOutput(), options);
  auto composite_reader = reader.getRoot<::CompositeProto>();

  // Parsing and checking
  composite::Parser parser;

  parser.requestSchema(source_schema);

  Timeseries<Vector3d, double> parsed_series;
  parser.parse(composite_reader, buffers, "time", parsed_series);

  EXPECT_EQ(parsed_series.size(), 2);

  EXPECT_EQ(parsed_series.at(0).state(0), kSpeed1X);
  EXPECT_EQ(parsed_series.at(0).state(1), kSpeed1Y);
  EXPECT_EQ(parsed_series.at(0).state(2), kTimestamp1);

  EXPECT_EQ(parsed_series.at(1).state(0), kSpeed2X);
  EXPECT_EQ(parsed_series.at(1).state(1), kSpeed2Y);
  EXPECT_EQ(parsed_series.at(1).state(2), kTimestamp2);
}

}  // namespace isaac
