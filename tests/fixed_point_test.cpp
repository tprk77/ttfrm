// Copyright (c) 2019 Tim Perkins

#include <gtest/gtest.h>

#include <ttfrm/fixed_point.hpp>

#include <iostream>
#include <ttfrm/fixed_point_iostream.hpp>

using MyFixedPoint = ttfrm::FixedPoint<uint64_t, 16>;

TEST(FixedPoint, ConstructFromRaw)
{
  const auto fxpt = MyFixedPoint::FromRaw(UINT64_C(3141592653));
  std::cout << fxpt << '\n';
  EXPECT_EQ(fxpt.raw_value, UINT64_C(3141592653));
}

TEST(FixedPoint, ConstructFromInteger)
{
  const auto fxpt = MyFixedPoint::FromInteger(UINT64_C(3141592653));
  std::cout << fxpt << '\n';
  EXPECT_EQ(fxpt.raw_value, (UINT64_C(3141592653) << 16));
}

TEST(FixedPoint, ConstructFromFloat)
{
  const auto fxpt = MyFixedPoint::FromFloatingPoint(3.141592653);
  std::cout << fxpt << '\n';
  std::cout << fmt::format("{:.16}", fxpt.AsFloatingPoint()) << '\n';
  EXPECT_EQ(fxpt.raw_value, static_cast<uint64_t>((3 << 16) + ((1 << 16) * 0.141592653)));
}

TEST(FixedPoint, Assignment)
{
  const MyFixedPoint fxpt_a = MyFixedPoint::FromRaw(UINT64_C(3141592653));
  MyFixedPoint fxpt_b = MyFixedPoint::FromRaw(UINT64_C(6283185306));
  fxpt_b = fxpt_a;
  EXPECT_EQ(fxpt_b.raw_value, UINT64_C(3141592653));
}

TEST(FixedPoint, AssignmentMove)
{
  MyFixedPoint fxpt_a = MyFixedPoint::FromRaw(UINT64_C(3141592653));
  MyFixedPoint fxpt_b = MyFixedPoint::FromRaw(UINT64_C(6283185306));
  fxpt_b = std::move(fxpt_a);
  EXPECT_EQ(fxpt_b.raw_value, UINT64_C(3141592653));
}

TEST(FixedPoint, Equality)
{
  const MyFixedPoint fxpt_a = MyFixedPoint::FromRaw(UINT64_C(3141592653));
  const MyFixedPoint fxpt_b = MyFixedPoint::FromRaw(UINT64_C(3141592653));
  const MyFixedPoint fxpt_c = MyFixedPoint::FromRaw(UINT64_C(6283185306));
  EXPECT_TRUE(fxpt_a == fxpt_b);
  EXPECT_FALSE(fxpt_a == fxpt_c);
}

TEST(FixedPoint, Inequality)
{
  const MyFixedPoint fxpt_a = MyFixedPoint::FromRaw(UINT64_C(3141592653));
  const MyFixedPoint fxpt_b = MyFixedPoint::FromRaw(UINT64_C(6283185306));
  const MyFixedPoint fxpt_c = MyFixedPoint::FromRaw(UINT64_C(3141592653));
  EXPECT_TRUE(fxpt_a != fxpt_b);
  EXPECT_FALSE(fxpt_a != fxpt_c);
}

TEST(FixedPoint, LessThan)
{
  const MyFixedPoint fxpt_a = MyFixedPoint::FromRaw(UINT64_C(3141592653));
  const MyFixedPoint fxpt_b = MyFixedPoint::FromRaw(UINT64_C(6283185306));
  const MyFixedPoint fxpt_c = MyFixedPoint::FromRaw(UINT64_C(3141592653));
  EXPECT_TRUE(fxpt_a < fxpt_b);
  EXPECT_FALSE(fxpt_a < fxpt_c);
}

TEST(FixedPoint, GreaterThan)
{
  const MyFixedPoint fxpt_a = MyFixedPoint::FromRaw(UINT64_C(6283185306));
  const MyFixedPoint fxpt_b = MyFixedPoint::FromRaw(UINT64_C(3141592653));
  const MyFixedPoint fxpt_c = MyFixedPoint::FromRaw(UINT64_C(6283185306));
  EXPECT_TRUE(fxpt_a > fxpt_b);
  EXPECT_FALSE(fxpt_a > fxpt_c);
}

TEST(FixedPoint, LessThanOrEqual)
{
  const MyFixedPoint fxpt_a = MyFixedPoint::FromRaw(UINT64_C(3141592653));
  const MyFixedPoint fxpt_b = MyFixedPoint::FromRaw(UINT64_C(6283185306));
  const MyFixedPoint fxpt_c = MyFixedPoint::FromRaw(UINT64_C(3141592653));
  const MyFixedPoint fxpt_d = MyFixedPoint::FromRaw(UINT64_C(1618033988));
  EXPECT_TRUE(fxpt_a <= fxpt_b);
  EXPECT_TRUE(fxpt_a <= fxpt_c);
  EXPECT_FALSE(fxpt_a <= fxpt_d);
}

TEST(FixedPoint, GreaterThanOrEqual)
{
  const MyFixedPoint fxpt_a = MyFixedPoint::FromRaw(UINT64_C(3141592653));
  const MyFixedPoint fxpt_b = MyFixedPoint::FromRaw(UINT64_C(1618033988));
  const MyFixedPoint fxpt_c = MyFixedPoint::FromRaw(UINT64_C(3141592653));
  const MyFixedPoint fxpt_d = MyFixedPoint::FromRaw(UINT64_C(6283185306));
  EXPECT_TRUE(fxpt_a >= fxpt_b);
  EXPECT_TRUE(fxpt_a >= fxpt_c);
  EXPECT_FALSE(fxpt_a >= fxpt_d);
}

TEST(FixedPoint, AddAssign)
{
  MyFixedPoint fxpt_a = MyFixedPoint::FromParts(3, 9279);
  const MyFixedPoint fxpt_b = MyFixedPoint::FromParts(6, 18558);
  fxpt_a += fxpt_a;
  EXPECT_EQ(fxpt_a, fxpt_b);
  // __builtin_uadd_overflow
  // https://wiki.sei.cmu.edu/confluence/display/c/INT32-C.+Ensure+that+operations+on+signed+integers+do+not+result+in+overflow
}

TEST(FixedPoint, SubtractAssign)
{
  //
}

TEST(FixedPoint, MultiplyAssign)
{
  //
}

TEST(FixedPoint, DivideAssign)
{
  //
}

TEST(FixedPoint, Add)
{
  //
}

TEST(FixedPoint, Subtract)
{
  //
}

TEST(FixedPoint, Multiply)
{
  //
}

TEST(FixedPoint, Divide)
{
  //
}

TEST(FixedPoint, ExtractIntegerPart)
{
  //
}

TEST(FixedPoint, ExtractFractionalPart)
{
  //
}

TEST(FixedPoint, AsFloatingPoint)
{
  //
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
