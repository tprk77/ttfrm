// Copyright (c) 2019 Tim Perkins

#include <gtest/gtest.h>

#include <ttfrm/tfrm.hpp>

#include "test_utils.hpp"

using MyFramePair = ttfrm::FramePair<std::string>;
using MyTfrm = ttfrm::Tfrm<std::string>;

TEST(Tfrm, Construct)
{
  const ttfrm::Quat rot = ttfrm::Quat::Identity();
  const ttfrm::Vec3 trans = ttfrm::Vec3::Zero();
  const MyTfrm test_from_world("test", "world", rot, trans);
  EXPECT_EQ(test_from_world.ToFrame(), "test");
  EXPECT_EQ(test_from_world.FromFrame(), "world");
}

TEST(Tfrm, ConstructCopy)
{
  const ttfrm::Quat rot = ttfrm::Quat::Identity();
  const ttfrm::Vec3 trans = ttfrm::Vec3::Zero();
  const MyTfrm test_from_world("test", "world", rot, trans);
  const MyTfrm test_from_world2 = test_from_world;
  EXPECT_EQ(test_from_world2.ToFrame(), "test");
  EXPECT_EQ(test_from_world2.FromFrame(), "world");
}

TEST(Tfrm, ConstructMove)
{
  const ttfrm::Quat rot = ttfrm::Quat::Identity();
  const ttfrm::Vec3 trans = ttfrm::Vec3::Zero();
  MyTfrm test_from_world("test", "world", rot, trans);
  const MyTfrm test_from_world2 = std::move(test_from_world);
  EXPECT_EQ(test_from_world2.ToFrame(), "test");
  EXPECT_EQ(test_from_world2.FromFrame(), "world");
}

TEST(Tfrm, Identity)
{
  const MyTfrm test_from_world = MyTfrm::Identity("test", "world");
  EXPECT_EQ(test_from_world.ToFrame(), "test");
  EXPECT_EQ(test_from_world.FromFrame(), "world");
}

TEST(Tfrm, FromRotation)
{
  const ttfrm::Quat rot = ttfrm::Quat::Identity();
  const MyTfrm test_from_world = MyTfrm::FromRotation("test", "world", rot);
  EXPECT_EQ(test_from_world.ToFrame(), "test");
  EXPECT_EQ(test_from_world.FromFrame(), "world");
}

TEST(Tfrm, FromTranslation)
{
  const ttfrm::Vec3 trans = ttfrm::Vec3::Zero();
  const MyTfrm test_from_world = MyTfrm::FromTranslation("test", "world", trans);
  EXPECT_EQ(test_from_world.ToFrame(), "test");
  EXPECT_EQ(test_from_world.FromFrame(), "world");
}

TEST(Tfrm, FromIsometry)
{
  const ttfrm::Quat rot = ttfrm::Quat::Identity();
  const ttfrm::Vec3 trans = ttfrm::Vec3::Zero();
  Eigen::Isometry3d iso_test_from_world;
  iso_test_from_world.prerotate(rot);
  iso_test_from_world.pretranslate(trans);
  const MyTfrm test_from_world = MyTfrm::FromIsometry("test", "world", iso_test_from_world);
  EXPECT_EQ(test_from_world.ToFrame(), "test");
  EXPECT_EQ(test_from_world.FromFrame(), "world");
}

TEST(Tfrm, Assignment)
{
  const MyTfrm test_from_world = MyTfrm::Identity("test", "world");
  MyTfrm test_from_world2 = MyTfrm::Identity("junk1", "junk2");
  test_from_world2 = test_from_world;
  EXPECT_EQ(test_from_world2.ToFrame(), "test");
  EXPECT_EQ(test_from_world2.FromFrame(), "world");
}

TEST(Tfrm, AssignmentMove)
{
  MyTfrm test_from_world = MyTfrm::Identity("test", "world");
  MyTfrm test_from_world2 = MyTfrm::Identity("junk1", "junk2");
  test_from_world2 = std::move(test_from_world);
  EXPECT_EQ(test_from_world2.ToFrame(), "test");
  EXPECT_EQ(test_from_world2.FromFrame(), "world");
}

TEST(Tfrm, Equality)
{
  const MyTfrm test_from_world = MyTfrm::Identity("test", "world");
  const MyTfrm test_from_world2 = MyTfrm::Identity("test", "world");
  EXPECT_TRUE(test_from_world == test_from_world2);
}

TEST(Tfrm, Inequality)
{
  const ttfrm::Quat rot = ttfrm::Quat::Identity();
  const ttfrm::Vec3 trans = ttfrm::Vec3::Zero();
  const MyTfrm test_from_world("test", "world", rot, trans);
  const MyTfrm abc_from_world("junk1", "world", rot, trans);
  EXPECT_TRUE(test_from_world != abc_from_world);
  const MyTfrm test_from_efg("test", "junk2", rot, trans);
  EXPECT_TRUE(test_from_world != test_from_efg);
  const ttfrm::Quat other_rot = QuatFromEulerXYZ(RadFromDeg({45.0, 90.0, 20.0}));
  const MyTfrm test_from_world2("test", "world", other_rot, trans);
  EXPECT_TRUE(test_from_world != test_from_world2);
  const ttfrm::Vec3 other_trans = {2.0, 1.0, 0.0};
  const MyTfrm test_from_world3("test", "world", rot, other_trans);
  EXPECT_TRUE(test_from_world != test_from_world3);
}

TEST(Tfrm, IsApprox)
{
  const ttfrm::Quat rot = QuatFromEulerXYZ(RadFromDeg({45.0, 90.0, 20.0}));
  const ttfrm::Vec3 trans = {2.0, 1.0, 0.0};
  const MyTfrm test_from_world("test", "world", rot, trans);
  EXPECT_TRUE(test_from_world.IsApprox(test_from_world));
  const double fuzz = 1.0e-7;
  const ttfrm::Quat other_rot = rot.slerp(fuzz, rot.inverse());
  const ttfrm::Vec3 other_trans = (1.0 + fuzz) * trans;
  const MyTfrm test_from_world2("test", "world", other_rot, trans);
  EXPECT_FALSE(test_from_world == test_from_world2);
  EXPECT_TRUE(test_from_world.IsApprox(test_from_world2));
  const MyTfrm test_from_world3("test", "world", rot, other_trans);
  EXPECT_FALSE(test_from_world == test_from_world3);
  EXPECT_TRUE(test_from_world.IsApprox(test_from_world3));
  // Also test for different frames
  const MyTfrm abs_from_world("junk1", "world", rot, trans);
  EXPECT_FALSE(test_from_world.IsApprox(abs_from_world));
  const MyTfrm test_from_efg("test", "junk2", rot, trans);
  EXPECT_FALSE(test_from_world.IsApprox(test_from_efg));
}

TEST(Tfrm, Frames)
{
  const MyTfrm test_from_world = MyTfrm::Identity("test", "world");
  const MyFramePair frame_pair = test_from_world.Frames();
  EXPECT_EQ(frame_pair.to_frame, "test");
  EXPECT_EQ(frame_pair.from_frame, "world");
}

TEST(Tfrm, Rotation)
{
  const ttfrm::Quat rot = QuatFromEulerXYZ(RadFromDeg({45.0, 90.0, 20.0}));
  const ttfrm::Vec3 trans = {2.0, 1.0, 0.0};
  const MyTfrm test_from_world("test", "world", rot, trans);
  const ttfrm::Quat got_rot = test_from_world.Rotation();
  static auto QuatEquality = [](const ttfrm::Quat& q1, const ttfrm::Quat& q2) {
    return (q1.w() == q2.w() && q1.x() == q2.x() && q1.y() == q2.y() && q1.z() == q2.z());
  };
  EXPECT_TRUE(QuatEquality(rot, got_rot));
}

TEST(Tfrm, Translation)
{
  const ttfrm::Quat rot = QuatFromEulerXYZ(RadFromDeg({45.0, 90.0, 20.0}));
  const ttfrm::Vec3 trans = {2.0, 1.0, 0.0};
  const MyTfrm test_from_world("test", "world", rot, trans);
  const ttfrm::Vec3 got_trans = test_from_world.Translation();
  EXPECT_EQ(trans, got_trans);
}

TEST(Tfrm, Apply)
{
  const MyTfrm test_from_world = MyTfrm::Identity("test", "world");
  const ttfrm::Vec3 vec_in_world = {3.0, 4.0, 5.0};
  const ttfrm::Vec3 vec_in_test = test_from_world.Apply(vec_in_world);
  (void) vec_in_test;
}

TEST(Tfrm, ApplyMultiply)
{
  const MyTfrm test_from_world = MyTfrm::Identity("test", "world");
  const ttfrm::Vec3 vec_in_world = {3.0, 4.0, 5.0};
  const ttfrm::Vec3 vec_in_test = test_from_world * vec_in_world;
  (void) vec_in_test;
}

TEST(Tfrm, ApplyCall)
{
  const MyTfrm test_from_world = MyTfrm::Identity("test", "world");
  const ttfrm::Vec3 vec_in_world = {3.0, 4.0, 5.0};
  const ttfrm::Vec3 vec_in_test = test_from_world(vec_in_world);
  (void) vec_in_test;
}

TEST(Tfrm, Inverse)
{
  const MyTfrm test_from_world = MyTfrm::Identity("test", "world");
  const MyTfrm world_from_test = test_from_world.Inverse();
  EXPECT_EQ(world_from_test.ToFrame(), "world");
  EXPECT_EQ(world_from_test.FromFrame(), "test");
}

TEST(Tfrm, Compose)
{
  const MyTfrm x_from_world = MyTfrm::Identity("x", "world");
  const MyTfrm test_from_x = MyTfrm::Identity("test", "x");
  const MyTfrm test_from_world = test_from_x.Compose(x_from_world);
  EXPECT_EQ(test_from_world.ToFrame(), "test");
  EXPECT_EQ(test_from_world.FromFrame(), "world");
}

TEST(Tfrm, ComposeBad)
{
  const MyTfrm x_from_world = MyTfrm::Identity("x", "world");
  const MyTfrm test_from_y = MyTfrm::Identity("test", "y");
  EXPECT_THROW(const MyTfrm garbage = test_from_y.Compose(x_from_world),
               ttfrm::TfrmComposeException);
}

TEST(Tfrm, ComposeMultiply)
{
  const MyTfrm x_from_world = MyTfrm::Identity("x", "world");
  const MyTfrm test_from_x = MyTfrm::Identity("test", "x");
  const MyTfrm test_from_world = test_from_x * x_from_world;
  EXPECT_EQ(test_from_world.ToFrame(), "test");
  EXPECT_EQ(test_from_world.FromFrame(), "world");
}

TEST(Tfrm, ComposeCall)
{
  const MyTfrm x_from_world = MyTfrm::Identity("x", "world");
  const MyTfrm test_from_x = MyTfrm::Identity("test", "x");
  const MyTfrm test_from_world = test_from_x(x_from_world);
  EXPECT_EQ(test_from_world.ToFrame(), "test");
  EXPECT_EQ(test_from_world.FromFrame(), "world");
}

TEST(Tfrm, Interpolate)
{
  const MyTfrm x_from_world = MyTfrm::Identity("x", "world");
  const MyTfrm y_from_world = MyTfrm::Identity("y", "world");
  const MyTfrm test_from_world = x_from_world.Interpolate("test", y_from_world, 0.77);
  EXPECT_EQ(test_from_world.ToFrame(), "test");
  EXPECT_EQ(test_from_world.FromFrame(), "world");
}

TEST(Tfrm, InterpolateBad)
{
  const MyTfrm x_from_world = MyTfrm::Identity("x", "world");
  const MyTfrm y_from_world = MyTfrm::Identity("y", "junk");
  EXPECT_THROW(const MyTfrm garbage = x_from_world.Interpolate("test", y_from_world, 0.77),
               ttfrm::TfrmInterpolateException);
}

TEST(Tfrm, AsIsometry)
{
  const MyTfrm test_from_world = MyTfrm::Identity("test", "world");
  const Eigen::Isometry3d iso_test_from_world = test_from_world.AsIsometry();
  (void) iso_test_from_world;
}

TEST(Tfrm, StringifyTfrm)
{
  const MyTfrm tfrm = MyTfrm::Identity("test", "world");
  const std::string tfrm_str = ttfrm::Stringify(tfrm);
  const std::string expected_str =
      "([test] <- [world], ROT: (W: 1, X: 0, Y: 0, Z: 0), TRANS: (X: 0, Y: 0, Z: 0))";
  EXPECT_EQ(tfrm_str, expected_str);
}

TEST(Tfrm, StringifyFramePair)
{
  const MyFramePair frame_pair{"test", "world"};
  const std::string tfrm_str = ttfrm::Stringify(frame_pair);
  const std::string expected_str = "([test] <- [world])";
  EXPECT_EQ(tfrm_str, expected_str);
}

TEST(Tfrm, ApplyAccuracy)
{
  const ttfrm::Quat rot = QuatFromEulerXYZ(RadFromDeg({45.0, 90.0, 20.0}));
  const ttfrm::Vec3 trans = {2.0, 1.0, 0.0};
  const MyTfrm test_from_world("test", "world", rot, trans);
  const ttfrm::Vec3 vec_in_world = {3.0, 4.0, 5.0};
  const ttfrm::Vec3 vec_in_test = test_from_world * vec_in_world;
  EXPECT_NEAR(vec_in_test.x(), 7.000000, 1.0e-6);
  EXPECT_NEAR(vec_in_test.y(), 5.409396, 1.0e-6);
  EXPECT_NEAR(vec_in_test.z(), 2.357376, 1.0e-6);
}

TEST(Tfrm, FromIsometryAccuracy)
{
  const ttfrm::Quat rot = QuatFromEulerXYZ(RadFromDeg({45.0, 90.0, 20.0}));
  const ttfrm::Vec3 trans = {2.0, 1.0, 0.0};
  Eigen::Isometry3d iso_test_from_world = Eigen::Isometry3d::Identity();
  iso_test_from_world.prerotate(rot);
  iso_test_from_world.pretranslate(trans);
  const MyTfrm test_from_world = MyTfrm::FromIsometry("test", "world", iso_test_from_world);
  const ttfrm::Vec3 vec_in_world = {3.0, 4.0, 5.0};
  const ttfrm::Vec3 vec_in_test = iso_test_from_world * vec_in_world;
  EXPECT_NEAR(vec_in_test.x(), 7.000000, 1.0e-6);
  EXPECT_NEAR(vec_in_test.y(), 5.409396, 1.0e-6);
  EXPECT_NEAR(vec_in_test.z(), 2.357376, 1.0e-6);
  const ttfrm::Vec3 vec_in_test2 = test_from_world * vec_in_world;
  EXPECT_NEAR(vec_in_test2.x(), 7.000000, 1.0e-6);
  EXPECT_NEAR(vec_in_test2.y(), 5.409396, 1.0e-6);
  EXPECT_NEAR(vec_in_test2.z(), 2.357376, 1.0e-6);
}

TEST(Tfrm, InverseAccuracy)
{
  const ttfrm::Quat rot = QuatFromEulerXYZ(RadFromDeg({45.0, 90.0, 20.0}));
  const ttfrm::Vec3 trans = {2.0, 1.0, 0.0};
  const MyTfrm test_from_world("test", "world", rot, trans);
  const MyTfrm world_from_test = test_from_world.Inverse();
  const ttfrm::Vec3 vec_in_world = {3.0, 4.0, 5.0};
  const ttfrm::Vec3 vec_in_test = test_from_world * vec_in_world;
  EXPECT_NEAR(vec_in_test.x(), 7.000000, 1.0e-6);
  EXPECT_NEAR(vec_in_test.y(), 5.409396, 1.0e-6);
  EXPECT_NEAR(vec_in_test.z(), 2.357376, 1.0e-6);
  const ttfrm::Vec3 vec_in_world2 = world_from_test * vec_in_test;
  EXPECT_NEAR(vec_in_world2.x(), 3.0, 1.0e-6);
  EXPECT_NEAR(vec_in_world2.y(), 4.0, 1.0e-6);
  EXPECT_NEAR(vec_in_world2.z(), 5.0, 1.0e-6);
}

TEST(Tfrm, ComposeAccuracy)
{
  const ttfrm::Quat rot = QuatFromEulerXYZ(RadFromDeg({45.0, 90.0, 20.0}));
  const ttfrm::Vec3 trans = {2.0, 1.0, 0.0};
  const MyTfrm x_from_world("x", "world", rot, trans);
  const MyTfrm test_from_x("test", "x", rot, trans);
  const MyTfrm test_from_world = test_from_x * x_from_world;
  const ttfrm::Vec3 vec_in_world = {3.0, 4.0, 5.0};
  const ttfrm::Vec3 vec_in_x = x_from_world * vec_in_world;
  EXPECT_NEAR(vec_in_x.x(), 7.000000, 1.0e-6);
  EXPECT_NEAR(vec_in_x.y(), 5.409396, 1.0e-6);
  EXPECT_NEAR(vec_in_x.z(), 2.357376, 1.0e-6);
  const ttfrm::Vec3 vec_in_test = test_from_x * vec_in_x;
  EXPECT_NEAR(vec_in_test.x(), 4.357376, 1.0e-6);
  EXPECT_NEAR(vec_in_test.y(), 9.630264, 1.0e-6);
  EXPECT_NEAR(vec_in_test.z(), 1.944250, 1.0e-6);
  const ttfrm::Vec3 vec_in_test2 = test_from_world * vec_in_world;
  EXPECT_NEAR(vec_in_test2.x(), 4.357376, 1.0e-6);
  EXPECT_NEAR(vec_in_test2.y(), 9.630264, 1.0e-6);
  EXPECT_NEAR(vec_in_test2.z(), 1.944250, 1.0e-6);
}

TEST(Tfrm, ComposePoseAccuracy)
{
  const ttfrm::Vec3 pose_a_trans = {0.0, 0.0, -1.0};
  const ttfrm::Quat pose_a_rot = QuatFromEulerXYZ(RadFromDeg({0.0, 90.0, 0.0}));
  const MyTfrm world_from_a("world", "a", pose_a_rot, pose_a_trans);
  // `world_from_a` also known as `pose_a_in_world`
  const ttfrm::Vec3 pose_b_trans = {1.0, 0.0, 1.0};
  const ttfrm::Quat pose_b_rot = QuatFromEulerXYZ(RadFromDeg({0.0, -90.0, 0.0}));
  const MyTfrm world_from_b("world", "b", pose_b_rot, pose_b_trans);
  const auto& pose_b_in_world = world_from_b;
  const MyTfrm pose_b_in_a = world_from_a.Inverse() * pose_b_in_world;
  const ttfrm::Vec3 pose_b_in_a_trans = pose_b_in_a.Translation();
  const ttfrm::Quat pose_b_in_a_rot = pose_b_in_a.Rotation();
  EXPECT_NEAR(pose_b_in_a_trans.x(), -2.0, 1.0e-6);
  EXPECT_NEAR(pose_b_in_a_trans.y(), 0.0, 1.0e-6);
  EXPECT_NEAR(pose_b_in_a_trans.z(), 1.0, 1.0e-6);
  // Special comparison because (0.0, 180.0, 0.0) == (180.0, 0.0, 180.0), etc
  const ttfrm::Quat expected_rot = QuatFromEulerXYZ(RadFromDeg({0.0, 180.0, 0.0}));
  EXPECT_LT(pose_b_in_a_rot.angularDistance(expected_rot), 1.0e-6);
}

TEST(Tfrm, InterpolateAccuracy)
{
  const ttfrm::Quat x_rot = QuatFromEulerXYZ(RadFromDeg({45.0, 90.0, 20.0}));
  const ttfrm::Vec3 x_trans = {2.0, 1.0, 0.0};
  const MyTfrm x_from_world("x", "world", x_rot, x_trans);
  const ttfrm::Quat y_rot = QuatFromEulerXYZ(RadFromDeg({20.0, 45.0, 90.0}));
  const ttfrm::Vec3 y_trans = {3.0, 4.0, 5.0};
  const MyTfrm y_from_world("y", "world", y_rot, y_trans);
  const MyTfrm test_from_world = x_from_world.Interpolate("test", y_from_world, 0.77);
  const ttfrm::Quat test_from_world_rot = test_from_world.Rotation();
  const ttfrm::Quat expected_rot = QuatFromEulerXYZ(RadFromDeg({14.443773, 54.984770, 84.443773}));
  EXPECT_LT(test_from_world_rot.angularDistance(expected_rot), 1.0e-6);
  const ttfrm::Vec3 test_from_world_trans = test_from_world.Translation();
  EXPECT_NEAR(test_from_world_trans.x(), 2.770000, 1.0e-6);
  EXPECT_NEAR(test_from_world_trans.y(), 3.310000, 1.0e-6);
  EXPECT_NEAR(test_from_world_trans.z(), 3.850000, 1.0e-6);
}

TEST(Tfrm, AsIsometryAccuracy)
{
  const ttfrm::Quat rot = QuatFromEulerXYZ(RadFromDeg({45.0, 90.0, 20.0}));
  const ttfrm::Vec3 trans = {2.0, 1.0, 0.0};
  const MyTfrm test_from_world("test", "world", rot, trans);
  const Eigen::Isometry3d iso_test_from_world = test_from_world.AsIsometry();
  const ttfrm::Vec3 vec_in_world = {3.0, 4.0, 5.0};
  const ttfrm::Vec3 vec_in_test = iso_test_from_world * vec_in_world;
  EXPECT_NEAR(vec_in_test.x(), 7.000000, 1.0e-6);
  EXPECT_NEAR(vec_in_test.y(), 5.409396, 1.0e-6);
  EXPECT_NEAR(vec_in_test.z(), 2.357376, 1.0e-6);
  Eigen::Isometry3d iso_test_from_world2 = Eigen::Isometry3d::Identity();
  iso_test_from_world2.prerotate(rot);
  iso_test_from_world2.pretranslate(trans);
  EXPECT_TRUE(iso_test_from_world.isApprox(iso_test_from_world2, 1.0e-3));
  const ttfrm::Vec3 vec_in_test2 = iso_test_from_world2 * vec_in_world;
  EXPECT_NEAR(vec_in_test2.x(), 7.000000, 1.0e-6);
  EXPECT_NEAR(vec_in_test2.y(), 5.409396, 1.0e-6);
  EXPECT_NEAR(vec_in_test2.z(), 2.357376, 1.0e-6);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
