// Copyright (c) 2019 Tim Perkins

#include <gtest/gtest.h>

#include <ttfrm/tfrm.hpp>

#include "test_utils.hpp"

using my_frame_pair = ttfrm::frame_pair<std::string>;
using my_tfrm = ttfrm::tfrm<std::string>;

using ttfrm::from_s;
using ttfrm::to_s;

TEST(tfrm, construct)
{
  const ttfrm::quat rot = ttfrm::quat::Identity();
  const ttfrm::vec3 trans = ttfrm::vec3::Zero();
  const my_tfrm test_from_world(to_s("test") << from_s("world"), rot, trans);
  EXPECT_EQ(test_from_world.to_frame(), "test");
  EXPECT_EQ(test_from_world.from_frame(), "world");
}

TEST(tfrm, construct_copy)
{
  const ttfrm::quat rot = ttfrm::quat::Identity();
  const ttfrm::vec3 trans = ttfrm::vec3::Zero();
  const my_tfrm test_from_world(to_s("test") << from_s("world"), rot, trans);
  const my_tfrm test_from_world2 = test_from_world;
  EXPECT_EQ(test_from_world2.to_frame(), "test");
  EXPECT_EQ(test_from_world2.from_frame(), "world");
}

TEST(tfrm, construct_move)
{
  const ttfrm::quat rot = ttfrm::quat::Identity();
  const ttfrm::vec3 trans = ttfrm::vec3::Zero();
  my_tfrm test_from_world(to_s("test") << from_s("world"), rot, trans);
  const my_tfrm test_from_world2 = std::move(test_from_world);
  EXPECT_EQ(test_from_world2.to_frame(), "test");
  EXPECT_EQ(test_from_world2.from_frame(), "world");
}

TEST(tfrm, identity)
{
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  EXPECT_EQ(test_from_world.to_frame(), "test");
  EXPECT_EQ(test_from_world.from_frame(), "world");
}

TEST(tfrm, from_rotation)
{
  const ttfrm::quat rot = ttfrm::quat::Identity();
  const my_tfrm test_from_world = my_tfrm::from_rotation(to_s("test") << from_s("world"), rot);
  EXPECT_EQ(test_from_world.to_frame(), "test");
  EXPECT_EQ(test_from_world.from_frame(), "world");
}

TEST(tfrm, from_translation)
{
  const ttfrm::vec3 trans = ttfrm::vec3::Zero();
  const my_tfrm test_from_world = my_tfrm::from_translation(to_s("test") << from_s("world"), trans);
  EXPECT_EQ(test_from_world.to_frame(), "test");
  EXPECT_EQ(test_from_world.from_frame(), "world");
}

TEST(tfrm, from_isometry)
{
  const ttfrm::quat rot = ttfrm::quat::Identity();
  const ttfrm::vec3 trans = ttfrm::vec3::Zero();
  Eigen::Isometry3d iso_test_from_world;
  iso_test_from_world.prerotate(rot);
  iso_test_from_world.pretranslate(trans);
  const my_tfrm test_from_world =
      my_tfrm::from_isometry(to_s("test") << from_s("world"), iso_test_from_world);
  EXPECT_EQ(test_from_world.to_frame(), "test");
  EXPECT_EQ(test_from_world.from_frame(), "world");
}

TEST(tfrm, assignment)
{
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  my_tfrm test_from_world2 = my_tfrm::identity(to_s("junk1") << from_s("junk2"));
  test_from_world2 = test_from_world;
  EXPECT_EQ(test_from_world2.to_frame(), "test");
  EXPECT_EQ(test_from_world2.from_frame(), "world");
}

TEST(tfrm, assignment_move)
{
  my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  my_tfrm test_from_world2 = my_tfrm::identity(to_s("junk1") << from_s("junk2"));
  test_from_world2 = std::move(test_from_world);
  EXPECT_EQ(test_from_world2.to_frame(), "test");
  EXPECT_EQ(test_from_world2.from_frame(), "world");
}

TEST(tfrm, equality)
{
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  const my_tfrm test_from_world2 = my_tfrm::identity(to_s("test") << from_s("world"));
  EXPECT_TRUE(test_from_world == test_from_world2);
}

TEST(tfrm, inequality)
{
  const ttfrm::quat rot = ttfrm::quat::Identity();
  const ttfrm::vec3 trans = ttfrm::vec3::Zero();
  const my_tfrm test_from_world(to_s("test") << from_s("world"), rot, trans);
  const my_tfrm abc_from_world(to_s("junk1") << from_s("world"), rot, trans);
  EXPECT_TRUE(test_from_world != abc_from_world);
  const my_tfrm test_from_efg(to_s("test") << from_s("junk2"), rot, trans);
  EXPECT_TRUE(test_from_world != test_from_efg);
  const ttfrm::quat other_rot = quat_from_euler_xyz(rad_from_deg({45.0, 90.0, 20.0}));
  const my_tfrm test_from_world2(to_s("test") << from_s("world"), other_rot, trans);
  EXPECT_TRUE(test_from_world != test_from_world2);
  const ttfrm::vec3 other_trans = {2.0, 1.0, 0.0};
  const my_tfrm test_from_world3(to_s("test") << from_s("world"), rot, other_trans);
  EXPECT_TRUE(test_from_world != test_from_world3);
}

TEST(tfrm, is_approx)
{
  const ttfrm::quat rot = quat_from_euler_xyz(rad_from_deg({45.0, 90.0, 20.0}));
  const ttfrm::vec3 trans = {2.0, 1.0, 0.0};
  const my_tfrm test_from_world(to_s("test") << from_s("world"), rot, trans);
  EXPECT_TRUE(test_from_world.is_approx(test_from_world));
  const double fuzz = 1.0e-7;
  const ttfrm::quat other_rot = rot.slerp(fuzz, rot.inverse());
  const ttfrm::vec3 other_trans = (1.0 + fuzz) * trans;
  const my_tfrm test_from_world2(to_s("test") << from_s("world"), other_rot, trans);
  EXPECT_FALSE(test_from_world == test_from_world2);
  EXPECT_TRUE(test_from_world.is_approx(test_from_world2));
  const my_tfrm test_from_world3(to_s("test") << from_s("world"), rot, other_trans);
  EXPECT_FALSE(test_from_world == test_from_world3);
  EXPECT_TRUE(test_from_world.is_approx(test_from_world3));
  // Also test for different frames
  const my_tfrm abs_from_world(to_s("junk1") << from_s("world"), rot, trans);
  EXPECT_FALSE(test_from_world.is_approx(abs_from_world));
  const my_tfrm test_from_efg(to_s("test") << from_s("junk2"), rot, trans);
  EXPECT_FALSE(test_from_world.is_approx(test_from_efg));
}

TEST(tfrm, frames)
{
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  const my_frame_pair fp = test_from_world.frames();
  EXPECT_EQ(fp.to_frame, "test");
  EXPECT_EQ(fp.from_frame, "world");
}

TEST(tfrm, rotation)
{
  const ttfrm::quat rot = quat_from_euler_xyz(rad_from_deg({45.0, 90.0, 20.0}));
  const ttfrm::vec3 trans = {2.0, 1.0, 0.0};
  const my_tfrm test_from_world(to_s("test") << from_s("world"), rot, trans);
  const ttfrm::quat got_rot = test_from_world.rotation();
  static auto quat_equality = [](const ttfrm::quat& q1, const ttfrm::quat& q2) {
    return (q1.w() == q2.w() && q1.x() == q2.x() && q1.y() == q2.y() && q1.z() == q2.z());
  };
  EXPECT_TRUE(quat_equality(rot, got_rot));
}

TEST(tfrm, translation)
{
  const ttfrm::quat rot = quat_from_euler_xyz(rad_from_deg({45.0, 90.0, 20.0}));
  const ttfrm::vec3 trans = {2.0, 1.0, 0.0};
  const my_tfrm test_from_world(to_s("test") << from_s("world"), rot, trans);
  const ttfrm::vec3 got_trans = test_from_world.translation();
  EXPECT_EQ(trans, got_trans);
}

TEST(tfrm, apply)
{
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  const ttfrm::vec3 vec_in_world = {3.0, 4.0, 5.0};
  const ttfrm::vec3 vec_in_test = test_from_world.apply(vec_in_world);
  (void) vec_in_test;
}

TEST(tfrm, apply_multiply)
{
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  const ttfrm::vec3 vec_in_world = {3.0, 4.0, 5.0};
  const ttfrm::vec3 vec_in_test = test_from_world * vec_in_world;
  (void) vec_in_test;
}

TEST(tfrm, apply_call)
{
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  const ttfrm::vec3 vec_in_world = {3.0, 4.0, 5.0};
  const ttfrm::vec3 vec_in_test = test_from_world(vec_in_world);
  (void) vec_in_test;
}

TEST(tfrm, inverse)
{
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  const my_tfrm world_from_test = test_from_world.inverse();
  EXPECT_EQ(world_from_test.to_frame(), "world");
  EXPECT_EQ(world_from_test.from_frame(), "test");
}

TEST(tfrm, compose)
{
  const my_tfrm x_from_world = my_tfrm::identity(to_s("x") << from_s("world"));
  const my_tfrm test_from_x = my_tfrm::identity(to_s("test") << from_s("x"));
  const my_tfrm test_from_world = test_from_x.compose(x_from_world);
  EXPECT_EQ(test_from_world.to_frame(), "test");
  EXPECT_EQ(test_from_world.from_frame(), "world");
}

TEST(tfrm, compose_bad)
{
  const my_tfrm x_from_world = my_tfrm::identity(to_s("x") << from_s("world"));
  const my_tfrm test_from_y = my_tfrm::identity(to_s("test") << from_s("y"));
  EXPECT_THROW(const my_tfrm garbage = test_from_y.compose(x_from_world), ttfrm::compose_exception);
}

TEST(tfrm, compose_multiply)
{
  const my_tfrm x_from_world = my_tfrm::identity(to_s("x") << from_s("world"));
  const my_tfrm test_from_x = my_tfrm::identity(to_s("test") << from_s("x"));
  const my_tfrm test_from_world = test_from_x * x_from_world;
  EXPECT_EQ(test_from_world.to_frame(), "test");
  EXPECT_EQ(test_from_world.from_frame(), "world");
}

TEST(tfrm, compose_call)
{
  const my_tfrm x_from_world = my_tfrm::identity(to_s("x") << from_s("world"));
  const my_tfrm test_from_x = my_tfrm::identity(to_s("test") << from_s("x"));
  const my_tfrm test_from_world = test_from_x(x_from_world);
  EXPECT_EQ(test_from_world.to_frame(), "test");
  EXPECT_EQ(test_from_world.from_frame(), "world");
}

#if TTFRM_HAS_OPTIONAL
TEST(tfrm, compose_opt)
{
  const my_tfrm x_from_world = my_tfrm::identity(to_s("x") << from_s("world"));
  const my_tfrm test_from_x = my_tfrm::identity(to_s("test") << from_s("x"));
  const std::optional<my_tfrm> test_from_world_opt = test_from_x.compose_opt(x_from_world);
  EXPECT_TRUE(test_from_world_opt.has_value());
  EXPECT_EQ(test_from_world_opt.value().to_frame(), "test");
  EXPECT_EQ(test_from_world_opt.value().from_frame(), "world");
}
#endif

TEST(tfrm, interpolate)
{
  const my_tfrm x_from_world = my_tfrm::identity(to_s("x") << from_s("world"));
  const my_tfrm y_from_world = my_tfrm::identity(to_s("y") << from_s("world"));
  const my_tfrm test_from_world = x_from_world.interpolate(to_s("test"), y_from_world, 0.77);
  EXPECT_EQ(test_from_world.to_frame(), "test");
  EXPECT_EQ(test_from_world.from_frame(), "world");
}

TEST(tfrm, interpolate_bad)
{
  const my_tfrm x_from_world = my_tfrm::identity(to_s("x") << from_s("world"));
  const my_tfrm y_from_world = my_tfrm::identity(to_s("y") << from_s("junk"));
  EXPECT_THROW(const my_tfrm garbage = x_from_world.interpolate(to_s("test"), y_from_world, 0.77),
               ttfrm::interpolate_exception);
}

#if TTFRM_HAS_OPTIONAL
TEST(tfrm, interpolate_opt)
{
  const my_tfrm x_from_world = my_tfrm::identity(to_s("x") << from_s("world"));
  const my_tfrm y_from_world = my_tfrm::identity(to_s("y") << from_s("world"));
  const std::optional<my_tfrm> test_from_world_opt =
      x_from_world.interpolate_opt(to_s("test"), y_from_world, 0.77);
  EXPECT_TRUE(test_from_world_opt.has_value());
  EXPECT_EQ(test_from_world_opt.value().to_frame(), "test");
  EXPECT_EQ(test_from_world_opt.value().from_frame(), "world");
}
#endif

TEST(tfrm, as_isometry)
{
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  const Eigen::Isometry3d iso_test_from_world = test_from_world.as_isometry();
  (void) iso_test_from_world;
}

TEST(tfrm, to_string_tfrm)
{
  const my_tfrm tf = my_tfrm::identity(to_s("test") << from_s("world"));
  const std::string tf_str = ttfrm::to_string(tf);
  const std::string expected_str =
      "([test] << [world], ROT: (W: 1.000000, X: 0.000000, Y: 0.000000, Z: 0.000000), TRANS: (X: "
      "0.000000, Y: 0.000000, Z: 0.000000))";
  EXPECT_EQ(tf_str, expected_str);
}

TEST(tfrm, to_string_frame_pair)
{
  const my_frame_pair fp = ttfrm::to_s("test") << ttfrm::from_s("world");
  const std::string tf_str = ttfrm::to_string(fp);
  const std::string expected_str = "([test] << [world])";
  EXPECT_EQ(tf_str, expected_str);
}

TEST(tfrm, apply_accuracy)
{
  const ttfrm::quat rot = quat_from_euler_xyz(rad_from_deg({45.0, 90.0, 20.0}));
  const ttfrm::vec3 trans = {2.0, 1.0, 0.0};
  const my_tfrm test_from_world(to_s("test") << from_s("world"), rot, trans);
  const ttfrm::vec3 vec_in_world = {3.0, 4.0, 5.0};
  const ttfrm::vec3 vec_in_test = test_from_world * vec_in_world;
  EXPECT_NEAR(vec_in_test.x(), 7.000000, 1.0e-6);
  EXPECT_NEAR(vec_in_test.y(), 5.409396, 1.0e-6);
  EXPECT_NEAR(vec_in_test.z(), 2.357376, 1.0e-6);
}

TEST(tfrm, from_isometry_accuracy)
{
  const ttfrm::quat rot = quat_from_euler_xyz(rad_from_deg({45.0, 90.0, 20.0}));
  const ttfrm::vec3 trans = {2.0, 1.0, 0.0};
  Eigen::Isometry3d iso_test_from_world = Eigen::Isometry3d::Identity();
  iso_test_from_world.prerotate(rot);
  iso_test_from_world.pretranslate(trans);
  const my_tfrm test_from_world =
      my_tfrm::from_isometry(to_s("test") << from_s("world"), iso_test_from_world);
  const ttfrm::vec3 vec_in_world = {3.0, 4.0, 5.0};
  const ttfrm::vec3 vec_in_test = iso_test_from_world * vec_in_world;
  EXPECT_NEAR(vec_in_test.x(), 7.000000, 1.0e-6);
  EXPECT_NEAR(vec_in_test.y(), 5.409396, 1.0e-6);
  EXPECT_NEAR(vec_in_test.z(), 2.357376, 1.0e-6);
  const ttfrm::vec3 vec_in_test2 = test_from_world * vec_in_world;
  EXPECT_NEAR(vec_in_test2.x(), 7.000000, 1.0e-6);
  EXPECT_NEAR(vec_in_test2.y(), 5.409396, 1.0e-6);
  EXPECT_NEAR(vec_in_test2.z(), 2.357376, 1.0e-6);
}

TEST(tfrm, inverse_accuracy)
{
  const ttfrm::quat rot = quat_from_euler_xyz(rad_from_deg({45.0, 90.0, 20.0}));
  const ttfrm::vec3 trans = {2.0, 1.0, 0.0};
  const my_tfrm test_from_world(to_s("test") << from_s("world"), rot, trans);
  const my_tfrm world_from_test = test_from_world.inverse();
  const ttfrm::vec3 vec_in_world = {3.0, 4.0, 5.0};
  const ttfrm::vec3 vec_in_test = test_from_world * vec_in_world;
  EXPECT_NEAR(vec_in_test.x(), 7.000000, 1.0e-6);
  EXPECT_NEAR(vec_in_test.y(), 5.409396, 1.0e-6);
  EXPECT_NEAR(vec_in_test.z(), 2.357376, 1.0e-6);
  const ttfrm::vec3 vec_in_world2 = world_from_test * vec_in_test;
  EXPECT_NEAR(vec_in_world2.x(), 3.0, 1.0e-6);
  EXPECT_NEAR(vec_in_world2.y(), 4.0, 1.0e-6);
  EXPECT_NEAR(vec_in_world2.z(), 5.0, 1.0e-6);
}

TEST(tfrm, compose_accuracy)
{
  const ttfrm::quat rot = quat_from_euler_xyz(rad_from_deg({45.0, 90.0, 20.0}));
  const ttfrm::vec3 trans = {2.0, 1.0, 0.0};
  const my_tfrm x_from_world(to_s("x") << from_s("world"), rot, trans);
  const my_tfrm test_from_x(to_s("test") << from_s("x"), rot, trans);
  const my_tfrm test_from_world = test_from_x * x_from_world;
  const ttfrm::vec3 vec_in_world = {3.0, 4.0, 5.0};
  const ttfrm::vec3 vec_in_x = x_from_world * vec_in_world;
  EXPECT_NEAR(vec_in_x.x(), 7.000000, 1.0e-6);
  EXPECT_NEAR(vec_in_x.y(), 5.409396, 1.0e-6);
  EXPECT_NEAR(vec_in_x.z(), 2.357376, 1.0e-6);
  const ttfrm::vec3 vec_in_test = test_from_x * vec_in_x;
  EXPECT_NEAR(vec_in_test.x(), 4.357376, 1.0e-6);
  EXPECT_NEAR(vec_in_test.y(), 9.630264, 1.0e-6);
  EXPECT_NEAR(vec_in_test.z(), 1.944250, 1.0e-6);
  const ttfrm::vec3 vec_in_test2 = test_from_world * vec_in_world;
  EXPECT_NEAR(vec_in_test2.x(), 4.357376, 1.0e-6);
  EXPECT_NEAR(vec_in_test2.y(), 9.630264, 1.0e-6);
  EXPECT_NEAR(vec_in_test2.z(), 1.944250, 1.0e-6);
}

TEST(tfrm, compose_pose_accuracy)
{
  const ttfrm::vec3 pose_a_trans = {0.0, 0.0, -1.0};
  const ttfrm::quat pose_a_rot = quat_from_euler_xyz(rad_from_deg({0.0, 90.0, 0.0}));
  const my_tfrm world_from_a(to_s("world") << from_s("a"), pose_a_rot, pose_a_trans);
  // `world_from_a` also known as `pose_a_in_world`
  const ttfrm::vec3 pose_b_trans = {1.0, 0.0, 1.0};
  const ttfrm::quat pose_b_rot = quat_from_euler_xyz(rad_from_deg({0.0, -90.0, 0.0}));
  const my_tfrm world_from_b(to_s("world") << from_s("b"), pose_b_rot, pose_b_trans);
  const auto& pose_b_in_world = world_from_b;
  const my_tfrm pose_b_in_a = world_from_a.inverse() * pose_b_in_world;
  const ttfrm::vec3 pose_b_in_a_trans = pose_b_in_a.translation();
  const ttfrm::quat pose_b_in_a_rot = pose_b_in_a.rotation();
  EXPECT_NEAR(pose_b_in_a_trans.x(), -2.0, 1.0e-6);
  EXPECT_NEAR(pose_b_in_a_trans.y(), 0.0, 1.0e-6);
  EXPECT_NEAR(pose_b_in_a_trans.z(), 1.0, 1.0e-6);
  // Special comparison because (0.0, 180.0, 0.0) == (180.0, 0.0, 180.0), etc
  const ttfrm::quat expected_rot = quat_from_euler_xyz(rad_from_deg({0.0, 180.0, 0.0}));
  EXPECT_LT(pose_b_in_a_rot.angularDistance(expected_rot), 1.0e-6);
}

TEST(tfrm, interpolate_accuracy)
{
  const ttfrm::quat x_rot = quat_from_euler_xyz(rad_from_deg({45.0, 90.0, 20.0}));
  const ttfrm::vec3 x_trans = {2.0, 1.0, 0.0};
  const my_tfrm x_from_world(to_s("x") << from_s("world"), x_rot, x_trans);
  const ttfrm::quat y_rot = quat_from_euler_xyz(rad_from_deg({20.0, 45.0, 90.0}));
  const ttfrm::vec3 y_trans = {3.0, 4.0, 5.0};
  const my_tfrm y_from_world(to_s("y") << from_s("world"), y_rot, y_trans);
  const my_tfrm test_from_world = x_from_world.interpolate(to_s("test"), y_from_world, 0.77);
  const ttfrm::quat test_from_world_rot = test_from_world.rotation();
  const ttfrm::quat expected_rot =
      quat_from_euler_xyz(rad_from_deg({14.443773, 54.984770, 84.443773}));
  EXPECT_LT(test_from_world_rot.angularDistance(expected_rot), 1.0e-6);
  const ttfrm::vec3 test_from_world_trans = test_from_world.translation();
  EXPECT_NEAR(test_from_world_trans.x(), 2.770000, 1.0e-6);
  EXPECT_NEAR(test_from_world_trans.y(), 3.310000, 1.0e-6);
  EXPECT_NEAR(test_from_world_trans.z(), 3.850000, 1.0e-6);
}

TEST(tfrm, as_isometry_accuracy)
{
  const ttfrm::quat rot = quat_from_euler_xyz(rad_from_deg({45.0, 90.0, 20.0}));
  const ttfrm::vec3 trans = {2.0, 1.0, 0.0};
  const my_tfrm test_from_world(to_s("test") << from_s("world"), rot, trans);
  const Eigen::Isometry3d iso_test_from_world = test_from_world.as_isometry();
  const ttfrm::vec3 vec_in_world = {3.0, 4.0, 5.0};
  const ttfrm::vec3 vec_in_test = iso_test_from_world * vec_in_world;
  EXPECT_NEAR(vec_in_test.x(), 7.000000, 1.0e-6);
  EXPECT_NEAR(vec_in_test.y(), 5.409396, 1.0e-6);
  EXPECT_NEAR(vec_in_test.z(), 2.357376, 1.0e-6);
  Eigen::Isometry3d iso_test_from_world2 = Eigen::Isometry3d::Identity();
  iso_test_from_world2.prerotate(rot);
  iso_test_from_world2.pretranslate(trans);
  EXPECT_TRUE(iso_test_from_world.isApprox(iso_test_from_world2, 1.0e-3));
  const ttfrm::vec3 vec_in_test2 = iso_test_from_world2 * vec_in_world;
  EXPECT_NEAR(vec_in_test2.x(), 7.000000, 1.0e-6);
  EXPECT_NEAR(vec_in_test2.y(), 5.409396, 1.0e-6);
  EXPECT_NEAR(vec_in_test2.z(), 2.357376, 1.0e-6);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
