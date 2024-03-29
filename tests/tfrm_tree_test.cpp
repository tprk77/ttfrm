// Copyright (c) 2019 Tim Perkins

#include <gtest/gtest.h>

#include <ttfrm/tfrm_tree.hpp>

using my_tfrm = ttfrm::tfrm<std::string>;
using my_tfrm_tree = ttfrm::tfrm_tree<std::string>;

using ttfrm::from_s;
using ttfrm::to_s;

TEST(tfrm_tree, construct)
{
  const my_tfrm_tree tf_tree(from_s("world"));
  EXPECT_EQ(tf_tree.root_frame(), "world");
}

TEST(tfrm_tree, construct_copy)
{
  const my_tfrm_tree tf_tree(from_s("world"));
  const my_tfrm_tree tf_tree2 = tf_tree;
  EXPECT_EQ(tf_tree2.root_frame(), "world");
}

TEST(tfrm_tree, construct_move)
{
  const my_tfrm_tree tf_tree(from_s("world"));
  const my_tfrm_tree tf_tree2 = std::move(tf_tree);
  EXPECT_EQ(tf_tree2.root_frame(), "world");
}

TEST(tfrm_tree, insert)
{
  my_tfrm_tree tf_tree(from_s("world"));
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  tf_tree.insert(test_from_world);
}

TEST(tfrm_tree, insert_chain)
{
  my_tfrm_tree tf_tree(from_s("world"));
  const my_tfrm x_from_world = my_tfrm::identity(to_s("x") << from_s("world"));
  const my_tfrm y_from_x = my_tfrm::identity(to_s("y") << from_s("x"));
  const my_tfrm z_from_y = my_tfrm::identity(to_s("z") << from_s("y"));
  const my_tfrm test_from_z = my_tfrm::identity(to_s("test") << from_s("z"));
  tf_tree.insert(x_from_world);
  tf_tree.insert(y_from_x);
  tf_tree.insert(z_from_y);
  tf_tree.insert(test_from_z);
}

TEST(tfrm_tree, insert_unrooted)
{
  my_tfrm_tree tf_tree(from_s("world"));
  const my_tfrm test_from_x = my_tfrm::identity(to_s("test") << from_s("x"));
  EXPECT_THROW(tf_tree.insert(test_from_x), ttfrm::insert_unrooted_exception);
}

TEST(tfrm_tree, insert_duplicate)
{
  my_tfrm_tree tf_tree(from_s("world"));
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  const my_tfrm test_from_world2 = my_tfrm::identity(to_s("test") << from_s("world"));
  tf_tree.insert(test_from_world);
  EXPECT_THROW(tf_tree.insert(test_from_world2), ttfrm::insert_duplicate_exception);
}

TEST(tfrm_tree, insert_cycle)
{
  my_tfrm_tree tf_tree(from_s("world"));
  const my_tfrm x_from_world = my_tfrm::identity(to_s("x") << from_s("world"));
  const my_tfrm test_from_x = my_tfrm::identity(to_s("test") << from_s("x"));
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  tf_tree.insert(x_from_world);
  tf_tree.insert(test_from_x);
  EXPECT_THROW(tf_tree.insert(test_from_world), ttfrm::insert_cycle_exception);
}

TEST(tfrm_tree, insert_cycle_to_self)
{
  my_tfrm_tree tf_tree(from_s("world"));
  const my_tfrm world_from_world = my_tfrm::identity(to_s("world") << from_s("world"));
  EXPECT_THROW(tf_tree.insert(world_from_world), ttfrm::insert_cycle_exception);
}

TEST(tfrm_tree, update)
{
  my_tfrm_tree tf_tree(from_s("world"));
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  tf_tree.insert(test_from_world);
  tf_tree.update(test_from_world);
}

TEST(tfrm_tree, update_chain)
{
  my_tfrm_tree tf_tree(from_s("world"));
  const my_tfrm x_from_world = my_tfrm::identity(to_s("x") << from_s("world"));
  my_tfrm y_from_x = my_tfrm::identity(to_s("y") << from_s("x"));
  const my_tfrm z_from_y = my_tfrm::identity(to_s("z") << from_s("y"));
  const my_tfrm test_from_z = my_tfrm::identity(to_s("test") << from_s("z"));
  tf_tree.insert(x_from_world);
  tf_tree.insert(y_from_x);
  tf_tree.insert(z_from_y);
  tf_tree.insert(test_from_z);
  y_from_x = my_tfrm::from_translation(to_s("y") << from_s("x"), {2.0, 1.0, 0.0});
  tf_tree.update(y_from_x);
}

TEST(tfrm_tree, update_nonexistent_to_frame)
{
  my_tfrm_tree tf_tree(from_s("world"));
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  tf_tree.insert(test_from_world);
  const my_tfrm junk1_from_world = my_tfrm::identity(to_s("junk1") << from_s("world"));
  EXPECT_THROW(tf_tree.update(junk1_from_world), ttfrm::update_nonexistent_exception);
}

TEST(tfrm_tree, update_nonexistent_from_frame)
{
  my_tfrm_tree tf_tree(from_s("world"));
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  tf_tree.insert(test_from_world);
  const my_tfrm test_from_junk2 = my_tfrm::identity(to_s("test") << from_s("junk2"));
  EXPECT_THROW(tf_tree.update(test_from_junk2), ttfrm::update_nonexistent_exception);
}

TEST(tfrm_tree, get)
{
  my_tfrm_tree tf_tree(from_s("world"));
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  tf_tree.insert(test_from_world);
  const my_tfrm test_from_world2 = tf_tree.get(to_s("test") << from_s("world"));
}

TEST(tfrm_tree, get_nonexistent_to_frame)
{
  my_tfrm_tree tf_tree(from_s("world"));
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  tf_tree.insert(test_from_world);
  EXPECT_THROW(const my_tfrm test_from_world2 = tf_tree.get(to_s("junk1") << from_s("world")),
               ttfrm::get_nonexistent_exception);
}

TEST(tfrm_tree, get_nonexistent_from_frame)
{
  my_tfrm_tree tf_tree(from_s("world"));
  const my_tfrm test_from_world = my_tfrm::identity(to_s("test") << from_s("world"));
  tf_tree.insert(test_from_world);
  EXPECT_THROW(const my_tfrm test_from_world2 = tf_tree.get(to_s("test") << from_s("junk2")),
               ttfrm::get_nonexistent_exception);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
