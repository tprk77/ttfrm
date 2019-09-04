// Copyright (c) 2019 Tim Perkins

#include <gtest/gtest.h>

#include <ttfrm/tfrm_tree.hpp>

using MyTfrm = ttfrm::Tfrm<std::string>;
using MyTfrmTree = ttfrm::TfrmTree<std::string>;

TEST(TfrmTree, Construct)
{
  const MyTfrmTree tfrm_tree("world");
  EXPECT_EQ(tfrm_tree.RootFrame(), "world");
}

TEST(TfrmTree, ConstructCopy)
{
  const MyTfrmTree tfrm_tree("world");
  const MyTfrmTree tfrm_tree2 = tfrm_tree;
  EXPECT_EQ(tfrm_tree2.RootFrame(), "world");
}

TEST(TfrmTree, ConstructMove)
{
  const MyTfrmTree tfrm_tree("world");
  const MyTfrmTree tfrm_tree2 = std::move(tfrm_tree);
  EXPECT_EQ(tfrm_tree2.RootFrame(), "world");
}

TEST(TfrmTree, Insert)
{
  MyTfrmTree tfrm_tree("world");
  const MyTfrm test_from_world = MyTfrm::Identity("test", "world");
  tfrm_tree.Insert(test_from_world);
}

TEST(TfrmTree, InsertChain)
{
  MyTfrmTree tfrm_tree("world");
  const MyTfrm x_from_world = MyTfrm::Identity("x", "world");
  const MyTfrm y_from_x = MyTfrm::Identity("y", "x");
  const MyTfrm z_from_y = MyTfrm::Identity("z", "y");
  const MyTfrm test_from_z = MyTfrm::Identity("test", "z");
  tfrm_tree.Insert(x_from_world);
  tfrm_tree.Insert(y_from_x);
  tfrm_tree.Insert(z_from_y);
  tfrm_tree.Insert(test_from_z);
}

TEST(TfrmTree, InsertUnrooted)
{
  MyTfrmTree tfrm_tree("world");
  const MyTfrm test_from_x = MyTfrm::Identity("test", "x");
  EXPECT_THROW(tfrm_tree.Insert(test_from_x), ttfrm::TfrmTreeInsertUnrootedException);
}

TEST(TfrmTree, InsertDuplicate)
{
  MyTfrmTree tfrm_tree("world");
  const MyTfrm test_from_world = MyTfrm::Identity("test", "world");
  const MyTfrm test_from_world2 = MyTfrm::Identity("test", "world");
  tfrm_tree.Insert(test_from_world);
  EXPECT_THROW(tfrm_tree.Insert(test_from_world2), ttfrm::TfrmTreeInsertDuplicateException);
}

TEST(TfrmTree, InsertCycle)
{
  MyTfrmTree tfrm_tree("world");
  const MyTfrm x_from_world = MyTfrm::Identity("x", "world");
  const MyTfrm test_from_x = MyTfrm::Identity("test", "x");
  const MyTfrm test_from_world = MyTfrm::Identity("test", "world");
  tfrm_tree.Insert(x_from_world);
  tfrm_tree.Insert(test_from_x);
  EXPECT_THROW(tfrm_tree.Insert(test_from_world), ttfrm::TfrmTreeInsertCycleException);
}

TEST(TfrmTree, InsertCycleToSelf)
{
  MyTfrmTree tfrm_tree("world");
  const MyTfrm world_from_world = MyTfrm::Identity("world", "world");
  EXPECT_THROW(tfrm_tree.Insert(world_from_world), ttfrm::TfrmTreeInsertCycleException);
}

TEST(TfrmTree, Update)
{
  MyTfrmTree tfrm_tree("world");
  const MyTfrm test_from_world = MyTfrm::Identity("test", "world");
  tfrm_tree.Insert(test_from_world);
  tfrm_tree.Update(test_from_world);
}

TEST(TfrmTree, UpdateChain)
{
  MyTfrmTree tfrm_tree("world");
  const MyTfrm x_from_world = MyTfrm::Identity("x", "world");
  MyTfrm y_from_x = MyTfrm::Identity("y", "x");
  const MyTfrm z_from_y = MyTfrm::Identity("z", "y");
  const MyTfrm test_from_z = MyTfrm::Identity("test", "z");
  tfrm_tree.Insert(x_from_world);
  tfrm_tree.Insert(y_from_x);
  tfrm_tree.Insert(z_from_y);
  tfrm_tree.Insert(test_from_z);
  y_from_x = MyTfrm::FromTranslation("y", "x", {2.0, 1.0, 0.0});
  tfrm_tree.Update(y_from_x);
}

TEST(TfrmTree, UpdateNonexistent)
{
  MyTfrmTree tfrm_tree("world");
  const MyTfrm test_from_world = MyTfrm::Identity("test", "world");
  EXPECT_THROW(tfrm_tree.Update(test_from_world), ttfrm::TfrmTreeUpdateNonexistentException);
}

TEST(TfrmTree, Get)
{
  MyTfrmTree tfrm_tree("world");
  const MyTfrm test_from_world = MyTfrm::Identity("test", "world");
  tfrm_tree.Insert(test_from_world);
  const MyTfrm test_from_world2 = tfrm_tree.Get("test", "world");
}

TEST(TfrmTree, GetNonexistent)
{
  MyTfrmTree tfrm_tree("world");
  const MyTfrm test_from_world = MyTfrm::Identity("test", "world");
  EXPECT_THROW(const MyTfrm test_from_world2 = tfrm_tree.Get("test", "world"),
               ttfrm::TfrmTreeGetNonexistentException);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
