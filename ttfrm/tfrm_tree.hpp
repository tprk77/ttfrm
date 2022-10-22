// Copyright (c) 2019 Tim Perkins

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:

// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
// BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
// ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef TTFRM_TFRM_TREE_HPP
#define TTFRM_TFRM_TREE_HPP

#include <deque>
#include <memory>
#include <unordered_map>
#include <vector>

#include <ttfrm/tfrm.hpp>

namespace ttfrm {

template <typename FrameT>
class tfrm_tree;

template <typename FrameT>
std::string to_string(const tfrm_tree<FrameT>& tf_tree);

template <typename FrameT>
class tfrm_tree {
 public:
  using frame_type = FrameT;

  explicit tfrm_tree(const FrameT& root_frame);
  tfrm_tree(const tfrm_tree& other_tf_tree);
  tfrm_tree(tfrm_tree&& other_tf_tree);

  tfrm_tree& operator=(const tfrm_tree& other_tf_tree);
  tfrm_tree& operator=(tfrm_tree&& other_tf_tree);

  FrameT root_frame() const;

  void insert(const tfrm<FrameT>& tf);
  void update(const tfrm<FrameT>& tf);

  bool has(const frame_pair<FrameT>& fp) const;
  bool has(const FrameT& to_frame, const FrameT& from_frame) const;

  tfrm<FrameT> get(const frame_pair<FrameT>& fp) const;
  tfrm<FrameT> get(const FrameT& to_frame, const FrameT& from_frame) const;

  // TODO This should really be using std::optional
  std::shared_ptr<tfrm<FrameT>> get_opt(const frame_pair<FrameT>& fp) const;
  std::shared_ptr<tfrm<FrameT>> get_opt(const FrameT& to_frame, const FrameT& from_frame) const;

 private:
  void update_root_transforms_from_(const FrameT& start_from_frame);
  bool get_(const frame_pair<FrameT>& fp, tfrm<FrameT>* tf_ptr) const;

  FrameT root_frame_;
  std::unordered_map<FrameT, std::vector<tfrm<FrameT>>> tf_tree_;
  std::unordered_map<FrameT, tfrm<FrameT>> frame_from_root_tfs_;
  std::unordered_map<FrameT, tfrm<FrameT>> root_from_frame_tfs_;

  friend std::string to_string<FrameT>(const tfrm_tree& tf_tree);
};

class insert_unrooted_exception : public std::runtime_error {
 public:
  template <typename FrameT>
  explicit insert_unrooted_exception(const frame_pair<FrameT>& fp);
  insert_unrooted_exception(const insert_unrooted_exception&) = default;
  insert_unrooted_exception(insert_unrooted_exception&&) = default;
};

class insert_duplicate_exception : public std::runtime_error {
 public:
  template <typename FrameT>
  explicit insert_duplicate_exception(const frame_pair<FrameT>& fp);
  insert_duplicate_exception(const insert_duplicate_exception&) = default;
  insert_duplicate_exception(insert_duplicate_exception&&) = default;
};

class insert_cycle_exception : public std::runtime_error {
 public:
  template <typename FrameT>
  explicit insert_cycle_exception(const frame_pair<FrameT>& fp);
  insert_cycle_exception(const insert_cycle_exception&) = default;
  insert_cycle_exception(insert_cycle_exception&&) = default;
};

class update_nonexistent_exception : public std::runtime_error {
 public:
  template <typename FrameT>
  explicit update_nonexistent_exception(const frame_pair<FrameT>& fp);
  update_nonexistent_exception(const update_nonexistent_exception&) = default;
  update_nonexistent_exception(update_nonexistent_exception&&) = default;
};

class get_nonexistent_exception : public std::runtime_error {
 public:
  template <typename FrameT>
  explicit get_nonexistent_exception(const frame_pair<FrameT>& fp);
  get_nonexistent_exception(const get_nonexistent_exception&) = default;
  get_nonexistent_exception(get_nonexistent_exception&&) = default;
};

template <typename FrameT>
tfrm_tree<FrameT>::tfrm_tree(const FrameT& root_frame)
    : root_frame_(root_frame),
      tf_tree_{{root_frame_, {}}},
      frame_from_root_tfs_{{root_frame_, tfrm<FrameT>::identity(root_frame_, root_frame_)}},
      root_from_frame_tfs_{{root_frame_, tfrm<FrameT>::identity(root_frame_, root_frame_)}}
{
  // Do nothing
}

template <typename FrameT>
tfrm_tree<FrameT>::tfrm_tree(const tfrm_tree& other_tf_tree)
    : root_frame_(other_tf_tree.root_frame_),
      tf_tree_(other_tf_tree.tf_tree_),
      frame_from_root_tfs_(other_tf_tree.frame_from_root_tfs_),
      root_from_frame_tfs_(other_tf_tree.root_from_frame_tfs_)
{
  // Do nothing
}

template <typename FrameT>
tfrm_tree<FrameT>::tfrm_tree(tfrm_tree&& other_tf_tree)
    : root_frame_(std::move(other_tf_tree.root_frame_)),
      tf_tree_(std::move(other_tf_tree.tf_tree_)),
      frame_from_root_tfs_(std::move(other_tf_tree.frame_from_root_tfs_)),
      root_from_frame_tfs_(std::move(other_tf_tree.root_from_frame_tfs_))
{
  // Do nothing
}

template <typename FrameT>
tfrm_tree<FrameT>& tfrm_tree<FrameT>::operator=(const tfrm_tree& other_tf_tree)
{
  root_frame_ = other_tf_tree.root_frame_;
  tf_tree_ = other_tf_tree.tf_tree_;
  frame_from_root_tfs_ = other_tf_tree.frame_from_root_tfs_;
  root_from_frame_tfs_ = other_tf_tree.root_from_frame_tfs_;
}

template <typename FrameT>
tfrm_tree<FrameT>& tfrm_tree<FrameT>::operator=(tfrm_tree&& other_tf_tree)
{
  root_frame_ = std::move(other_tf_tree.root_frame_);
  tf_tree_ = std::move(other_tf_tree.tf_tree_);
  frame_from_root_tfs_ = std::move(other_tf_tree.frame_from_root_tfs_);
  root_from_frame_tfs_ = std::move(other_tf_tree.root_from_frame_tfs_);
}

template <typename FrameT>
FrameT tfrm_tree<FrameT>::root_frame() const
{
  return root_frame_;
}

template <typename FrameT>
void tfrm_tree<FrameT>::insert(const tfrm<FrameT>& tf)
{
  const FrameT& to_frame = tf.to_frame();
  const FrameT& from_frame = tf.from_frame();
  // Ensure the transform is rooted in the tree
  const auto tf_tree_iter = tf_tree_.find(from_frame);
  if (tf_tree_iter == tf_tree_.end()) {
    throw insert_unrooted_exception(tf.frames());
  }
  // Ensure the transform is not a duplicate
  std::vector<tfrm<FrameT>>& tfs_for_frame = std::get<1>(*tf_tree_iter);
  const auto tf_iter =
      std::find_if(tfs_for_frame.begin(), tfs_for_frame.end(),
                   [&](const tfrm<FrameT>& other_tf) { return other_tf.to_frame() == to_frame; });
  if (tf_iter != tfs_for_frame.end()) {
    throw insert_duplicate_exception(tf.frames());
  }
  // Ensure the transform would not create a cycle
  if (to_frame == from_frame) {
    throw insert_cycle_exception(tf.frames());
  }
  const auto to_frame_iter = frame_from_root_tfs_.find(to_frame);
  if (to_frame_iter != frame_from_root_tfs_.end()) {
    throw insert_cycle_exception(tf.frames());
  }
  // Insert this transform in the tree
  tfs_for_frame.push_back(tf);
  tf_tree_.emplace(to_frame, std::vector<tfrm<FrameT>>{});
  // Insert the corresponding transforms relative to the root frame
  const tfrm<FrameT>& from_frame_from_root = frame_from_root_tfs_.at(from_frame);
  const tfrm<FrameT>& root_from_from_frame = root_from_frame_tfs_.at(from_frame);
  frame_from_root_tfs_.emplace(to_frame, tf * from_frame_from_root);
  root_from_frame_tfs_.emplace(to_frame, root_from_from_frame * tf.inverse());
}

template <typename FrameT>
void tfrm_tree<FrameT>::update(const tfrm<FrameT>& tf)
{
  const FrameT& to_frame = tf.to_frame();
  const FrameT& from_frame = tf.from_frame();
  // Ensure the transform already exists
  const auto tf_tree_iter = tf_tree_.find(from_frame);
  if (tf_tree_iter == tf_tree_.end()) {
    throw update_nonexistent_exception(tf.frames());
  }
  std::vector<tfrm<FrameT>>& tfs_for_frame = std::get<1>(*tf_tree_iter);
  auto tf_iter =
      std::find_if(tfs_for_frame.begin(), tfs_for_frame.end(),
                   [&](const tfrm<FrameT>& other_tf) { return other_tf.to_frame() == to_frame; });
  if (tf_iter == tfs_for_frame.end()) {
    throw update_nonexistent_exception(tf.frames());
  }
  // Update this transform in the tree
  *tf_iter = tf;
  // Update the transforms relative to the root frame
  const tfrm<FrameT>& from_frame_from_root = frame_from_root_tfs_.at(from_frame);
  const tfrm<FrameT>& root_from_from_frame = root_from_frame_tfs_.at(from_frame);
  frame_from_root_tfs_.at(to_frame) = tf * from_frame_from_root;
  root_from_frame_tfs_.at(to_frame) = root_from_from_frame * tf.inverse();
  update_root_transforms_from_(to_frame);
}

template <typename FrameT>
void tfrm_tree<FrameT>::update_root_transforms_from_(const FrameT& start_from_frame)
{
  std::deque<FrameT> from_fringe = {start_from_frame};
  while (!from_fringe.empty()) {
    const FrameT from_frame = from_fringe.front();
    from_fringe.pop_front();
    // Update all the transforms for this frame
    std::vector<tfrm<FrameT>>& tfs_for_frame = tf_tree_.at(from_frame);
    const tfrm<FrameT>& from_frame_from_root = frame_from_root_tfs_.at(from_frame);
    const tfrm<FrameT>& root_from_from_frame = root_from_frame_tfs_.at(from_frame);
    for (const tfrm<FrameT>& tf : tfs_for_frame) {
      const FrameT to_frame = tf.to_frame();
      frame_from_root_tfs_.at(to_frame) = tf * from_frame_from_root;
      root_from_frame_tfs_.at(to_frame) = root_from_from_frame * tf.inverse();
      // Push more frames onto the fringe (they will be unique)
      from_fringe.push_back(tf.to_frame());
    }
  }
}

template <typename FrameT>
bool tfrm_tree<FrameT>::has(const frame_pair<FrameT>& fp) const
{
  return get_(fp, nullptr);
}

template <typename FrameT>
bool tfrm_tree<FrameT>::has(const FrameT& to_frame, const FrameT& from_frame) const
{
  return has({to_frame, from_frame});
}

template <typename FrameT>
tfrm<FrameT> tfrm_tree<FrameT>::get(const frame_pair<FrameT>& fp) const
{
  tfrm<FrameT> tf = tfrm<FrameT>::identity("INVALID", "INVALID");
  if (!get_(fp, &tf)) {
    throw get_nonexistent_exception(fp);
  }
  return tf;
}

template <typename FrameT>
tfrm<FrameT> tfrm_tree<FrameT>::get(const FrameT& to_frame, const FrameT& from_frame) const
{
  return get({to_frame, from_frame});
}

template <typename FrameT>
std::shared_ptr<tfrm<FrameT>> tfrm_tree<FrameT>::get_opt(const frame_pair<FrameT>& fp) const
{
  tfrm<FrameT> tf;
  if (!get_(fp, &tf)) {
    return nullptr;
  }
  return std::make_shared(tf);
}

template <typename FrameT>
std::shared_ptr<tfrm<FrameT>> tfrm_tree<FrameT>::get_opt(const FrameT& to_frame,
                                                         const FrameT& from_frame) const
{
  return get_opt({to_frame, from_frame});
}

template <typename FrameT>
bool tfrm_tree<FrameT>::get_(const frame_pair<FrameT>& fp, tfrm<FrameT>* tf_ptr) const
{
  const auto to_frame_from_root_iter = frame_from_root_tfs_.find(fp.to_frame);
  if (to_frame_from_root_iter == frame_from_root_tfs_.end()) {
    return false;
  }
  const auto root_from_from_frame_iter = root_from_frame_tfs_.find(fp.from_frame);
  if (root_from_from_frame_iter == root_from_frame_tfs_.end()) {
    return false;
  }
  const tfrm<FrameT>& to_frame_from_root = std::get<1>(*to_frame_from_root_iter);
  const tfrm<FrameT>& root_from_from_frame = std::get<1>(*root_from_from_frame_iter);
  if (tf_ptr != nullptr) {
    *tf_ptr = to_frame_from_root * root_from_from_frame;
  }
  return true;
}

template <typename FrameT>
std::string to_string(const tfrm_tree<FrameT>& tf_tree)
{
  // TODO Should probably print other frames in the tree as well
  return detail::string_concat("(ROOT: [", tf_tree.root_frame_, "])");
}

template <typename FrameT>
insert_unrooted_exception::insert_unrooted_exception(const frame_pair<FrameT>& fp)
    : std::runtime_error(detail::string_concat("Unrooted transform ", fp))
{
  // Do nothing
}

template <typename FrameT>
insert_duplicate_exception::insert_duplicate_exception(const frame_pair<FrameT>& fp)
    : std::runtime_error(detail::string_concat("Duplicate transform ", fp))
{
  // Do nothing
}

template <typename FrameT>
insert_cycle_exception::insert_cycle_exception(const frame_pair<FrameT>& fp)
    : std::runtime_error(detail::string_concat("Inserting cycle with transform ", fp))
{
  // Do nothing
}

template <typename FrameT>
update_nonexistent_exception::update_nonexistent_exception(const frame_pair<FrameT>& fp)
    : std::runtime_error(detail::string_concat("Nonexistent transform ", fp))
{
  // Do nothing
}

template <typename FrameT>
get_nonexistent_exception::get_nonexistent_exception(const frame_pair<FrameT>& fp)
    : std::runtime_error(detail::string_concat("Nonexistent transform ", fp))
{
  // Do nothing
}

}  // namespace ttfrm

#endif  // TTFRM_TFRM_TREE_HPP
