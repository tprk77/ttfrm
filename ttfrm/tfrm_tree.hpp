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

template <typename FrameId>
class TfrmTree;

template <typename FrameId>
std::string to_string(const TfrmTree<FrameId>& tfrm_tree);

template <typename FrameId_>
class TfrmTree {
 public:
  using FrameId = FrameId_;

  explicit TfrmTree(const FrameId& root_frame);
  TfrmTree(const TfrmTree& other_tfrm_tree);
  TfrmTree(TfrmTree&& other_tfrm_tree);

  TfrmTree<FrameId>& operator=(const TfrmTree<FrameId>& other_tfrm_tree);
  TfrmTree<FrameId>& operator=(TfrmTree<FrameId>&& other_tfrm_tree);

  FrameId RootFrame() const;

  void Insert(const Tfrm<FrameId>& tfrm);
  void Update(const Tfrm<FrameId>& tfrm);

  bool Has(const FramePair<FrameId>& frame_pair) const;
  bool Has(const FrameId& to_frame, const FrameId& from_frame) const;

  Tfrm<FrameId> Get(const FramePair<FrameId>& frame_pair) const;
  Tfrm<FrameId> Get(const FrameId& to_frame, const FrameId& from_frame) const;

  // TODO This should really be using std::optional
  std::shared_ptr<Tfrm<FrameId>> GetOpt(const FramePair<FrameId>& frame_pair) const;
  std::shared_ptr<Tfrm<FrameId>> GetOpt(const FrameId& to_frame, const FrameId& from_frame) const;

 private:
  void UpdateRootTransformsFrom_(const FrameId& start_from_frame);
  bool Get_(const FramePair<FrameId>& frame_pair, Tfrm<FrameId>* tfrm_ptr) const;

  FrameId root_frame_;
  std::unordered_map<FrameId, std::vector<Tfrm<FrameId>>> tfrm_tree_;
  std::unordered_map<FrameId, Tfrm<FrameId>> frame_from_root_tfrms_;
  std::unordered_map<FrameId, Tfrm<FrameId>> root_from_frame_tfrms_;

  friend std::string to_string<FrameId>(const TfrmTree<FrameId>& tfrm_tree);
};

class TfrmTreeInsertUnrootedException : public std::runtime_error {
 public:
  template <typename FrameId>
  explicit TfrmTreeInsertUnrootedException(const FramePair<FrameId>& frame_pair);
  TfrmTreeInsertUnrootedException(const TfrmTreeInsertUnrootedException&) = default;
  TfrmTreeInsertUnrootedException(TfrmTreeInsertUnrootedException&&) = default;
};

class TfrmTreeInsertDuplicateException : public std::runtime_error {
 public:
  template <typename FrameId>
  explicit TfrmTreeInsertDuplicateException(const FramePair<FrameId>& frame_pair);
  TfrmTreeInsertDuplicateException(const TfrmTreeInsertDuplicateException&) = default;
  TfrmTreeInsertDuplicateException(TfrmTreeInsertDuplicateException&&) = default;
};

class TfrmTreeInsertCycleException : public std::runtime_error {
 public:
  template <typename FrameId>
  explicit TfrmTreeInsertCycleException(const FramePair<FrameId>& frame_pair);
  TfrmTreeInsertCycleException(const TfrmTreeInsertCycleException&) = default;
  TfrmTreeInsertCycleException(TfrmTreeInsertCycleException&&) = default;
};

class TfrmTreeUpdateNonexistentException : public std::runtime_error {
 public:
  template <typename FrameId>
  explicit TfrmTreeUpdateNonexistentException(const FramePair<FrameId>& frame_pair);
  TfrmTreeUpdateNonexistentException(const TfrmTreeUpdateNonexistentException&) = default;
  TfrmTreeUpdateNonexistentException(TfrmTreeUpdateNonexistentException&&) = default;
};

class TfrmTreeGetNonexistentException : public std::runtime_error {
 public:
  template <typename FrameId>
  explicit TfrmTreeGetNonexistentException(const FramePair<FrameId>& frame_pair);
  TfrmTreeGetNonexistentException(const TfrmTreeGetNonexistentException&) = default;
  TfrmTreeGetNonexistentException(TfrmTreeGetNonexistentException&&) = default;
};

template <typename FrameId>
TfrmTree<FrameId>::TfrmTree(const FrameId& root_frame)
    : root_frame_(root_frame),
      tfrm_tree_{{root_frame_, {}}},
      frame_from_root_tfrms_{{root_frame_, Tfrm<FrameId>::Identity(root_frame_, root_frame_)}},
      root_from_frame_tfrms_{{root_frame_, Tfrm<FrameId>::Identity(root_frame_, root_frame_)}}
{
  // Do nothing
}

template <typename FrameId>
TfrmTree<FrameId>::TfrmTree(const TfrmTree& other_tfrm_tree)
    : root_frame_(other_tfrm_tree.root_frame_),
      tfrm_tree_(other_tfrm_tree.tfrm_tree_),
      frame_from_root_tfrms_(other_tfrm_tree.frame_from_root_tfrms_),
      root_from_frame_tfrms_(other_tfrm_tree.root_from_frame_tfrms_)
{
  // Do nothing
}

template <typename FrameId>
TfrmTree<FrameId>::TfrmTree(TfrmTree&& other_tfrm_tree)
    : root_frame_(std::move(other_tfrm_tree.root_frame_)),
      tfrm_tree_(std::move(other_tfrm_tree.tfrm_tree_)),
      frame_from_root_tfrms_(std::move(other_tfrm_tree.frame_from_root_tfrms_)),
      root_from_frame_tfrms_(std::move(other_tfrm_tree.root_from_frame_tfrms_))
{
  // Do nothing
}

template <typename FrameId>
TfrmTree<FrameId>& TfrmTree<FrameId>::operator=(const TfrmTree<FrameId>& other_tfrm_tree)
{
  root_frame_ = other_tfrm_tree.root_frame_;
  tfrm_tree_ = other_tfrm_tree.tfrm_tree_;
  frame_from_root_tfrms_ = other_tfrm_tree.frame_from_root_tfrms_;
  root_from_frame_tfrms_ = other_tfrm_tree.root_from_frame_tfrms_;
}

template <typename FrameId>
TfrmTree<FrameId>& TfrmTree<FrameId>::operator=(TfrmTree<FrameId>&& other_tfrm_tree)
{
  root_frame_ = std::move(other_tfrm_tree.root_frame_);
  tfrm_tree_ = std::move(other_tfrm_tree.tfrm_tree_);
  frame_from_root_tfrms_ = std::move(other_tfrm_tree.frame_from_root_tfrms_);
  root_from_frame_tfrms_ = std::move(other_tfrm_tree.root_from_frame_tfrms_);
}

template <typename FrameId>
FrameId TfrmTree<FrameId>::RootFrame() const
{
  return root_frame_;
}

template <typename FrameId>
void TfrmTree<FrameId>::Insert(const Tfrm<FrameId>& tfrm)
{
  const FrameId& to_frame = tfrm.ToFrame();
  const FrameId& from_frame = tfrm.FromFrame();
  // Ensure the transform is rooted in the tree
  const auto tfrm_tree_iter = tfrm_tree_.find(from_frame);
  if (tfrm_tree_iter == tfrm_tree_.end()) {
    throw TfrmTreeInsertUnrootedException(tfrm.Frames());
  }
  // Ensure the transform is not a duplicate
  std::vector<Tfrm<FrameId>>& tfrms_for_frame = std::get<1>(*tfrm_tree_iter);
  const auto tfrm_iter = std::find_if(
      tfrms_for_frame.begin(), tfrms_for_frame.end(),
      [&](const Tfrm<FrameId>& other_tfrm) { return other_tfrm.ToFrame() == to_frame; });
  if (tfrm_iter != tfrms_for_frame.end()) {
    throw TfrmTreeInsertDuplicateException(tfrm.Frames());
  }
  // Ensure the transform would not create a cycle
  if (to_frame == from_frame) {
    throw TfrmTreeInsertCycleException(tfrm.Frames());
  }
  const auto to_frame_iter = frame_from_root_tfrms_.find(to_frame);
  if (to_frame_iter != frame_from_root_tfrms_.end()) {
    throw TfrmTreeInsertCycleException(tfrm.Frames());
  }
  // Insert this transform in the tree
  tfrms_for_frame.push_back(tfrm);
  tfrm_tree_.emplace(to_frame, std::vector<Tfrm<FrameId>>{});
  // Insert the corresponding transforms relative to the root frame
  const Tfrm<FrameId>& from_frame_from_root = frame_from_root_tfrms_.at(from_frame);
  const Tfrm<FrameId>& root_from_from_frame = root_from_frame_tfrms_.at(from_frame);
  frame_from_root_tfrms_.emplace(to_frame, tfrm * from_frame_from_root);
  root_from_frame_tfrms_.emplace(to_frame, root_from_from_frame * tfrm.Inverse());
}

template <typename FrameId>
void TfrmTree<FrameId>::Update(const Tfrm<FrameId>& tfrm)
{
  const FrameId& to_frame = tfrm.ToFrame();
  const FrameId& from_frame = tfrm.FromFrame();
  // Ensure the transform already exists
  const auto tfrm_tree_iter = tfrm_tree_.find(from_frame);
  if (tfrm_tree_iter == tfrm_tree_.end()) {
    throw TfrmTreeUpdateNonexistentException(tfrm.Frames());
  }
  std::vector<Tfrm<FrameId>>& tfrms_for_frame = std::get<1>(*tfrm_tree_iter);
  auto tfrm_iter = std::find_if(
      tfrms_for_frame.begin(), tfrms_for_frame.end(),
      [&](const Tfrm<FrameId>& other_tfrm) { return other_tfrm.ToFrame() == to_frame; });
  if (tfrm_iter == tfrms_for_frame.end()) {
    throw TfrmTreeUpdateNonexistentException(tfrm.Frames());
  }
  // Update this transform in the tree
  *tfrm_iter = tfrm;
  // Update the transforms relative to the root frame
  const Tfrm<FrameId>& from_frame_from_root = frame_from_root_tfrms_.at(from_frame);
  const Tfrm<FrameId>& root_from_from_frame = root_from_frame_tfrms_.at(from_frame);
  frame_from_root_tfrms_.at(to_frame) = tfrm * from_frame_from_root;
  root_from_frame_tfrms_.at(to_frame) = root_from_from_frame * tfrm.Inverse();
  UpdateRootTransformsFrom_(to_frame);
}

template <typename FrameId>
void TfrmTree<FrameId>::UpdateRootTransformsFrom_(const FrameId& start_from_frame)
{
  std::deque<FrameId> from_fringe = {start_from_frame};
  while (!from_fringe.empty()) {
    const FrameId from_frame = from_fringe.front();
    from_fringe.pop_front();
    // Update all the transforms for this frame
    std::vector<Tfrm<FrameId>>& tfrms_for_frame = tfrm_tree_.at(from_frame);
    const Tfrm<FrameId>& from_frame_from_root = frame_from_root_tfrms_.at(from_frame);
    const Tfrm<FrameId>& root_from_from_frame = root_from_frame_tfrms_.at(from_frame);
    for (const Tfrm<FrameId>& tfrm : tfrms_for_frame) {
      const FrameId to_frame = tfrm.ToFrame();
      frame_from_root_tfrms_.at(to_frame) = tfrm * from_frame_from_root;
      root_from_frame_tfrms_.at(to_frame) = root_from_from_frame * tfrm.Inverse();
      // Push more frames onto the fringe (they will be unique)
      from_fringe.push_back(tfrm.ToFrame());
    }
  }
}

template <typename FrameId>
bool TfrmTree<FrameId>::Has(const FramePair<FrameId>& frame_pair) const
{
  return Get_(frame_pair, nullptr);
}

template <typename FrameId>
bool TfrmTree<FrameId>::Has(const FrameId& to_frame, const FrameId& from_frame) const
{
  return Has({to_frame, from_frame});
}

template <typename FrameId>
Tfrm<FrameId> TfrmTree<FrameId>::Get(const FramePair<FrameId>& frame_pair) const
{
  Tfrm<FrameId> tfrm = Tfrm<FrameId>::Identity("INVALID", "INVALID");
  if (!Get_(frame_pair, &tfrm)) {
    throw TfrmTreeGetNonexistentException(frame_pair);
  }
  return tfrm;
}

template <typename FrameId>
Tfrm<FrameId> TfrmTree<FrameId>::Get(const FrameId& to_frame, const FrameId& from_frame) const
{
  return Get({to_frame, from_frame});
}

template <typename FrameId>
std::shared_ptr<Tfrm<FrameId>> TfrmTree<FrameId>::GetOpt(const FramePair<FrameId>& frame_pair) const
{
  Tfrm<FrameId> tfrm;
  if (!Get_(frame_pair, &tfrm)) {
    return nullptr;
  }
  return std::make_shared(tfrm);
}

template <typename FrameId>
std::shared_ptr<Tfrm<FrameId>> TfrmTree<FrameId>::GetOpt(const FrameId& to_frame,
                                                         const FrameId& from_frame) const
{
  return GetOpt({to_frame, from_frame});
}

template <typename FrameId>
bool TfrmTree<FrameId>::Get_(const FramePair<FrameId>& frame_pair, Tfrm<FrameId>* tfrm_ptr) const
{
  const auto to_frame_from_root_iter = frame_from_root_tfrms_.find(frame_pair.to_frame);
  if (to_frame_from_root_iter == frame_from_root_tfrms_.end()) {
    return false;
  }
  const auto root_from_from_frame_iter = root_from_frame_tfrms_.find(frame_pair.from_frame);
  if (root_from_from_frame_iter == root_from_frame_tfrms_.end()) {
    return false;
  }
  const Tfrm<FrameId>& to_frame_from_root = std::get<1>(*to_frame_from_root_iter);
  const Tfrm<FrameId>& root_from_from_frame = std::get<1>(*root_from_from_frame_iter);
  if (tfrm_ptr != nullptr) {
    *tfrm_ptr = to_frame_from_root * root_from_from_frame;
  }
  return true;
}

template <typename FrameId>
std::string to_string(const TfrmTree<FrameId>& tfrm_tree)
{
  // TODO Should probably print other frames in the tree as well
  return detail::StringConcat("(ROOT: [", tfrm_tree.root_frame_, "])");
}

template <typename FrameId>
TfrmTreeInsertUnrootedException::TfrmTreeInsertUnrootedException(
    const FramePair<FrameId>& frame_pair)
    : std::runtime_error(detail::StringConcat("Unrooted transform ", frame_pair))
{
  // Do nothing
}

template <typename FrameId>
TfrmTreeInsertDuplicateException::TfrmTreeInsertDuplicateException(
    const FramePair<FrameId>& frame_pair)
    : std::runtime_error(detail::StringConcat("Duplicate transform ", frame_pair))
{
  // Do nothing
}

template <typename FrameId>
TfrmTreeInsertCycleException::TfrmTreeInsertCycleException(const FramePair<FrameId>& frame_pair)
    : std::runtime_error(detail::StringConcat("Inserting cycle with transform ", frame_pair))
{
  // Do nothing
}

template <typename FrameId>
TfrmTreeUpdateNonexistentException::TfrmTreeUpdateNonexistentException(
    const FramePair<FrameId>& frame_pair)
    : std::runtime_error(detail::StringConcat("Nonexistent transform ", frame_pair))
{
  // Do nothing
}

template <typename FrameId>
TfrmTreeGetNonexistentException::TfrmTreeGetNonexistentException(
    const FramePair<FrameId>& frame_pair)
    : std::runtime_error(detail::StringConcat("Nonexistent transform ", frame_pair))
{
  // Do nothing
}

}  // namespace ttfrm

#endif  // TTFRM_TFRM_TREE_HPP
