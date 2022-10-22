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

#ifndef TTFRM_TFRM_HPP
#define TTFRM_TFRM_HPP

#include <exception>
#include <string>
#include <tuple>

#include <Eigen/Geometry>

namespace ttfrm {
namespace detail {

template <typename... T>
std::string string_concat(const T&... val);

}  // namespace detail
}  // namespace ttfrm

namespace ttfrm {

using quat = Eigen::Quaterniond;
using vec3 = Eigen::Vector3d;

template <typename FrameT>
class tfrm;

template <typename FrameT>
struct frame_pair {
  using frame_type = FrameT;

  FrameT to_frame;
  FrameT from_frame;
};

template <typename FrameT>
std::string to_string(const tfrm<FrameT>& tf);

template <typename FrameT>
std::string to_string(const frame_pair<FrameT>& fp);

template <typename FrameT>
class tfrm {
 public:
  using frame_type = FrameT;

  tfrm(const FrameT& to_frame, const FrameT& from_frame, const quat& rot, const vec3& trans);
  tfrm(const tfrm& other_tf);
  tfrm(tfrm&& other_tf);

  static tfrm identity(const FrameT& to_frame, const FrameT& from_frame);
  static tfrm from_rotation(const FrameT& to_frame, const FrameT& from_frame, const quat& rot);
  static tfrm from_translation(const FrameT& to_frame, const FrameT& from_frame, const vec3& trans);
  static tfrm from_isometry(const FrameT& to_frame, const FrameT& from_frame,
                            const Eigen::Isometry3d& isometry);

  tfrm& operator=(const tfrm& other_tf);
  tfrm& operator=(tfrm&& other_tf);

  bool operator==(const tfrm& other_tf) const;
  bool operator!=(const tfrm& other_tf) const;
  bool is_approx(const tfrm& other_tf, double precision = 1.0e-6) const;

  frame_pair<FrameT> frames() const;
  FrameT to_frame() const;
  FrameT from_frame() const;

  quat rotation() const;
  vec3 translation() const;

  tfrm inverse() const;

  tfrm compose(const tfrm& other_tf) const;
  tfrm operator*(const tfrm& other_tf) const;
  tfrm operator()(const tfrm& other_tf) const;

  vec3 apply(const vec3& trans) const;
  vec3 operator*(const vec3& trans) const;
  vec3 operator()(const vec3& trans) const;

  tfrm interpolate(const FrameT& interp_to_frame, const tfrm& other_tf, double t) const;

  Eigen::Isometry3d as_isometry() const;

 private:
  FrameT to_frame_;
  FrameT from_frame_;
  quat rot_;
  vec3 trans_;

  friend std::string to_string<FrameT>(const tfrm& tf);
};

class compose_exception : public std::runtime_error {
 public:
  template <typename FrameT>
  compose_exception(const tfrm<FrameT>& tf, const tfrm<FrameT>& other_tf);
  compose_exception(const compose_exception& other_comp_excp) = default;
  compose_exception(compose_exception&& other_comp_excp) = default;
};

class interpolate_exception : public std::runtime_error {
 public:
  template <typename FrameT>
  interpolate_exception(const tfrm<FrameT>& tf, const tfrm<FrameT>& other_tf);
  interpolate_exception(const interpolate_exception& other_comp_excp) = default;
  interpolate_exception(interpolate_exception&& other_comp_excp) = default;
};

template <typename FrameT>
tfrm<FrameT>::tfrm(const FrameT& to_frame, const FrameT& from_frame, const quat& rot,
                   const vec3& trans)
    : to_frame_(to_frame), from_frame_(from_frame), rot_(rot.normalized()), trans_(trans)
{
  // Do nothing
}

template <typename FrameT>
tfrm<FrameT>::tfrm(const tfrm& other_tf)
    : to_frame_(other_tf.to_frame_),
      from_frame_(other_tf.from_frame_),
      rot_(other_tf.rot_),
      trans_(other_tf.trans_)
{
  // Do nothing
}

template <typename FrameT>
tfrm<FrameT>::tfrm(tfrm&& other_tf)
    : to_frame_(std::move(other_tf.to_frame_)),
      from_frame_(std::move(other_tf.from_frame_)),
      rot_(std::move(other_tf.rot_)),
      trans_(std::move(other_tf.trans_))
{
  // Do nothing
}

template <typename FrameT>
tfrm<FrameT> tfrm<FrameT>::identity(const FrameT& to_frame, const FrameT& from_frame)
{
  return tfrm(to_frame, from_frame, quat::Identity(), vec3::Zero());
}

template <typename FrameT>
tfrm<FrameT> tfrm<FrameT>::from_rotation(const FrameT& to_frame, const FrameT& from_frame,
                                         const quat& rot)
{
  return tfrm(to_frame, from_frame, rot, vec3::Zero());
}

template <typename FrameT>
tfrm<FrameT> tfrm<FrameT>::from_translation(const FrameT& to_frame, const FrameT& from_frame,
                                            const vec3& trans)
{
  return tfrm(to_frame, from_frame, quat::Identity(), trans);
}

template <typename FrameT>
tfrm<FrameT> tfrm<FrameT>::from_isometry(const FrameT& to_frame, const FrameT& from_frame,
                                         const Eigen::Isometry3d& isometry)
{
  return tfrm(to_frame, from_frame, quat(isometry.rotation()), isometry.translation());
}

template <typename FrameT>
tfrm<FrameT>& tfrm<FrameT>::operator=(const tfrm& other_tf)
{
  to_frame_ = other_tf.to_frame_;
  from_frame_ = other_tf.from_frame_;
  rot_ = other_tf.rot_;
  trans_ = other_tf.trans_;
  return *this;
}

template <typename FrameT>
tfrm<FrameT>& tfrm<FrameT>::operator=(tfrm&& other_tf)
{
  to_frame_ = std::move(other_tf.to_frame_);
  from_frame_ = std::move(other_tf.from_frame_);
  rot_ = std::move(other_tf.rot_);
  trans_ = std::move(other_tf.trans_);
  return *this;
}

template <typename FrameT>
bool tfrm<FrameT>::operator==(const tfrm& other_tf) const
{
  static auto quat_equality = [](const quat& q1, const quat& q2) {
    return (q1.w() == q2.w() && q1.x() == q2.x() && q1.y() == q2.y() && q1.z() == q2.z());
  };
  return (to_frame_ == other_tf.to_frame_ && from_frame_ == other_tf.from_frame_
          && quat_equality(rot_, other_tf.rot_) && trans_ == other_tf.trans_);
}

template <typename FrameT>
bool tfrm<FrameT>::operator!=(const tfrm& other_tf) const
{
  return !(*this == other_tf);
}

template <typename FrameT>
bool tfrm<FrameT>::is_approx(const tfrm& other_tf, double precision) const
{
  return (to_frame_ == other_tf.to_frame_ && from_frame_ == other_tf.from_frame_
          && rot_.isApprox(other_tf.rot_, precision)
          && trans_.isApprox(other_tf.trans_, precision));
}

template <typename FrameT>
frame_pair<FrameT> tfrm<FrameT>::frames() const
{
  return {to_frame_, from_frame_};
}

template <typename FrameT>
FrameT tfrm<FrameT>::to_frame() const
{
  return to_frame_;
}

template <typename FrameT>
FrameT tfrm<FrameT>::from_frame() const
{
  return from_frame_;
}

template <typename FrameT>
quat tfrm<FrameT>::rotation() const
{
  return rot_;
}

template <typename FrameT>
vec3 tfrm<FrameT>::translation() const
{
  return trans_;
}

template <typename FrameT>
tfrm<FrameT> tfrm<FrameT>::inverse() const
{
  const quat inv_rot = rot_.conjugate();
  return tfrm(from_frame_, to_frame_, inv_rot, inv_rot * -trans_);
}

template <typename FrameT>
tfrm<FrameT> tfrm<FrameT>::compose(const tfrm& other_tf) const
{
  if (from_frame_ != other_tf.to_frame_) {
    throw compose_exception(*this, other_tf);
  }
  // T2(T1(v)) = R2 * (R1 * v + t1) + t2 = (R2 * R1) * v + (t2 + R2 * t1)
  return tfrm(to_frame_, other_tf.from_frame_, rot_ * other_tf.rot_,
              trans_ + rot_ * other_tf.trans_);
}

template <typename FrameT>
tfrm<FrameT> tfrm<FrameT>::operator*(const tfrm& other_tf) const
{
  return compose(other_tf);
}

template <typename FrameT>
tfrm<FrameT> tfrm<FrameT>::operator()(const tfrm& other_tf) const
{
  return compose(other_tf);
}

template <typename FrameT>
vec3 tfrm<FrameT>::apply(const vec3& trans) const
{
  // T(v) = R * v + t
  return vec3(rot_ * trans + trans_);
}

template <typename FrameT>
vec3 tfrm<FrameT>::operator*(const vec3& trans) const
{
  return apply(trans);
}

template <typename FrameT>
vec3 tfrm<FrameT>::operator()(const vec3& trans) const
{
  return apply(trans);
}

template <typename FrameT>
tfrm<FrameT> tfrm<FrameT>::interpolate(const FrameT& interp_to_frame, const tfrm& other_tf,
                                       double ratio) const
{
  if (from_frame_ != other_tf.from_frame_) {
    throw interpolate_exception(*this, other_tf);
  }
  return tfrm(interp_to_frame, from_frame_, rot_.slerp(ratio, other_tf.rot_),
              trans_ + ratio * (other_tf.trans_ - trans_));
}

template <typename FrameT>
Eigen::Isometry3d tfrm<FrameT>::as_isometry() const
{
  // The concatenation of the translation and the rotation
  return Eigen::Translation3d(trans_) * rot_;
}

template <typename FrameT>
compose_exception::compose_exception(const tfrm<FrameT>& tf, const tfrm<FrameT>& other_tf)
    : std::runtime_error(detail::string_concat("Cannot compose transforms ", tf.frames(), " and ",
                                               other_tf.frames()))

{
  // Do nothing
}

template <typename FrameT>
interpolate_exception::interpolate_exception(const tfrm<FrameT>& tf, const tfrm<FrameT>& other_tf)
    : std::runtime_error(detail::string_concat("Cannot interpolate transforms ", tf.frames(),
                                               " and ", other_tf.frames()))
{
  // Do nothing
}

template <typename FrameT>
std::string to_string(const tfrm<FrameT>& tf)
{
  return detail::string_concat(
      "([", tf.to_frame_, "] <- [", tf.from_frame_, "], ROT: (W: ", tf.rot_.w(),
      ", X: ", tf.rot_.x(), ", Y: ", tf.rot_.y(), ", Z: ", tf.rot_.z(),
      "), TRANS: (X: ", tf.trans_.x(), ", Y: ", tf.trans_.y(), ", Z: ", tf.trans_.z(), "))");
}

template <typename FrameT>
std::string to_string(const frame_pair<FrameT>& fp)
{
  return detail::string_concat("([", fp.to_frame, "] <- [", fp.from_frame, "])");
}

}  // namespace ttfrm

namespace ttfrm {
namespace detail {

namespace stringshim {

template <int N>
std::string to_string(const char (&cstr)[N])
{
  // Because this is used with string constants, we know N will always be 1 or
  // greater, due to the final null byte terminator.
  return std::string(cstr, N - 1);
}

// WARNING Only ever use this from the string concat function to avoid
// unneccesary string copies. Can result dangling references otherwise.
const std::string& to_string(const std::string& str)
{
  return str;
}

}  // namespace stringshim

template <typename... String>
std::string string_concat_strs(const String&... str)
{
  size_t total_len = 0;
  (void) std::initializer_list<int>{(total_len += str.length(), 0)...};
  std::string concat_str(total_len, ' ');
  auto concat_iter = concat_str.begin();
  (void) std::initializer_list<int>{
      (concat_iter = std::copy(str.begin(), str.end(), concat_iter), 0)...};
  return concat_str;
}

template <typename... T>
std::string string_concat(const T&... val)
{
  using std::to_string;
  using stringshim::to_string;
  return string_concat_strs(to_string(val)...);
}

}  // namespace detail
}  // namespace ttfrm

#endif  // TTFRM_TFRM_HPP
