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
std::string StringConcat(const T&... val);

}  // namespace detail
}  // namespace ttfrm

namespace ttfrm {

using Quat = Eigen::Quaterniond;
using Vec3 = Eigen::Vector3d;

template <typename FrameId>
class Tfrm;

template <typename FrameId_>
struct FramePair {
  using FrameId = FrameId_;

  FrameId to_frame;
  FrameId from_frame;
};

template <typename FrameId>
std::string to_string(const Tfrm<FrameId>& tfrm);

template <typename FrameId>
std::string to_string(const FramePair<FrameId>& frame_pair);

template <typename FrameId_>
class Tfrm {
 public:
  using FrameId = FrameId_;

  Tfrm(const FrameId& to_frame, const FrameId& from_frame, const Quat& rot, const Vec3& trans);
  Tfrm(const Tfrm& other_tfrm);
  Tfrm(Tfrm&& other_tfrm);

  static Tfrm<FrameId> Identity(const FrameId& to_frame, const FrameId& from_frame);
  static Tfrm<FrameId> FromRotation(const FrameId& to_frame, const FrameId& from_frame,
                                    const Quat& rot);
  static Tfrm<FrameId> FromTranslation(const FrameId& to_frame, const FrameId& from_frame,
                                       const Vec3& trans);
  static Tfrm<FrameId> FromIsometry(const FrameId& to_frame, const FrameId& from_frame,
                                    const Eigen::Isometry3d& isometry);

  Tfrm<FrameId>& operator=(const Tfrm<FrameId>& other_tfrm);
  Tfrm<FrameId>& operator=(Tfrm<FrameId>&& other_tfrm);

  bool operator==(const Tfrm<FrameId>& other_tfrm) const;
  bool operator!=(const Tfrm<FrameId>& other_tfrm) const;
  bool IsApprox(const Tfrm& other_tfrm, double precision = 1.0e-6) const;

  FramePair<FrameId> Frames() const;
  FrameId ToFrame() const;
  FrameId FromFrame() const;

  Quat Rotation() const;
  Vec3 Translation() const;

  Tfrm<FrameId> Inverse() const;

  Tfrm<FrameId> Compose(const Tfrm<FrameId>& other_tfrm) const;
  Tfrm<FrameId> operator*(const Tfrm<FrameId>& other_tfrm) const;
  Tfrm<FrameId> operator()(const Tfrm<FrameId>& other_tfrm) const;

  Vec3 Apply(const Vec3& trans) const;
  Vec3 operator*(const Vec3& trans) const;
  Vec3 operator()(const Vec3& trans) const;

  Tfrm<FrameId> Interpolate(const FrameId& interp_to_frame, const Tfrm<FrameId>& other_tfrm,
                            double t) const;

  Eigen::Isometry3d AsIsometry() const;

 private:
  FrameId to_frame_;
  FrameId from_frame_;
  Quat rot_;
  Vec3 trans_;

  friend std::string to_string<FrameId>(const Tfrm<FrameId>& tfrm);
  friend std::string to_string<FrameId>(const FramePair<FrameId>& frame_pair);
};

class TfrmComposeException : public std::runtime_error {
 public:
  template <typename FrameId>
  TfrmComposeException(const Tfrm<FrameId>& tfrm, const Tfrm<FrameId>& other_tfrm);
  TfrmComposeException(const TfrmComposeException& other_comp_excp) = default;
  TfrmComposeException(TfrmComposeException&& other_comp_excp) = default;
};

class TfrmInterpolateException : public std::runtime_error {
 public:
  template <typename FrameId>
  TfrmInterpolateException(const Tfrm<FrameId>& tfrm, const Tfrm<FrameId>& other_tfrm);
  TfrmInterpolateException(const TfrmInterpolateException& other_comp_excp) = default;
  TfrmInterpolateException(TfrmInterpolateException&& other_comp_excp) = default;
};

template <typename FrameId>
Tfrm<FrameId>::Tfrm(const FrameId& to_frame, const FrameId& from_frame, const Quat& rot,
                    const Vec3& trans)
    : to_frame_(to_frame), from_frame_(from_frame), rot_(rot.normalized()), trans_(trans)
{
  // Do nothing
}

template <typename FrameId>
Tfrm<FrameId>::Tfrm(const Tfrm<FrameId>& other_tfrm)
    : to_frame_(other_tfrm.to_frame_),
      from_frame_(other_tfrm.from_frame_),
      rot_(other_tfrm.rot_),
      trans_(other_tfrm.trans_)
{
  // Do nothing
}

template <typename FrameId>
Tfrm<FrameId>::Tfrm(Tfrm&& other_tfrm)
    : to_frame_(std::move(other_tfrm.to_frame_)),
      from_frame_(std::move(other_tfrm.from_frame_)),
      rot_(std::move(other_tfrm.rot_)),
      trans_(std::move(other_tfrm.trans_))
{
  // Do nothing
}

template <typename FrameId>
Tfrm<FrameId> Tfrm<FrameId>::Identity(const FrameId& to_frame, const FrameId& from_frame)
{
  return Tfrm<FrameId>(to_frame, from_frame, Quat::Identity(), Vec3::Zero());
}

template <typename FrameId>
Tfrm<FrameId> Tfrm<FrameId>::FromRotation(const FrameId& to_frame, const FrameId& from_frame,
                                          const Quat& rot)
{
  return Tfrm<FrameId>(to_frame, from_frame, rot, Vec3::Zero());
}

template <typename FrameId>
Tfrm<FrameId> Tfrm<FrameId>::FromTranslation(const FrameId& to_frame, const FrameId& from_frame,
                                             const Vec3& trans)
{
  return Tfrm<FrameId>(to_frame, from_frame, Quat::Identity(), trans);
}

template <typename FrameId>
Tfrm<FrameId> Tfrm<FrameId>::FromIsometry(const FrameId& to_frame, const FrameId& from_frame,
                                          const Eigen::Isometry3d& isometry)
{
  return Tfrm<FrameId>(to_frame, from_frame, Quat(isometry.rotation()), isometry.translation());
}

template <typename FrameId>
Tfrm<FrameId>& Tfrm<FrameId>::operator=(const Tfrm& other_tfrm)
{
  to_frame_ = other_tfrm.to_frame_;
  from_frame_ = other_tfrm.from_frame_;
  rot_ = other_tfrm.rot_;
  trans_ = other_tfrm.trans_;
  return *this;
}

template <typename FrameId>
Tfrm<FrameId>& Tfrm<FrameId>::operator=(Tfrm&& other_tfrm)
{
  to_frame_ = std::move(other_tfrm.to_frame_);
  from_frame_ = std::move(other_tfrm.from_frame_);
  rot_ = std::move(other_tfrm.rot_);
  trans_ = std::move(other_tfrm.trans_);
  return *this;
}

template <typename FrameId>
bool Tfrm<FrameId>::operator==(const Tfrm& other_tfrm) const
{
  static auto QuatEquality = [](const Quat& q1, const Quat& q2) {
    return (q1.w() == q2.w() && q1.x() == q2.x() && q1.y() == q2.y() && q1.z() == q2.z());
  };
  return (to_frame_ == other_tfrm.to_frame_ && from_frame_ == other_tfrm.from_frame_
          && QuatEquality(rot_, other_tfrm.rot_) && trans_ == other_tfrm.trans_);
}

template <typename FrameId>
bool Tfrm<FrameId>::operator!=(const Tfrm& other_tfrm) const
{
  return !(*this == other_tfrm);
}

template <typename FrameId>
bool Tfrm<FrameId>::IsApprox(const Tfrm& other_tfrm, double precision) const
{
  return (to_frame_ == other_tfrm.to_frame_ && from_frame_ == other_tfrm.from_frame_
          && rot_.isApprox(other_tfrm.rot_, precision)
          && trans_.isApprox(other_tfrm.trans_, precision));
}

template <typename FrameId>
FramePair<FrameId> Tfrm<FrameId>::Frames() const
{
  return {to_frame_, from_frame_};
}

template <typename FrameId>
FrameId Tfrm<FrameId>::ToFrame() const
{
  return to_frame_;
}

template <typename FrameId>
FrameId Tfrm<FrameId>::FromFrame() const
{
  return from_frame_;
}

template <typename FrameId>
Quat Tfrm<FrameId>::Rotation() const
{
  return rot_;
}

template <typename FrameId>
Vec3 Tfrm<FrameId>::Translation() const
{
  return trans_;
}

template <typename FrameId>
Tfrm<FrameId> Tfrm<FrameId>::Inverse() const
{
  const Quat inv_rot = rot_.conjugate();
  return Tfrm<FrameId>(from_frame_, to_frame_, inv_rot, inv_rot * -trans_);
}

template <typename FrameId>
Tfrm<FrameId> Tfrm<FrameId>::Compose(const Tfrm& other_tfrm) const
{
  if (from_frame_ != other_tfrm.to_frame_) {
    throw TfrmComposeException(*this, other_tfrm);
  }
  // T2(T1(v)) = R2 * (R1 * v + t1) + t2 = (R2 * R1) * v + (t2 + R2 * t1)
  return Tfrm<FrameId>(to_frame_, other_tfrm.from_frame_, rot_ * other_tfrm.rot_,
                       trans_ + rot_ * other_tfrm.trans_);
}

template <typename FrameId>
Tfrm<FrameId> Tfrm<FrameId>::operator*(const Tfrm& other_tfrm) const
{
  return Compose(other_tfrm);
}

template <typename FrameId>
Tfrm<FrameId> Tfrm<FrameId>::operator()(const Tfrm& other_tfrm) const
{
  return Compose(other_tfrm);
}

template <typename FrameId>
Vec3 Tfrm<FrameId>::Apply(const Vec3& trans) const
{
  // T(v) = R * v + t
  return Vec3(rot_ * trans + trans_);
}

template <typename FrameId>
Vec3 Tfrm<FrameId>::operator*(const Vec3& trans) const
{
  return Apply(trans);
}

template <typename FrameId>
Vec3 Tfrm<FrameId>::operator()(const Vec3& trans) const
{
  return Apply(trans);
}

template <typename FrameId>
Tfrm<FrameId> Tfrm<FrameId>::Interpolate(const FrameId& interp_to_frame,
                                         const Tfrm<FrameId>& other_tfrm, double ratio) const
{
  if (from_frame_ != other_tfrm.from_frame_) {
    throw TfrmInterpolateException(*this, other_tfrm);
  }
  return Tfrm<FrameId>(interp_to_frame, from_frame_, rot_.slerp(ratio, other_tfrm.rot_),
                       trans_ + ratio * (other_tfrm.trans_ - trans_));
}

template <typename FrameId>
Eigen::Isometry3d Tfrm<FrameId>::AsIsometry() const
{
  // The concatenation of the translation and the rotation
  return Eigen::Translation3d(trans_) * rot_;
}

template <typename FrameId>
TfrmComposeException::TfrmComposeException(const Tfrm<FrameId>& tfrm,
                                           const Tfrm<FrameId>& other_tfrm)
    : std::runtime_error(detail::StringConcat("Cannot compose transforms ", tfrm.Frames(), " and ",
                                              other_tfrm.Frames()))

{
  // Do nothing
}

template <typename FrameId>
TfrmInterpolateException::TfrmInterpolateException(const Tfrm<FrameId>& tfrm,
                                                   const Tfrm<FrameId>& other_tfrm)
    : std::runtime_error(detail::StringConcat("Cannot interpolate transforms ", tfrm.Frames(),
                                              " and ", other_tfrm.Frames()))
{
  // Do nothing
}

template <typename FrameId>
std::string to_string(const Tfrm<FrameId>& tfrm)
{
  return detail::StringConcat(
      "([", tfrm.to_frame_, "] <- [", tfrm.from_frame_, "], ROT: (W: ", tfrm.rot_.w(),
      ", X: ", tfrm.rot_.x(), ", Y: ", tfrm.rot_.y(), ", Z: ", tfrm.rot_.z(),
      "), TRANS: (X: ", tfrm.trans_.x(), ", Y: ", tfrm.trans_.y(), ", Z: ", tfrm.trans_.z(), "))");
}

template <typename FrameId>
std::string to_string(const FramePair<FrameId>& frame_pair)
{
  return detail::StringConcat("([", frame_pair.to_frame, "] <- [", frame_pair.from_frame, "])");
}

}  // namespace ttfrm

namespace ttfrm {
namespace detail {

template <int N>
std::string to_string(const char (&cstr)[N])
{
  return std::string(cstr);
}

namespace stringshim {

// WARNING Only ever use this from the string concat function to avoid
// unneccesary string copies. Can result dangling references otherwise.
const std::string& to_string(const std::string& str)
{
  return str;
}

}  // namespace stringshim

template <typename... String>
std::string StringConcatStrings(const String&... str)
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
std::string StringConcat(const T&... val)
{
  using detail::to_string;
  using detail::stringshim::to_string;
  using std::to_string;
  return StringConcatStrings(to_string(val)...);
}

}  // namespace detail
}  // namespace ttfrm

#endif  // TTFRM_TFRM_HPP
