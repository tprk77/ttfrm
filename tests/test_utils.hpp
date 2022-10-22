// Copyright (c) 2019 Tim Perkins

#ifndef TTFRM_TEST_UTILS_HPP
#define TTFRM_TEST_UTILS_HPP

#include <ttfrm/tfrm.hpp>

inline ttfrm::vec3 deg_from_rad(const ttfrm::vec3& angles_rad)
{
  constexpr double TAU = 2.0 * M_PI;
  constexpr double DEG_FROM_RAD = 360.0 / TAU;
  return {DEG_FROM_RAD * angles_rad[0], DEG_FROM_RAD * angles_rad[1], DEG_FROM_RAD * angles_rad[2]};
}

inline ttfrm::vec3 rad_from_deg(const ttfrm::vec3& angles_deg)
{
  constexpr double TAU = 2.0 * M_PI;
  constexpr double RAD_FROM_DEG = TAU / 360.0;
  return {RAD_FROM_DEG * angles_deg[0], RAD_FROM_DEG * angles_deg[1], RAD_FROM_DEG * angles_deg[2]};
}

inline ttfrm::quat quat_from_euler_xyz(const ttfrm::vec3& euler_xyz)
{
  // Right handed, X forward, Y up, intrinsic rotaions
  return (Eigen::AngleAxisd(euler_xyz[0], ttfrm::vec3::UnitX())
          * Eigen::AngleAxisd(euler_xyz[1], ttfrm::vec3::UnitY())
          * Eigen::AngleAxisd(euler_xyz[2], ttfrm::vec3::UnitZ()))
      .normalized();
}

inline ttfrm::vec3 euler_xyz_from_quat(const ttfrm::quat& rot)
{
  // Right handed, X forward, Y up, intrinsic rotaions
  return rot.normalized().toRotationMatrix().eulerAngles(0, 1, 2);
}

#endif  // TTFRM_TEST_UTILS_HPP
