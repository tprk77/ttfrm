// Copyright (c) 2019 Tim Perkins

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <ttfrm/tfrm.hpp>

#include "test_utils.hpp"

struct EmptyId {
  // Blank struct for placeholder ID
};

template <typename FirstType, typename SecondType>
std::tuple<std::string, std::size_t> CompareSizes()
{
  std::string descriptor;
  std::size_t memory_saved = 0;
  const std::size_t first_size = sizeof(FirstType);
  const std::size_t second_size = sizeof(SecondType);
  if (first_size < second_size) {
    descriptor = "IS SMALLER THAN";
    memory_saved = second_size - first_size;
  }
  else if (first_size > second_size) {
    descriptor = "IS BIGGER THAN";
  }
  else {
    descriptor = "IS THE SAME SIZE AS";
  }
  return std::make_tuple(descriptor, memory_saved);
}

void MemoryUsageInfo()
{
  std::cout << "///////////////////////\n";
  std::cout << "// MEMORY USAGE INFO //\n";
  std::cout << "///////////////////////\n\n";
  std::cout << "Sizes of ttfrm types:\n";
  std::cout << "  sizeof(ttfrm::Quat) = " << sizeof(ttfrm::Quat) << "\n";
  std::cout << "  sizeof(ttfrm::Quat) = " << sizeof(ttfrm::Quat) << "\n";
  std::cout << "  sizeof(ttfrm::Vec3) = " << sizeof(ttfrm::Vec3) << "\n";
  std::cout << "  sizeof(ttfrm::Tfrm<int>) = " << sizeof(ttfrm::Tfrm<int>) << "\n";
  std::cout << "  sizeof(ttfrm::Tfrm<std::uint8_t>) = " << sizeof(ttfrm::Tfrm<std::uint8_t>)
            << "\n";
  std::cout << "  sizeof(ttfrm::Tfrm<std::sting>) = " << sizeof(ttfrm::Tfrm<std::string>) << "\n";
  std::cout << "  sizeof(ttfrm::Tfrm<(Empty Struct)>) = " << sizeof(ttfrm::Tfrm<EmptyId>) << "\n";
  std::cout << "Sizes of Eigen types:\n";
  std::cout << "  sizeof(Eigen::Matrix3d) = " << sizeof(Eigen::Matrix3d) << "\n";
  std::cout << "  sizeof(Eigen::Vector3d) = " << sizeof(Eigen::Vector3d) << "\n";
  std::cout << "  sizeof(Eigen::Matrix4d) = " << sizeof(Eigen::Matrix4d) << "\n";
  std::cout << "  sizeof(Eigen::Isometry3d) = " << sizeof(Eigen::Isometry3d) << "\n";
  const auto int_comp_tuple = CompareSizes<ttfrm::Tfrm<int>, Eigen::Isometry3d>();
  const auto str_comp_tuple = CompareSizes<ttfrm::Tfrm<std::string>, Eigen::Isometry3d>();
  const double int_kib_saved = 1000.0 * std::get<1>(int_comp_tuple) / 1024.0;
  const double str_kib_saved = 1000.0 * std::get<1>(str_comp_tuple) / 1024.0;
  std::cout << "Memory usage summary:\n";
  std::cout << "  ttfrm::Tfrm<int> " << std::get<0>(int_comp_tuple) << " Eigen::Isometry3d\n";
  if (int_kib_saved != 0.0) {
    std::cout << "    (You save " << std::get<1>(int_comp_tuple) << " B per transform, "
              << int_kib_saved << " KiB for every 1000 transforms)\n";
  }
  std::cout << "  ttfrm::Tfrm<std::string> " << std::get<0>(str_comp_tuple)
            << " Eigen::Isometry3d\n";
  if (str_kib_saved != 0.0) {
    std::cout << "    (You save " << std::get<1>(str_comp_tuple) << " B per transform, "
              << str_kib_saved << " KiB for every 1000 transforms)\n";
  }
  std::cout << "\n\n" << std::flush;
}

struct BenchFunc {
  virtual ~BenchFunc() {}
  virtual void operator()() = 0;
  virtual std::size_t GetResultHash() const = 0;
};

struct BenchResult {
  double total_time_s;
  double average_time_ns;
  std::size_t result_hash;
};

template <typename BenchFunc>
BenchResult Benchmark(const std::size_t warmup_iters, const std::size_t num_iters,
                      BenchFunc bench_func)
{
  // Warmup, not timed
  for (std::size_t ii = 0; ii < warmup_iters; ++ii) {
    bench_func();
  }
  // Start the timer and run the benchmark
  const auto start_time = std::chrono::steady_clock::now();
  for (std::size_t ii = 0; ii < num_iters; ++ii) {
    bench_func();
  }
  const auto end_time = std::chrono::steady_clock::now();
  const auto total_time = end_time - start_time;
  const std::size_t total_time_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(total_time).count();
  const double total_time_s = total_time_ns / 1.0e9;
  const double average_time_ns = total_time_ns / num_iters;
  const std::size_t result_hash = bench_func.GetResultHash();
  return {total_time_s, average_time_ns, result_hash};
}

template <typename T>
std::size_t HashCombine(const T& val)
{
  return std::hash<T>()(val);
}

template <typename T, typename... Rest>
std::size_t HashCombine(const T& val, const Rest&... rest)
{
  const std::size_t orig_hash = std::hash<T>()(val);
  const std::size_t next_hash = HashCombine(rest...);
  return orig_hash ^ (next_hash + 0x9e3779b9 + (orig_hash << 6) + (orig_hash >> 2));
}

std::size_t HashVec3(const ttfrm::Vec3& vec)
{
  return HashCombine(vec.x(), vec.y(), vec.z());
}

std::size_t HashQuat(const ttfrm::Quat& quat)
{
  return HashCombine(quat.w(), quat.x(), quat.y(), quat.z());
}

std::size_t HashTfrm(const ttfrm::Tfrm<int>& tfrm)
{
  return HashCombine(HashQuat(tfrm.Rotation()), HashVec3(tfrm.Translation()));
}

std::size_t HashIsometry3d(const Eigen::Isometry3d& iso)
{
  const auto& matrix = iso.matrix();
  return HashCombine(matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(0, 3), matrix(1, 0),
                     matrix(1, 1), matrix(1, 2), matrix(1, 3), matrix(2, 0), matrix(2, 1),
                     matrix(2, 2), matrix(2, 3), matrix(3, 0), matrix(3, 1), matrix(3, 2),
                     matrix(3, 3));
}

struct TtfrmApplyBench : public BenchFunc {
  ttfrm::Tfrm<int> tfrm;
  ttfrm::Vec3 vec;

  explicit TtfrmApplyBench(const ttfrm::Tfrm<int>& tfrm) : tfrm(tfrm), vec({3.0, 4.0, 5.0}) {}

  void operator()() override
  {
    vec = tfrm * vec;
  }

  std::size_t GetResultHash() const override
  {
    return HashVec3(vec);
  }
};

struct TtfrmComposeBench : public BenchFunc {
  ttfrm::Tfrm<int> tfrm;

  explicit TtfrmComposeBench(const ttfrm::Tfrm<int>& tfrm) : tfrm(tfrm) {}

  void operator()() override
  {
    tfrm = tfrm * tfrm;
  }

  std::size_t GetResultHash() const override
  {
    return HashTfrm(tfrm);
  }
};

struct TtfrmInverseBench : public BenchFunc {
  ttfrm::Tfrm<int> tfrm;

  explicit TtfrmInverseBench(const ttfrm::Tfrm<int>& tfrm) : tfrm(tfrm) {}

  void operator()() override
  {
    tfrm = tfrm.Inverse();
  }

  std::size_t GetResultHash() const override
  {
    return HashTfrm(tfrm);
  }
};

struct TtfrmInterpolateBench : public BenchFunc {
  ttfrm::Tfrm<int> tfrm;
  ttfrm::Tfrm<int> other_tfrm;

  explicit TtfrmInterpolateBench(const ttfrm::Tfrm<int>& tfrm)
      : tfrm(tfrm), other_tfrm(0, 0, tfrm.Rotation().inverse(), 2.0 * tfrm.Translation())
  {
    // Do nothing
  }

  void operator()() override
  {
    tfrm = tfrm.Interpolate(0, other_tfrm, 0.01);
  }

  std::size_t GetResultHash() const override
  {
    return HashTfrm(tfrm);
  }
};

struct EigenApplyBench : public BenchFunc {
  Eigen::Isometry3d iso;
  Eigen::Vector3d vec;

  explicit EigenApplyBench(const Eigen::Isometry3d& iso) : iso(iso), vec({3.0, 4.0, 5.0}) {}

  void operator()() override
  {
    vec = iso * vec;
  }

  std::size_t GetResultHash() const override
  {
    return HashVec3(vec);
  }
};

struct EigenComposeBench : public BenchFunc {
  Eigen::Isometry3d iso;

  explicit EigenComposeBench(const Eigen::Isometry3d& iso) : iso(iso) {}

  void operator()() override
  {
    iso = iso * iso;
  }

  std::size_t GetResultHash() const override
  {
    return HashIsometry3d(iso);
  }
};

struct EigenInverseBench : public BenchFunc {
  Eigen::Isometry3d iso;

  explicit EigenInverseBench(const Eigen::Isometry3d& iso) : iso(iso) {}

  void operator()() override
  {
    iso = iso.inverse();
  }

  std::size_t GetResultHash() const override
  {
    return HashIsometry3d(iso);
  }
};

struct EigenInterpolateBench : public BenchFunc {
  Eigen::Isometry3d iso;
  Eigen::Isometry3d other_iso;

  explicit EigenInterpolateBench(const Eigen::Isometry3d& iso)
      : iso(iso),
        other_iso(Eigen::Translation3d(2.0 * iso.translation())
                  * Eigen::Quaterniond(iso.rotation().inverse()))
  {
    // Do nothing
  }

  void operator()() override
  {
    // So you can't really do spherical linear interpolation (slerp) with matrices! So this is not
    // really a fair benchmark, but for illustrative purposes, if you really needed slerp, and all
    // you had was matrices, this is how long it would take you to do the conversions and do it.
    const Eigen::Quaterniond iso_rot(iso.rotation());
    const Eigen::Quaterniond other_iso_rot(other_iso.rotation());
    iso = (Eigen::Translation3d(iso.translation()
                                + 0.01 * (other_iso.translation() - iso.translation()))
           * iso_rot.slerp(0.01, other_iso_rot));
  }

  std::size_t GetResultHash() const override
  {
    return HashIsometry3d(iso);
  }
};

std::string StringifyTimeSummary(const BenchResult& bench_res)
{
  std::stringstream ss;
  ss << "Took " << std::fixed << std::setprecision(3) << bench_res.total_time_s << " s ("
     << std::defaultfloat << bench_res.average_time_ns << " ns average)";
  return ss.str();
}

std::string StringifyHash(const BenchResult& bench_res)
{
  std::stringstream ss;
  ss << std::hex << std::setw(16) << std::setfill('0') << bench_res.result_hash;
  return ss.str();
}

void CpuUsageInfo()
{
  std::cout << "////////////////////\n";
  std::cout << "// CPU USAGE INFO //\n";
  std::cout << "////////////////////\n\n";
  const ttfrm::Quat rot = QuatFromEulerXYZ(RadFromDeg({45.0, 90.0, 20.0}));
  const ttfrm::Vec3 trans = {2.0, 1.0, 0.0};
  const ttfrm::Tfrm<int> tfrm(0, 0, rot, trans);
  const Eigen::Isometry3d iso = tfrm.AsIsometry();
  std::cout << "Running ttfrm::Tfrm<int> benchmarks over 100M iterations... ";
  std::cout << std::flush;
  const auto tfrm_apply_res = Benchmark(1000, 100000000, TtfrmApplyBench(tfrm));
  const auto tfrm_compose_res = Benchmark(1000, 100000000, TtfrmComposeBench(tfrm));
  const auto tfrm_inverse_res = Benchmark(1000, 100000000, TtfrmInverseBench(tfrm));
  const auto tfrm_interp_res = Benchmark(1000, 100000000, TtfrmInterpolateBench(tfrm));
  std::cout << "Done!\n";
  std::cout << "Running Eigen::Isometry3d benchmarks over 100M iterations... ";
  std::cout << std::flush;
  const auto iso_apply_res = Benchmark(1000, 100000000, EigenApplyBench(iso));
  const auto iso_compose_res = Benchmark(1000, 100000000, EigenComposeBench(iso));
  const auto iso_inverse_res = Benchmark(1000, 100000000, EigenInverseBench(iso));
  const auto iso_interp_res = Benchmark(1000, 10000000, EigenInterpolateBench(iso));
  std::cout << "Done!\n\n";
  std::cout << "CPU usage summary:\n";
  std::cout << "  ttfrm::Tfrm<int> benchmarks:\n";
  std::cout << "    Apply:   " << StringifyTimeSummary(tfrm_apply_res) << "\n";
  std::cout << "    Compose: " << StringifyTimeSummary(tfrm_compose_res) << "\n";
  std::cout << "    Inverse: " << StringifyTimeSummary(tfrm_inverse_res) << "\n";
  std::cout << "    Interp:  " << StringifyTimeSummary(tfrm_interp_res) << "\n";
  std::cout << "  Eigen::Isometry3d benchmarks:\n";
  std::cout << "    Apply:   " << StringifyTimeSummary(iso_apply_res) << "\n";
  std::cout << "    Compose: " << StringifyTimeSummary(iso_compose_res) << "\n";
  std::cout << "    Inverse: " << StringifyTimeSummary(iso_inverse_res) << "\n";
  std::cout << "    Interp:  " << StringifyTimeSummary(iso_interp_res) << " [*]\n";
  std::cout << "\n";
  std::cout << "[*]: 10M iterations only. Includes required quaternion conversions for slerp.\n";
  std::cout << "\n";
  std::cout << "Result hashes:\n";
  std::cout << "  ttfrm::Tfrm<int> benchmarks:\n";
  std::cout << "    Apply:   " << StringifyHash(tfrm_apply_res) << "\n";
  std::cout << "    Compose: " << StringifyHash(tfrm_compose_res) << "\n";
  std::cout << "    Inverse: " << StringifyHash(tfrm_inverse_res) << "\n";
  std::cout << "    Interp:  " << StringifyHash(tfrm_interp_res) << "\n";
  std::cout << "  Eigen::Isometry3d benchmarks:\n";
  std::cout << "    Apply:   " << StringifyHash(iso_apply_res) << "\n";
  std::cout << "    Compose: " << StringifyHash(iso_compose_res) << "\n";
  std::cout << "    Inverse: " << StringifyHash(iso_inverse_res) << "\n";
  std::cout << "    Interp:  " << StringifyHash(iso_interp_res) << "\n";
  std::cout << "\n\n" << std::flush;
}

int main(int argc, char** argv)
{
  (void) argc;
  (void) argv;
  MemoryUsageInfo();
  CpuUsageInfo();
  return 0;
}
