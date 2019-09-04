// Copyright (c) 2019 Tim Perkins

#include <chrono>
#include <iostream>

#include <ttfrm/tfrm.hpp>

#include "test_utils.hpp"

// Blank struct for placeholder ID
struct EmptyId {
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
  return {descriptor, memory_saved};
}

void MemoryUsageInfo()
{
  fmt::print("///////////////////////\n");
  fmt::print("// MEMORY USAGE INFO //\n");
  fmt::print("///////////////////////\n\n");
  // Just display some info about some sizes
  fmt::print("Sizes of ttfrm types:\n");
  fmt::print("  sizeof(ttfrm::Quat) = {}\n", sizeof(ttfrm::Quat));
  fmt::print("  sizeof(ttfrm::Quat) = {}\n", sizeof(ttfrm::Quat));
  fmt::print("  sizeof(ttfrm::Vec3) = {}\n", sizeof(ttfrm::Vec3));
  fmt::print("  sizeof(ttfrm::Tfrm<int>) = {}\n", sizeof(ttfrm::Tfrm<int>));
  fmt::print("  sizeof(ttfrm::Tfrm<std::uint8_t>) = {}\n", sizeof(ttfrm::Tfrm<std::uint8_t>));
  fmt::print("  sizeof(ttfrm::Tfrm<std::sting>) = {}\n", sizeof(ttfrm::Tfrm<std::string>));
  fmt::print("  sizeof(ttfrm::Tfrm<(Empty Struct)>) = {}\n", sizeof(ttfrm::Tfrm<EmptyId>));
  fmt::print("Sizes of Eigen types:\n");
  fmt::print("  sizeof(Eigen::Matrix3d) = {}\n", sizeof(Eigen::Matrix3d));
  fmt::print("  sizeof(Eigen::Vector3d) = {}\n", sizeof(Eigen::Vector3d));
  fmt::print("  sizeof(Eigen::Matrix4d) = {}\n", sizeof(Eigen::Matrix4d));
  fmt::print("  sizeof(Eigen::Isometry3d) = {}\n", sizeof(Eigen::Isometry3d));
  const auto int_comp_tuple = CompareSizes<ttfrm::Tfrm<int>, Eigen::Isometry3d>();
  const auto str_comp_tuple = CompareSizes<ttfrm::Tfrm<std::string>, Eigen::Isometry3d>();
  const double int_kib_saved = 1000.0 * std::get<1>(int_comp_tuple) / 1024.0;
  const double str_kib_saved = 1000.0 * std::get<1>(str_comp_tuple) / 1024.0;
  fmt::print("Memory usage summary:\n");
  fmt::print("  ttfrm::Tfrm<int> {} Eigen::Isometry3d\n", std::get<0>(int_comp_tuple));
  if (int_kib_saved != 0.0) {
    fmt::print("    (You save {} B per transform, {} KiB for every 1000 transforms)\n",
               std::get<1>(int_comp_tuple), int_kib_saved);
  }
  fmt::print("  ttfrm::Tfrm<std::string> {} Eigen::Isometry3d\n", std::get<0>(str_comp_tuple));
  if (str_kib_saved != 0.0) {
    fmt::print("    (You save {} B per transform, {} KiB for every 1000 transforms)\n",
               std::get<1>(str_comp_tuple), str_kib_saved);
  }
  fmt::print("\n\n");
}

template <typename BenchFunc>
std::tuple<double, double> Benchmark(const std::size_t warmup_iters, const std::size_t num_iters,
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
  const double total_time_s =
      std::chrono::duration_cast<std::chrono::milliseconds>(total_time).count() / 1000.0;
  const double avg_time_us = total_time_s * (1.0e6 / num_iters);
  return {total_time_s, avg_time_us};
}

struct BenchFunc {
  virtual ~BenchFunc() {}
  virtual void operator()() = 0;
};

struct TtfrmApplyBench : public BenchFunc {
  ttfrm::Tfrm<int> tfrm;
  ttfrm::Vec3 vec;

  explicit TtfrmApplyBench(const ttfrm::Tfrm<int>& tfrm) : tfrm(tfrm), vec({3.0, 4.0, 5.0}) {}

  void operator()() override
  {
    vec = tfrm * vec;
  }
};

struct TtfrmComposeBench : public BenchFunc {
  ttfrm::Tfrm<int> tfrm;

  explicit TtfrmComposeBench(const ttfrm::Tfrm<int>& tfrm) : tfrm(tfrm) {}

  void operator()() override
  {
    tfrm = tfrm * tfrm;
  }
};

struct TtfrmInverseBench : public BenchFunc {
  ttfrm::Tfrm<int> tfrm;

  explicit TtfrmInverseBench(const ttfrm::Tfrm<int>& tfrm) : tfrm(tfrm) {}

  void operator()() override
  {
    tfrm = tfrm.Inverse();
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
};

struct EigenComposeBench : public BenchFunc {
  Eigen::Isometry3d iso;

  explicit EigenComposeBench(const Eigen::Isometry3d& iso) : iso(iso) {}

  void operator()() override
  {
    iso = iso * iso;
  }
};

struct EigenInverseBench : public BenchFunc {
  Eigen::Isometry3d iso;

  explicit EigenInverseBench(const Eigen::Isometry3d& iso) : iso(iso) {}

  void operator()() override
  {
    iso = iso.inverse();
  }
};

void CpuUsageInfo()
{
  fmt::print("////////////////////\n");
  fmt::print("// CPU USAGE INFO //\n");
  fmt::print("////////////////////\n\n");
  const ttfrm::Quat rot = QuatFromEulerXYZ(RadFromDeg({45.0, 90.0, 20.0}));
  const ttfrm::Vec3 trans = {2.0, 1.0, 0.0};
  const ttfrm::Tfrm<int> tfrm(0, 0, rot, trans);
  const Eigen::Isometry3d iso = tfrm.AsIsometry();
  fmt::print("Running ttfrm::Tfrm<int> benchmarks over 1M iterations... ");
  std::cout << std::flush;
  const auto tfrm_apply_tuple = Benchmark(1000, 1000000, TtfrmApplyBench(tfrm));
  const auto tfrm_compose_tuple = Benchmark(1000, 1000000, TtfrmComposeBench(tfrm));
  const auto tfrm_inverse_tuple = Benchmark(1000, 1000000, TtfrmInverseBench(tfrm));
  fmt::print("Done!\n");
  fmt::print("Running Eigen::Isometry3d benchmarks over 1M iterations... ");
  std::cout << std::flush;
  const auto iso_apply_tuple = Benchmark(1000, 1000000, EigenApplyBench(iso));
  const auto iso_compose_tuple = Benchmark(1000, 1000000, EigenComposeBench(iso));
  const auto iso_inverse_tuple = Benchmark(1000, 1000000, EigenInverseBench(iso));
  fmt::print("Done!\n\n");
  fmt::print("CPU usage summary:\n");
  fmt::print("  ttfrm::Tfrm<int> benchmarks:\n");
  fmt::print("    Apply:   Took {} s ({} us average)\n", std::get<0>(tfrm_apply_tuple),
             std::get<1>(tfrm_apply_tuple));
  fmt::print("    Compose: Took {} s ({} us average)\n", std::get<0>(tfrm_compose_tuple),
             std::get<1>(tfrm_compose_tuple));
  fmt::print("    Inverse: Took {} s ({} us average)\n", std::get<0>(tfrm_inverse_tuple),
             std::get<1>(tfrm_inverse_tuple));
  fmt::print("  Eigen::Isometry3d benchmarks:\n");
  fmt::print("    Apply:   Took {} s ({} us average)\n", std::get<0>(iso_apply_tuple),
             std::get<1>(iso_apply_tuple));
  fmt::print("    Compose: Took {} s ({} us average)\n", std::get<0>(iso_compose_tuple),
             std::get<1>(iso_compose_tuple));
  fmt::print("    Inverse: Took {} s ({} us average)\n", std::get<0>(iso_inverse_tuple),
             std::get<1>(iso_inverse_tuple));
  fmt::print("\n\n");
}

int main(int argc, char** argv)
{
  (void) argc;
  (void) argv;
  MemoryUsageInfo();
  CpuUsageInfo();
  return 0;
}
