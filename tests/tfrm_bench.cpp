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

template <typename FirstT, typename SecondT>
std::tuple<std::string, std::size_t> compare_sizes()
{
  std::string descriptor;
  std::size_t memory_saved = 0;
  const std::size_t first_size = sizeof(FirstT);
  const std::size_t second_size = sizeof(SecondT);
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

void memory_usage_info()
{
  std::cout << "///////////////////////\n";
  std::cout << "// MEMORY USAGE INFO //\n";
  std::cout << "///////////////////////\n\n";
  std::cout << "Sizes of ttfrm types:\n";
  std::cout << "  sizeof(ttfrm::quat) = " << sizeof(ttfrm::quat) << "\n";
  std::cout << "  sizeof(ttfrm::vec3) = " << sizeof(ttfrm::vec3) << "\n";
  std::cout << "  sizeof(ttfrm::tfrm<int>) = " << sizeof(ttfrm::tfrm<int>) << "\n";
  std::cout << "  sizeof(ttfrm::tfrm<std::uint8_t>) = " << sizeof(ttfrm::tfrm<std::uint8_t>)
            << "\n";
  std::cout << "  sizeof(ttfrm::tfrm<std::sting>) = " << sizeof(ttfrm::tfrm<std::string>) << "\n";
  std::cout << "  sizeof(ttfrm::tfrm<(Empty Struct)>) = " << sizeof(ttfrm::tfrm<EmptyId>) << "\n";
  std::cout << "Sizes of Eigen types:\n";
  std::cout << "  sizeof(Eigen::Matrix3d) = " << sizeof(Eigen::Matrix3d) << "\n";
  std::cout << "  sizeof(Eigen::Vector3d) = " << sizeof(Eigen::Vector3d) << "\n";
  std::cout << "  sizeof(Eigen::Matrix4d) = " << sizeof(Eigen::Matrix4d) << "\n";
  std::cout << "  sizeof(Eigen::Isometry3d) = " << sizeof(Eigen::Isometry3d) << "\n";
  const auto int_comp_tuple = compare_sizes<ttfrm::tfrm<int>, Eigen::Isometry3d>();
  const auto str_comp_tuple = compare_sizes<ttfrm::tfrm<std::string>, Eigen::Isometry3d>();
  const double int_kib_saved = 1000.0 * std::get<1>(int_comp_tuple) / 1024.0;
  const double str_kib_saved = 1000.0 * std::get<1>(str_comp_tuple) / 1024.0;
  std::cout << "Memory usage summary:\n";
  std::cout << "  ttfrm::tfrm<int> " << std::get<0>(int_comp_tuple) << " Eigen::Isometry3d\n";
  if (int_kib_saved != 0.0) {
    std::cout << "    (You save " << std::get<1>(int_comp_tuple) << " B per transform, "
              << int_kib_saved << " KiB for every 1000 transforms)\n";
  }
  std::cout << "  ttfrm::tfrm<std::string> " << std::get<0>(str_comp_tuple)
            << " Eigen::Isometry3d\n";
  if (str_kib_saved != 0.0) {
    std::cout << "    (You save " << std::get<1>(str_comp_tuple) << " B per transform, "
              << str_kib_saved << " KiB for every 1000 transforms)\n";
  }
  std::cout << "\n\n" << std::flush;
}

struct bench_func {
  virtual ~bench_func() {}
  virtual void operator()() = 0;
  virtual std::size_t get_result_hash() const = 0;
};

struct bench_result {
  double total_time_s;
  double average_time_ns;
  std::size_t result_hash;
};

template <typename BenchF>
bench_result benchmark(const std::size_t warmup_iters, const std::size_t num_iters, BenchF bench_fn)
{
  // Warmup, not timed
  for (std::size_t ii = 0; ii < warmup_iters; ++ii) {
    bench_fn();
  }
  // Start the timer and run the benchmark
  const auto start_time = std::chrono::steady_clock::now();
  for (std::size_t ii = 0; ii < num_iters; ++ii) {
    bench_fn();
  }
  const auto end_time = std::chrono::steady_clock::now();
  const auto total_time = end_time - start_time;
  const std::size_t total_time_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(total_time).count();
  const double total_time_s = total_time_ns / 1.0e9;
  const double average_time_ns = total_time_ns / num_iters;
  const std::size_t result_hash = bench_fn.get_result_hash();
  return {total_time_s, average_time_ns, result_hash};
}

template <typename T>
std::size_t hash_combine(const T& val)
{
  return std::hash<T>()(val);
}

template <typename T, typename... Rest>
std::size_t hash_combine(const T& val, const Rest&... rest)
{
  const std::size_t orig_hash = std::hash<T>()(val);
  const std::size_t next_hash = hash_combine(rest...);
  return orig_hash ^ (next_hash + 0x9e3779b9 + (orig_hash << 6) + (orig_hash >> 2));
}

std::size_t hash_vec3(const ttfrm::vec3& vec)
{
  return hash_combine(vec.x(), vec.y(), vec.z());
}

std::size_t hash_quat(const ttfrm::quat& rot)
{
  return hash_combine(rot.w(), rot.x(), rot.y(), rot.z());
}

std::size_t hash_tfrm(const ttfrm::tfrm<int>& tf)
{
  return hash_combine(hash_quat(tf.rotation()), hash_vec3(tf.translation()));
}

std::size_t hash_isometry3d(const Eigen::Isometry3d& iso)
{
  const auto& matrix = iso.matrix();
  return hash_combine(matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(0, 3), matrix(1, 0),
                      matrix(1, 1), matrix(1, 2), matrix(1, 3), matrix(2, 0), matrix(2, 1),
                      matrix(2, 2), matrix(2, 3), matrix(3, 0), matrix(3, 1), matrix(3, 2),
                      matrix(3, 3));
}

struct tfrm_apply_bench : public bench_func {
  ttfrm::tfrm<int> tf;
  ttfrm::vec3 vec;

  explicit tfrm_apply_bench(const ttfrm::tfrm<int>& tf) : tf(tf), vec({3.0, 4.0, 5.0}) {}

  void operator()() override
  {
    vec = tf * vec;
  }

  std::size_t get_result_hash() const override
  {
    return hash_vec3(vec);
  }
};

struct tfrm_compose_bench : public bench_func {
  ttfrm::tfrm<int> tf;

  explicit tfrm_compose_bench(const ttfrm::tfrm<int>& tf) : tf(tf) {}

  void operator()() override
  {
    tf = tf * tf;
  }

  std::size_t get_result_hash() const override
  {
    return hash_tfrm(tf);
  }
};

struct tfrm_inverse_bench : public bench_func {
  ttfrm::tfrm<int> tf;

  explicit tfrm_inverse_bench(const ttfrm::tfrm<int>& tf) : tf(tf) {}

  void operator()() override
  {
    tf = tf.inverse();
  }

  std::size_t get_result_hash() const override
  {
    return hash_tfrm(tf);
  }
};

struct tfrm_interpolate_bench : public bench_func {
  ttfrm::tfrm<int> tf;
  ttfrm::tfrm<int> other_tf;

  explicit tfrm_interpolate_bench(const ttfrm::tfrm<int>& tf)
      : tf(tf),
        other_tf(ttfrm::to(0) << ttfrm::from(0), tf.rotation().inverse(), 2.0 * tf.translation())
  {
    // Do nothing
  }

  void operator()() override
  {
    tf = tf.interpolate(ttfrm::to(0), other_tf, 0.01);
  }

  std::size_t get_result_hash() const override
  {
    return hash_tfrm(tf);
  }
};

struct iso_apply_bench : public bench_func {
  Eigen::Isometry3d iso;
  Eigen::Vector3d vec;

  explicit iso_apply_bench(const Eigen::Isometry3d& iso) : iso(iso), vec({3.0, 4.0, 5.0}) {}

  void operator()() override
  {
    vec = iso * vec;
  }

  std::size_t get_result_hash() const override
  {
    return hash_vec3(vec);
  }
};

struct iso_compose_bench : public bench_func {
  Eigen::Isometry3d iso;

  explicit iso_compose_bench(const Eigen::Isometry3d& iso) : iso(iso) {}

  void operator()() override
  {
    iso = iso * iso;
  }

  std::size_t get_result_hash() const override
  {
    return hash_isometry3d(iso);
  }
};

struct iso_inverse_bench : public bench_func {
  Eigen::Isometry3d iso;

  explicit iso_inverse_bench(const Eigen::Isometry3d& iso) : iso(iso) {}

  void operator()() override
  {
    iso = iso.inverse();
  }

  std::size_t get_result_hash() const override
  {
    return hash_isometry3d(iso);
  }
};

struct iso_interpolate_bench : public bench_func {
  Eigen::Isometry3d iso;
  Eigen::Isometry3d other_iso;

  explicit iso_interpolate_bench(const Eigen::Isometry3d& iso)
      : iso(iso),
        other_iso(Eigen::Translation3d(2.0 * iso.translation())
                  * Eigen::Quaterniond(iso.rotation().inverse()))
  {
    // Do nothing
  }

  void operator()() override
  {
    // So you can't really do spherical linear interpolation (Slerp) with matrices! So this is not
    // really a fair benchmark, but for illustrative purposes, if you really needed Slerp, and all
    // you had was matrices, this is how long it would take you to do the conversions and do it.
    const Eigen::Quaterniond iso_rot(iso.rotation());
    const Eigen::Quaterniond other_iso_rot(other_iso.rotation());
    iso = (Eigen::Translation3d(iso.translation()
                                + 0.01 * (other_iso.translation() - iso.translation()))
           * iso_rot.slerp(0.01, other_iso_rot));
  }

  std::size_t get_result_hash() const override
  {
    return hash_isometry3d(iso);
  }
};

std::string time_summary_to_str(const bench_result& bench_res)
{
  std::stringstream ss;
  ss << "Took " << std::fixed << std::setprecision(3) << bench_res.total_time_s << " s ("
     << std::defaultfloat << bench_res.average_time_ns << " ns average)";
  return ss.str();
}

std::string hash_to_str(const bench_result& bench_res)
{
  std::stringstream ss;
  ss << std::hex << std::setw(16) << std::setfill('0') << bench_res.result_hash;
  return ss.str();
}

void cpu_usage_info()
{
  std::cout << "////////////////////\n";
  std::cout << "// CPU USAGE INFO //\n";
  std::cout << "////////////////////\n\n";
  const ttfrm::quat rot = quat_from_euler_xyz(rad_from_deg({45.0, 90.0, 20.0}));
  const ttfrm::vec3 trans = {2.0, 1.0, 0.0};
  const ttfrm::tfrm<int> tf(ttfrm::to(0) << ttfrm::from(0), rot, trans);
  const Eigen::Isometry3d iso = tf.as_isometry();
  std::cout << "Running ttfrm::tfrm<int> benchmarks over 100M iterations... ";
  std::cout << std::flush;
  const auto tfrm_apply_res = benchmark(1000, 100000000, tfrm_apply_bench(tf));
  const auto tfrm_compose_res = benchmark(1000, 100000000, tfrm_compose_bench(tf));
  const auto tfrm_inverse_res = benchmark(1000, 100000000, tfrm_inverse_bench(tf));
  const auto tfrm_interp_res = benchmark(1000, 100000000, tfrm_interpolate_bench(tf));
  std::cout << "Done!\n";
  std::cout << "Running Eigen::Isometry3d benchmarks over 100M iterations... ";
  std::cout << std::flush;
  const auto iso_apply_res = benchmark(1000, 100000000, iso_apply_bench(iso));
  const auto iso_compose_res = benchmark(1000, 100000000, iso_compose_bench(iso));
  const auto iso_inverse_res = benchmark(1000, 100000000, iso_inverse_bench(iso));
  const auto iso_interp_res = benchmark(1000, 100000000, iso_interpolate_bench(iso));
  std::cout << "Done!\n\n";
  std::cout << "CPU usage summary:\n";
  std::cout << "  ttfrm::tfrm<int> benchmarks:\n";
  std::cout << "    Apply:   " << time_summary_to_str(tfrm_apply_res) << "\n";
  std::cout << "    Compose: " << time_summary_to_str(tfrm_compose_res) << "\n";
  std::cout << "    Inverse: " << time_summary_to_str(tfrm_inverse_res) << "\n";
  std::cout << "    Interp:  " << time_summary_to_str(tfrm_interp_res) << "\n";
  std::cout << "  Eigen::Isometry3d benchmarks:\n";
  std::cout << "    Apply:   " << time_summary_to_str(iso_apply_res) << "\n";
  std::cout << "    Compose: " << time_summary_to_str(iso_compose_res) << "\n";
  std::cout << "    Inverse: " << time_summary_to_str(iso_inverse_res) << "\n";
  std::cout << "    Interp:  " << time_summary_to_str(iso_interp_res) << "\n";
  std::cout << "\n";
  std::cout << "Result hashes:\n";
  std::cout << "  ttfrm::tfrm<int> benchmarks:\n";
  std::cout << "    Apply:   " << hash_to_str(tfrm_apply_res) << "\n";
  std::cout << "    Compose: " << hash_to_str(tfrm_compose_res) << "\n";
  std::cout << "    Inverse: " << hash_to_str(tfrm_inverse_res) << "\n";
  std::cout << "    Interp:  " << hash_to_str(tfrm_interp_res) << "\n";
  std::cout << "  Eigen::Isometry3d benchmarks:\n";
  std::cout << "    Apply:   " << hash_to_str(iso_apply_res) << "\n";
  std::cout << "    Compose: " << hash_to_str(iso_compose_res) << "\n";
  std::cout << "    Inverse: " << hash_to_str(iso_inverse_res) << "\n";
  std::cout << "    Interp:  " << hash_to_str(iso_interp_res) << "\n";
  std::cout << "\n\n" << std::flush;
}

int main(int argc, char** argv)
{
  (void) argc;
  (void) argv;
  memory_usage_info();
  cpu_usage_info();
  return 0;
}
