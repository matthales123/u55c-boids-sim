#include "u55c_framework/runner.hpp"

#include "u55c_framework/buffer_manager.hpp"
#include "u55c_framework/profiler.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <vector>

#if defined(U55C_ENABLE_XRT)
#include <xrt/experimental/xrt_xclbin.h>
#include <xrt/xrt_bo.h>
#include <xrt/xrt_device.h>
#include <xrt/xrt_kernel.h>
#endif

namespace u55c_framework {
namespace {

constexpr float kVisualRange = 75.0f;
constexpr float kProtectedRange = 8.0f;
constexpr float kCenteringFactor = 0.0005f;
constexpr float kAvoidFactor = 0.05f;
constexpr float kMatchingFactor = 0.05f;
constexpr float kTurnFactor = 0.2f;
constexpr float kMinSpeed = 2.0f;
constexpr float kMaxSpeed = 3.0f;

constexpr float kHitTop = 100.0f;
constexpr float kHitBottom = 380.0f;
constexpr float kHitLeft = 100.0f;
constexpr float kHitRight = 540.0f;

constexpr float kWorldMinX = 0.0f;
constexpr float kWorldMaxX = 640.0f;
constexpr float kWorldMinY = 0.0f;
constexpr float kWorldMaxY = 480.0f;

std::string utc_now_iso8601() {
  const std::time_t now = std::time(nullptr);
  std::tm tm_utc{};
#if defined(_WIN32)
  gmtime_s(&tm_utc, &now);
#else
  gmtime_r(&now, &tm_utc);
#endif

  std::ostringstream oss;
  oss << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%SZ");
  return oss.str();
}

double throughput_gbps(const std::size_t elements, const double total_ms) {
  if (total_ms <= 0.0) {
    return 0.0;
  }

  const double bytes_moved = static_cast<double>(elements) * sizeof(std::uint32_t) * 3.0;
  const double bits_moved = bytes_moved * 8.0;
  return bits_moved / (total_ms * 1.0e6);
}

std::uint32_t pack_xy(const std::int16_t x, const std::int16_t y) {
  const auto hi = static_cast<std::uint32_t>(static_cast<std::uint16_t>(x));
  const auto lo = static_cast<std::uint32_t>(static_cast<std::uint16_t>(y));
  return (hi << 16) | lo;
}

std::int16_t unpack_hi(const std::uint32_t value) {
  return static_cast<std::int16_t>((value >> 16) & 0xFFFFu);
}

std::int16_t unpack_lo(const std::uint32_t value) {
  return static_cast<std::int16_t>(value & 0xFFFFu);
}

short clamp_to_i16(const float value) {
  int rounded = static_cast<int>(value >= 0.0f ? value + 0.5f : value - 0.5f);
  if (rounded > 32767) {
    rounded = 32767;
  }
  if (rounded < -32768) {
    rounded = -32768;
  }
  return static_cast<short>(rounded);
}

void boids_step_model_batch(const std::uint32_t* state_mem,
                            const std::uint32_t* vel_mem,
                            std::uint32_t* next_state_mem,
                            const std::size_t elements) {
  const float protected_range2 = kProtectedRange * kProtectedRange;
  const float visual_range2 = kVisualRange * kVisualRange;

  for (std::size_t i = 0; i < elements; ++i) {
    float x = static_cast<float>(unpack_hi(state_mem[i]));
    float y = static_cast<float>(unpack_lo(state_mem[i]));
    float vx = static_cast<float>(unpack_hi(vel_mem[i]));
    float vy = static_cast<float>(unpack_lo(vel_mem[i]));

    float close_dx = 0.0f;
    float close_dy = 0.0f;
    float xpos_avg = 0.0f;
    float ypos_avg = 0.0f;
    float xvel_avg = 0.0f;
    float yvel_avg = 0.0f;
    std::size_t neighbors = 0;

    for (std::size_t j = 0; j < elements; ++j) {
      if (j == i) {
        continue;
      }
      const float ox = static_cast<float>(unpack_hi(state_mem[j]));
      const float oy = static_cast<float>(unpack_lo(state_mem[j]));
      const float ovx = static_cast<float>(unpack_hi(vel_mem[j]));
      const float ovy = static_cast<float>(unpack_lo(vel_mem[j]));

      const float dx = x - ox;
      const float dy = y - oy;
      if (std::fabs(dx) < kVisualRange && std::fabs(dy) < kVisualRange) {
        const float dist2 = dx * dx + dy * dy;
        if (dist2 < protected_range2) {
          close_dx += dx;
          close_dy += dy;
        } else if (dist2 < visual_range2) {
          xpos_avg += ox;
          ypos_avg += oy;
          xvel_avg += ovx;
          yvel_avg += ovy;
          neighbors += 1;
        }
      }
    }

    if (neighbors > 0) {
      const float inv_neighbors = 1.0f / static_cast<float>(neighbors);
      xpos_avg *= inv_neighbors;
      ypos_avg *= inv_neighbors;
      xvel_avg *= inv_neighbors;
      yvel_avg *= inv_neighbors;

      vx += (xpos_avg - x) * kCenteringFactor + (xvel_avg - vx) * kMatchingFactor;
      vy += (ypos_avg - y) * kCenteringFactor + (yvel_avg - vy) * kMatchingFactor;
    }

    vx += close_dx * kAvoidFactor;
    vy += close_dy * kAvoidFactor;

    if (x < kHitLeft) {
      vx += kTurnFactor;
    } else if (x > kHitRight) {
      vx -= kTurnFactor;
    }
    if (y < kHitTop) {
      vy += kTurnFactor;
    } else if (y > kHitBottom) {
      vy -= kTurnFactor;
    }

    const float speed = std::sqrt((vx * vx) + (vy * vy));
    if (speed > 1.0e-5f) {
      if (speed < kMinSpeed) {
        const float scale = kMinSpeed / speed;
        vx *= scale;
        vy *= scale;
      } else if (speed > kMaxSpeed) {
        const float scale = kMaxSpeed / speed;
        vx *= scale;
        vy *= scale;
      }
    }

    x += vx;
    y += vy;

    if (x < kWorldMinX) {
      x = x + 15.0f;
      vx = -vx;
    } else if (x > kWorldMaxX) {
      x = x - 15.0f;
      vx = -vx;
    }

    if (y < kWorldMinY) {
      y = y + 15.0f;
      vy = -vy;
    } else if (y > kWorldMaxY) {
      y = y - 15.0f;
      vy = -vy;
    }

    x = std::min(std::max(x, kWorldMinX), kWorldMaxX);
    y = std::min(std::max(y, kWorldMinY), kWorldMaxY);

    next_state_mem[i] = pack_xy(clamp_to_i16(x), clamp_to_i16(y));
  }
}

void fill_inputs(std::uint32_t* a,
                 std::uint32_t* b,
                 const std::size_t elements,
                 const std::uint32_t seed,
                 const std::size_t iteration) {
  std::mt19937 rng(seed + static_cast<std::uint32_t>(iteration));
  std::uniform_int_distribution<std::int16_t> pos_x_dist(100, 540);
  std::uniform_int_distribution<std::int16_t> pos_y_dist(100, 380);
  std::uniform_int_distribution<std::int16_t> vel_dist(-6, 6);

  for (std::size_t i = 0; i < elements; ++i) {
    const auto va = pack_xy(pos_x_dist(rng), pos_y_dist(rng));
    const auto vb = pack_xy(vel_dist(rng), vel_dist(rng));
    a[i] = va;
    b[i] = vb;
  }
}

bool verify_outputs(const std::vector<std::uint32_t>& expected,
                    const std::uint32_t* out,
                    const std::size_t elements) {
  for (std::size_t i = 0; i < elements; ++i) {
    if (out[i] != expected[i]) {
      return false;
    }
  }
  return true;
}

BenchmarkRow base_row(const BenchmarkConfig& cfg, const std::size_t iteration) {
  BenchmarkRow row;
  row.timestamp_utc = utc_now_iso8601();
  row.mode = run_mode_string(cfg.mode);
  row.kernel = cfg.kernel_name;
  row.device_index = cfg.device_index;
  row.elements = cfg.elements;
  row.iteration = iteration;
  row.verify_ok = false;
  return row;
}

std::vector<BenchmarkRow> run_dry_run(const BenchmarkConfig& cfg) {
  std::vector<BenchmarkRow> rows;
  rows.reserve(cfg.iterations);

  std::vector<std::uint32_t> host_a(cfg.elements);
  std::vector<std::uint32_t> host_b(cfg.elements);
  std::vector<std::uint32_t> expected(cfg.elements);
  std::vector<std::uint32_t> dev_a(cfg.elements);
  std::vector<std::uint32_t> dev_b(cfg.elements);
  std::vector<std::uint32_t> dev_out(cfg.elements);
  std::vector<std::uint32_t> host_out(cfg.elements);

  for (std::size_t iteration = 0; iteration < cfg.iterations; ++iteration) {
    auto row = base_row(cfg, iteration);
    fill_inputs(host_a.data(), host_b.data(), cfg.elements, cfg.seed, iteration);
    boids_step_model_batch(host_a.data(), host_b.data(), expected.data(), cfg.elements);

    const auto t_total_start = Clock::now();

    const auto t_h2d_start = Clock::now();
    std::copy(host_a.begin(), host_a.end(), dev_a.begin());
    std::copy(host_b.begin(), host_b.end(), dev_b.begin());
    const auto t_h2d_end = Clock::now();

    const auto t_kernel_start = Clock::now();
    boids_step_model_batch(dev_a.data(), dev_b.data(), dev_out.data(), cfg.elements);
    const auto t_kernel_end = Clock::now();

    const auto t_d2h_start = Clock::now();
    std::copy(dev_out.begin(), dev_out.end(), host_out.begin());
    const auto t_d2h_end = Clock::now();

    const bool verify_ok = !cfg.verify || verify_outputs(expected, host_out.data(), cfg.elements);
    const auto t_total_end = Clock::now();

    row.h2d_ms = elapsed_ms(t_h2d_start, t_h2d_end);
    row.kernel_ms = elapsed_ms(t_kernel_start, t_kernel_end);
    row.d2h_ms = elapsed_ms(t_d2h_start, t_d2h_end);
    row.total_ms = elapsed_ms(t_total_start, t_total_end);
    row.verify_ok = verify_ok;
    row.throughput_gbps = throughput_gbps(cfg.elements, row.total_ms);
    rows.push_back(row);
  }

  return rows;
}

#if defined(U55C_ENABLE_XRT)
std::vector<BenchmarkRow> run_hardware(const BenchmarkConfig& cfg) {
  if (cfg.xclbin_path.empty()) {
    throw std::runtime_error("hardware mode requires --xclbin <path>");
  }

  if (cfg.elements > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    throw std::runtime_error("elements exceeds kernel arg width (uint32)");
  }

  std::vector<BenchmarkRow> rows;
  rows.reserve(cfg.iterations);

  const std::size_t bytes = cfg.elements * sizeof(std::uint32_t);

  xrt::device device(cfg.device_index);
  auto uuid = device.load_xclbin(cfg.xclbin_path);
  xrt::kernel kernel(device, uuid, cfg.kernel_name);
  BufferManager buffers(device, kernel, bytes);

  std::vector<std::uint32_t> expected(cfg.elements);

  for (std::size_t iteration = 0; iteration < cfg.iterations; ++iteration) {
    auto row = base_row(cfg, iteration);

    fill_inputs(buffers.input_a(), buffers.input_b(), cfg.elements, cfg.seed, iteration);
    boids_step_model_batch(buffers.input_a(), buffers.input_b(), expected.data(), cfg.elements);

    const auto t_total_start = Clock::now();

    const auto t_h2d_start = Clock::now();
    buffers.sync_inputs_to_device();
    const auto t_h2d_end = Clock::now();

    const auto t_kernel_start = Clock::now();
    auto run = kernel(
        buffers.input_a_bo(),
        buffers.input_b_bo(),
        buffers.output_bo(),
        static_cast<std::uint32_t>(cfg.elements));
    run.wait();
    const auto t_kernel_end = Clock::now();

    const auto t_d2h_start = Clock::now();
    buffers.sync_output_from_device();
    const auto t_d2h_end = Clock::now();

    const bool verify_ok = !cfg.verify || verify_outputs(expected, buffers.output(), cfg.elements);
    const auto t_total_end = Clock::now();

    row.h2d_ms = elapsed_ms(t_h2d_start, t_h2d_end);
    row.kernel_ms = elapsed_ms(t_kernel_start, t_kernel_end);
    row.d2h_ms = elapsed_ms(t_d2h_start, t_d2h_end);
    row.total_ms = elapsed_ms(t_total_start, t_total_end);
    row.verify_ok = verify_ok;
    row.throughput_gbps = throughput_gbps(cfg.elements, row.total_ms);
    rows.push_back(row);
  }

  return rows;
}
#endif

}  // namespace

const char* run_mode_string(const RunMode mode) {
  switch (mode) {
    case RunMode::DryRun:
      return "dry-run";
    case RunMode::Hardware:
      return "hw";
    default:
      return "unknown";
  }
}

std::vector<BenchmarkRow> run_benchmark(const BenchmarkConfig& cfg) {
  if (cfg.elements == 0) {
    throw std::runtime_error("elements must be > 0");
  }
  if (cfg.iterations == 0) {
    throw std::runtime_error("iterations must be > 0");
  }

  switch (cfg.mode) {
    case RunMode::DryRun:
      return run_dry_run(cfg);
    case RunMode::Hardware:
#if defined(U55C_ENABLE_XRT)
      return run_hardware(cfg);
#else
      throw std::runtime_error(
          "hardware mode requested, but this binary was built without XRT support");
#endif
    default:
      throw std::runtime_error("unsupported run mode");
  }
}

}  // namespace u55c_framework
