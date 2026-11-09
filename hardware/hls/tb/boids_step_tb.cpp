#include "../src/boids_step.hpp"

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <vector>

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

short unpack_hi(const ap_uint<32> value) {
  return static_cast<short>((value >> 16) & 0xFFFF);
}

short unpack_lo(const ap_uint<32> value) {
  return static_cast<short>(value & 0xFFFF);
}

ap_uint<32> pack_xy(const short x, const short y) {
  const ap_uint<32> hi = static_cast<unsigned short>(x);
  const ap_uint<32> lo = static_cast<unsigned short>(y);
  return (hi << 16) | lo;
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

void model_step_batch(const std::vector<ap_uint<32>>& state_mem,
                      const std::vector<ap_uint<32>>& vel_mem,
                      std::vector<ap_uint<32>>& expected) {
  const unsigned int n = static_cast<unsigned int>(state_mem.size());
  const float protected_range2 = kProtectedRange * kProtectedRange;
  const float visual_range2 = kVisualRange * kVisualRange;

  for (unsigned int i = 0; i < n; ++i) {
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
    unsigned int neighbors = 0;

    for (unsigned int j = 0; j < n; ++j) {
      if (j == i) continue;
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
          neighbors++;
        }
      }
    }

    if (neighbors > 0) {
      const float inv = 1.0f / static_cast<float>(neighbors);
      xpos_avg *= inv;
      ypos_avg *= inv;
      xvel_avg *= inv;
      yvel_avg *= inv;
      vx += (xpos_avg - x) * kCenteringFactor + (xvel_avg - vx) * kMatchingFactor;
      vy += (ypos_avg - y) * kCenteringFactor + (yvel_avg - vy) * kMatchingFactor;
    }

    vx += close_dx * kAvoidFactor;
    vy += close_dy * kAvoidFactor;

    if (x < kHitLeft) vx += kTurnFactor;
    else if (x > kHitRight) vx -= kTurnFactor;
    if (y < kHitTop) vy += kTurnFactor;
    else if (y > kHitBottom) vy -= kTurnFactor;

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
    if (x < kWorldMinX) x = kWorldMinX;
    else if (x > kWorldMaxX) x = kWorldMaxX;
    if (y < kWorldMinY) y = kWorldMinY;
    else if (y > kWorldMaxY) y = kWorldMaxY;

    expected[i] = pack_xy(clamp_to_i16(x), clamp_to_i16(y));
  }
}

}  // namespace

int main() {
  constexpr unsigned int n = 128;
  std::vector<ap_uint<32>> state_mem(n);
  std::vector<ap_uint<32>> vel_mem(n);
  std::vector<ap_uint<32>> next_state_mem(n);
  std::vector<ap_uint<32>> expected(n);

  for (unsigned int i = 0; i < n; ++i) {
    const short x = static_cast<short>((static_cast<int>(i * 5) % 640));
    const short y = static_cast<short>((static_cast<int>(i * 7) % 480));
    const short vx = static_cast<short>((static_cast<int>(i) % 9) - 4);
    const short vy = static_cast<short>((static_cast<int>(i * 3) % 9) - 4);
    state_mem[i] = pack_xy(x, y);
    vel_mem[i] = pack_xy(vx, vy);
  }

  model_step_batch(state_mem, vel_mem, expected);
  boids_step(state_mem.data(), vel_mem.data(), next_state_mem.data(), n);

  for (unsigned int i = 0; i < n; ++i) {
    if (next_state_mem[i] != expected[i]) {
      std::cerr << "Mismatch at i=" << i << ": got " << next_state_mem[i]
                << " expected " << expected[i] << '\n';
      return EXIT_FAILURE;
    }
  }

  std::cout << "boids_step_tb PASS\n";
  return EXIT_SUCCESS;
}
