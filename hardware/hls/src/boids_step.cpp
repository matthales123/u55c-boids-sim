#include "boids_step.hpp"

#include <hls_math.h>

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

inline short unpack_hi(const ap_uint<32> value) {
  return static_cast<short>((value >> 16) & 0xFFFF);
}

inline short unpack_lo(const ap_uint<32> value) {
  return static_cast<short>(value & 0xFFFF);
}

inline ap_uint<32> pack_xy(const short x, const short y) {
  const ap_uint<32> hi = static_cast<unsigned short>(x);
  const ap_uint<32> lo = static_cast<unsigned short>(y);
  return (hi << 16) | lo;
}

inline short clamp_to_i16(const float value) {
  int rounded = static_cast<int>(value >= 0.0f ? value + 0.5f : value - 0.5f);
  if (rounded > 32767) {
    rounded = 32767;
  }
  if (rounded < -32768) {
    rounded = -32768;
  }
  return static_cast<short>(rounded);
}

}  // namespace

extern "C" {
void boids_step(const ap_uint<32>* state_mem,
               const ap_uint<32>* vel_mem,
               ap_uint<32>* next_state_mem,
               const unsigned int n) {
#pragma HLS INTERFACE m_axi port = state_mem offset = slave bundle = gmem0 max_read_burst_length = 64 num_read_outstanding = 16
#pragma HLS INTERFACE m_axi port = vel_mem offset = slave bundle = gmem1 max_read_burst_length = 64 num_read_outstanding = 16
#pragma HLS INTERFACE m_axi port = next_state_mem offset = slave bundle = gmem2 max_write_burst_length = 64 num_write_outstanding = 16
#pragma HLS INTERFACE s_axilite port = state_mem bundle = control
#pragma HLS INTERFACE s_axilite port = vel_mem bundle = control
#pragma HLS INTERFACE s_axilite port = next_state_mem bundle = control
#pragma HLS INTERFACE s_axilite port = n bundle = control
#pragma HLS INTERFACE s_axilite port = return bundle = control

  const float protected_range2 = kProtectedRange * kProtectedRange;
  const float visual_range2 = kVisualRange * kVisualRange;

  for (unsigned int i = 0; i < n; ++i) {
    const ap_uint<32> self_state = state_mem[i];
    const ap_uint<32> self_vel = vel_mem[i];

    float x = static_cast<float>(unpack_hi(self_state));
    float y = static_cast<float>(unpack_lo(self_state));
    float vx = static_cast<float>(unpack_hi(self_vel));
    float vy = static_cast<float>(unpack_lo(self_vel));

    float close_dx = 0.0f;
    float close_dy = 0.0f;
    float xpos_avg = 0.0f;
    float ypos_avg = 0.0f;
    float xvel_avg = 0.0f;
    float yvel_avg = 0.0f;
    unsigned int neighbors = 0;

    for (unsigned int j = 0; j < n; ++j) {
#pragma HLS PIPELINE II = 1
      if (j == i) {
        continue;
      }

      const ap_uint<32> other_state = state_mem[j];
      const ap_uint<32> other_vel = vel_mem[j];

      const float ox = static_cast<float>(unpack_hi(other_state));
      const float oy = static_cast<float>(unpack_lo(other_state));
      const float ovx = static_cast<float>(unpack_hi(other_vel));
      const float ovy = static_cast<float>(unpack_lo(other_vel));

      const float dx = x - ox;
      const float dy = y - oy;
      if (hls::fabsf(dx) < kVisualRange && hls::fabsf(dy) < kVisualRange) {
        const float dist2 = dx * dx + dy * dy;
        if (dist2 < protected_range2) {
          // Move away from close neighbors.
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

    const float speed = hls::sqrtf(vx * vx + vy * vy);
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

    if (x < kWorldMinX) {
      x = kWorldMinX;
    } else if (x > kWorldMaxX) {
      x = kWorldMaxX;
    }
    if (y < kWorldMinY) {
      y = kWorldMinY;
    } else if (y > kWorldMaxY) {
      y = kWorldMaxY;
    }

    next_state_mem[i] = pack_xy(clamp_to_i16(x), clamp_to_i16(y));
  }
}
}
