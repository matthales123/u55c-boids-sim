#pragma once

#include <ap_int.h>

extern "C" {
void boids_step(const ap_uint<32>* state_mem,
               const ap_uint<32>* vel_mem,
               ap_uint<32>* next_state_mem,
               unsigned int n);
}
