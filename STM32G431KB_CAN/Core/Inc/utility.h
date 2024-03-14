#ifndef __UTILITY_H
#define __UTILITY_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "main.h"

typedef struct {
  float32_t   time_constant;	// Parameter: time constant
  float32_t   sampling_time;	// Parameter: sampling time
  float32_t   input;			// Input: 
  float32_t   output_shadow;	// History: Previous output
  float32_t   output;			// Output: filtered output
} LowPassFilter;

void filter_compute(LowPassFilter *h);
	
int16_t sign(float32_t _val);
float64_t rad2deg(float64_t _val);
float64_t deg2rad(float64_t _val);
float64_t norm_f64(float64_t v[], uint16_t _size);

#ifdef __cplusplus
}
#endif

#endif /* __UTILITY_H */