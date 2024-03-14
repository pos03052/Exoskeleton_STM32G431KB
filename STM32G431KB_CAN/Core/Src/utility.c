#include "utility.h"

int16_t sign(float32_t _val)
{
	return (_val > 0 ? 1 : (_val < 0 ? -1 : 0));
}

float64_t rad2deg(float64_t _val)
{
	return (_val * (180.0 / PI));
}

float64_t deg2rad(float64_t _val)
{
	return (_val * (PI / 180.0));
}

float64_t norm_f64(float64_t v[], uint16_t _size)
{
	if (_size < 1) return -1;
	
	float64_t sum = 0;
	for (int i = 0; i < _size; i++) {
		sum += v[i] * v[i];
	}
	
	float32_t rtn = 0;
	arm_sqrt_f32((float32_t)sum, &rtn);
	if (rtn < 0.00001) rtn = 0.00001;
	
	return rtn;
}

void filter_compute(LowPassFilter *h)
{
	h->output = (h->time_constant * h->output_shadow + h->sampling_time * h->input) / (h->time_constant + h->sampling_time);
	h->output_shadow = h->output;   
}