
#ifndef Z_MATH_H
#define Z_MATH_H

#ifdef __cplusplus
  extern "C" {
#endif

#include "Config.h"

/* typedefs */
typedef float TYPE_SCALE;
typedef float TYPE_MAX;

/* global functions */
extern void dsp_max_min_val(const TYPE_MAX* x, int nx, TYPE_MAX *max, TYPE_MAX *min);
extern void scale(TYPE_SCALE x[], 
           TYPE_SCALE xmax, 
           TYPE_SCALE xmin,
           int32_t n, 
           TYPE_SCALE s_low, 
           TYPE_SCALE s_high);
extern float my_cabs(COMPLEX x);
extern int ones_32(uint32_t n);  
extern uint32_t floor_log2_32(uint32_t x);

#ifdef __cplusplus
 }
#endif 

#endif