#pragma once
#include <math.h>

/** @brief Normalize angle to [-π, π] in place */
static inline void normalize_angle(float *a)
{
    while (*a > (float)M_PI)  *a -= 2.0f * (float)M_PI;
    while (*a < -(float)M_PI) *a += 2.0f * (float)M_PI;
}
