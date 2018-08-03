
#pragma once

#ifndef M_PI
#define M_PI acos(-1.0)
#endif

// External or internal return values
#define SUCCESS 0
#define ERROR_GAUSSIAN_FIT 1
#define ERROR_PARAMETER_VARIATION 2
#define ERROR_CHI2_TOL_NOT_ACHIEVED 3
#define ERROR_WEAK_IMAGE 4
#define ERROR_SATURATED_IMAGE 5

// Internal return values
#define ERROR_UNDETERMINED_VALUE 6
#define ERROR_INFINITE_VALUE 7

typedef unsigned char intensity;
