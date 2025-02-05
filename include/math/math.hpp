#pragma once

#include <cmath>

#define BIG_NUMBER 999999999.0

/**
 * @brief Return the sign of a number
 *
 * @param val The number to check
 * @return -1 for negative, 0 for zero, and 1 for positive
 */
int sgn(double val);

double normalize(const double value, const double start, const double end);
