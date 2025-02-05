#include "math/math.hpp"

int sgn(double val) { return (0 < val) - (val < 0); }
double normalize(const double value, const double start, const double end)
{
    const double width = end - start;         //
    const double offsetValue = value - start; // value relative to 0

    return (offsetValue - (floor(offsetValue / width) * width)) + start;
    // + start to reset back to start of original range
}