#pragma once
#include <limits>
#include <cmath>

//TODO: If a lot of time passes without any other functions here
//Move this to Math module
bool floatAlmostEqual(float a, float b, float epsilon = std::numeric_limits<float>::epsilon()) {
    return std::abs(a - b) < epsilon;
}