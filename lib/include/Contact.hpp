#pragma once
#include "Math.hpp"
#include <cstdint>

namespace sas
{
    struct ContactPoint2D
    {
        math::Vec2 position;
        float depth;
    };

    struct Contact2D
    {
        uint32_t bodyA;
        uint32_t bodyB;

        math::Vec2 normal;
        ContactPoint2D points[2];
        uint32_t pointCount = 0;
        float overlap;
    };

} // namespace sas
