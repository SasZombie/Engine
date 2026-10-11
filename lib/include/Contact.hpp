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

    struct ContactPoint3D
    {
        math::Vec3 position;
        float depth;
    };

    struct Contact3D
    {
        uint32_t bodyA;
        uint32_t bodyB;

        math::Vec3 normal;
        ContactPoint3D points[4];
        uint32_t pointCount = 0;
        float overlap;
    };

} // namespace sas
