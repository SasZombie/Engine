#pragma once
#include "Math.hpp"

namespace sas
{
    struct PhysicsSettings
    {
        math::Vec3 gravity;
        float dragCoeff = 0.47f;
        float groundFriction = 0.98f;
        float wallFriction = 0.98f;
        float physicsDt = 1.0f / 120.0f;
    };

} // namespace sas
