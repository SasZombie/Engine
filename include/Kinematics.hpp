#pragma once
#include "Math.hpp"

namespace sas
{
    struct Kinematics
    {
        float inverseMass = 0;
        //0 = perfect innelastic
        //1 = perfect ellastic
        //>1 = explosive
        float restituition = 0;
        math::Vec2 velocity{0};
        math::Vec2 acceleration{0};
        float angularVelocity = 0;
        float inertia = 0;
        float inverseInertia = 0;
    };
} // namespace sas
