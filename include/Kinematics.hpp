#pragma once
#include "Math.hpp"

namespace sas
{
    struct Kinematics
    {
        float inverseMass;
        //0 = perfect innelastic
        //1 = perfect ellastic
        //>1 = explosive
        float restituition;
        math::Vec2 velocity;
        math::Vec2 acceleration;
    };
} // namespace sas
