#pragma once
#include "Math.hpp"

namespace sas
{
    struct Kinematics
    {
        float inverseMass = 0;
        // 0 = perfect innelastic
        // 1 = perfect ellastic
        //>1 = explosive
        float restituition = 0;
        math::Vec2 velocity;
        math::Vec2 acceleration;
        float angularVelocity = 0;
        float inertia = 0;
        float inverseInertia = 0;

        void reset() noexcept
        {
            inverseMass = 0;
            velocity = math::Vec2{0};
            acceleration = math::Vec2{0};
            angularVelocity = 0;
            inertia = 0;
            inverseInertia = 0;
        }
    };
} // namespace sas
