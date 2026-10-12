#pragma once
#include "Math.hpp"

namespace sas
{
    struct Kinematics2D
    {
        float inverseMass = 0;
        // 0 = perfect innelastic
        // 1 = perfect ellastic
        //>1 = explosive
        float restitution = 0;
        math::Vec2 velocity;
        math::Vec2 acceleration;
        float angularVelocity = 0;
        float inertia = 0;
        float inverseInertia = 0;
        float pushForce = 0;

        void reset() noexcept
        {
            inverseMass = 0;
            velocity = math::Vec2{0};
            acceleration = math::Vec2{0};
            angularVelocity = 0;
            inertia = 0;
            inverseInertia = 0;
            pushForce = 0;
        }
    };

    struct Kinematics3D
    {
        float inverseMass = 0.0f;

        // 0 = perfect innelastic
        // 1 = perfect ellastic
        //>1 = explosive
        float restitution = 0.0f;
        math::Vec3 velocity{0.0f};
        math::Vec3 pushForce{0.0f};
        math::Vec3 acceleration{0.0f};

        math::Vec3 angularVelocity{0.0f};
        math::Vec3 torque{0.0f};
        math::Mat3 inverseInertiaTensorLocal{0.0f};
        math::Mat3 inverseInertiaTensorWorld{0.0f};

        void reset() noexcept
        {
            inverseMass = 0.0f;
            velocity = math::Vec3{0.0f};
            pushForce = math::Vec3{0.0f};

            angularVelocity = math::Vec3{0.0f};
            torque = math::Vec3{0.0f};

            inverseInertiaTensorLocal = math::Mat3{0.0f};
            inverseInertiaTensorWorld = math::Mat3{0.0f};
            restitution = 0.0f;
        }
    };
} // namespace sas
