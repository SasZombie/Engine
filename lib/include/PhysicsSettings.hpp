#pragma once

namespace sas
{
    struct PhysicsSettings
    {
        float gravity = 500.f;
        float dragCoeff = 0.47f;
        float groundFriction = 0.98f;
        float wallFriction = 0.98f;
        float physicsDt = 1.0f / 120.0f;
    };

} // namespace sas
