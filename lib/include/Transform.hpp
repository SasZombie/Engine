#pragma once
#include "Math.hpp"

namespace sas
{

    struct Transform2D
    {
        float rotation = 0;
        math::Vec2 position;
        math::Vec2 scale{1.f, 1.f};

        Transform2D() noexcept = default;
        explicit Transform2D(const math::Vec2& pos) noexcept
            : position(pos)
        {

        }

    };

    struct Transform3D
    {
        float rotation = 0;
        math::Vec3 position;
        math::Vec3 scale{1.f, 1.f, 1.f};

        Transform3D() noexcept = default;
        explicit Transform3D(const math::Vec3& pos) noexcept
            : position(pos)
        {

        }
    };
    
}