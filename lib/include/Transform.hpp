#pragma once
#include "Math.hpp"

namespace sas
{

    struct Transform
    {
        float rotation = 0;
        math::Vec2 position;
        math::Vec2 scale{1.f, 1.f};

        Transform() noexcept = default;
        explicit Transform(const math::Vec2& pos) noexcept
            : position(pos)
        {

        }

    };
    
}