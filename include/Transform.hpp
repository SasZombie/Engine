#pragma once
#include "Math.hpp"

namespace sas
{

    struct Transform
    {
        float rotation;
        math::Vec2 position;
        math::Vec2 scale{1.f, 1.f};
    };
    
}