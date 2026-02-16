#pragma once

#include "Transform.hpp"
#include "Kinematics.hpp"

namespace sas
{

    enum struct ShapeType
    {
        Circle,
        Box
    };

    struct Shape
    {
        ShapeType type;

        union
        {
            float radius;
            math::Vec2 halfSizes;
        };
    };

    struct Body
    {
        Shape shape;
        Transform transform;
        Kinematics kinematics;
    };
    
} // namespace sas