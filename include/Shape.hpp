#pragma once
#include <cstdint>

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
        uint32_t bodyID;
        Shape shape;
        Transform transform;
        Kinematics kinematics;
    };
    
} // namespace sas