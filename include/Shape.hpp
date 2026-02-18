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
        Transform transform;
        Kinematics kinematics;
        
        Shape shape;
        uint32_t bodyID;
        bool isColliding = false;
    };
    
} // namespace sas