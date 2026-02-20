#pragma once
#include <cstdint>

#include "Transform.hpp"
#include "Kinematics.hpp"

namespace sas
{

    namespace Filter
    {

        enum BodyFlags : uint32_t
        {
            Active = 1 << 0,
            Static = 1 << 1
        };

        enum CollisionFlags : uint32_t
        {
            //Who am I?
            LayerNone = 0,
            LayerAll = 0x0000FFFF, 

            Layer1 = 1 << 0,
            Layer2 = 1 << 1,

            //Who did I hit?
            MaskNone = 0,
            MaskAll = 0xFFFF0000,

            Mask1 = 1 << 16,
            Mask2 = 1 << 17,
        };
    } // namespace Filter

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
        uint32_t flags;
        uint32_t filter;
        
    };

} // namespace sas