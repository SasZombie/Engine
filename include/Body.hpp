#pragma once
#include <cstdint>

#include "Transform.hpp"
#include "Kinematics.hpp"

namespace sas
{

    namespace Flags
    {

        enum BodyFlags : uint32_t
        {
            None = 0,
            Active = 1 << 0,
            Static = 1 << 1,
            RigidBody = 1 << 2,
            InCollisionPool = 1 << 3,
            // Trigger     = 1 << 3
        };

        enum CollisionFlags : uint32_t
        {
            // Who am I?
            LayerNone = 0,
            LayerAll = 0x0000FFFF,

            Layer1 = 1 << 0,
            Layer2 = 1 << 1,

            // Who did I hit?
            MaskNone = 0,
            MaskAll = 0xFFFF0000,

            Mask1 = 1 << 16,
            Mask2 = 1 << 17,
        };
    } // namespace Flags

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
            math::Vec2 halfSize;
        };

        static Shape MakeCircle(float r)
        {
            return {.type = ShapeType::Box, .radius = r};
        }

        static Shape MakeBox(float x, float y)
        {
            return {.type = ShapeType::Box, .halfSize = {x, y}};
        }
    };

    struct Body
    {
        Transform transform;
        Kinematics kinematics;

        Shape shape;

        uint32_t bodyID;
        uint32_t flags;
        uint32_t collisionMask;
    };

} // namespace sas