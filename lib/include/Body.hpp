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
            RigidBodyStatic = 1 << 1,
            RigidBodyDynamic = 1 << 2,
            RigidBodyKinematic = 1 << 3,
            InCollisionPool = 1 << 4,
            Trigger = 1 << 5
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
            return {.type = ShapeType::Circle, .radius = r};
        }

        static Shape MakeBox(float x, float y)
        {
            return {.type = ShapeType::Box, .halfSize = {x / 2, y / 2}};
        }
    };

    struct Body
    {
        Transform2D transform;
        Kinematics kinematics;

        Shape shape;

        uint32_t bodyID;
        uint32_t flags;
        uint32_t collisionMask;

        float gravityScale = 1.f;

        uint32_t dataType = 0;
        void *userData = nullptr;


        [[nodiscard]] constexpr bool hasFlag(uint32_t flag) const noexcept
        {
            return (flags & flag) != 0;
        }
    };

} // namespace sas