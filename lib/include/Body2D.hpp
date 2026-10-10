#pragma once
#include <cstdint>

#include <meta>
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

    enum struct ShapeType2D
    {
        Circle,
        Box,
        __count__
    };

    // constexpr auto ShapeType2DCount = std::meta::enumerators_of(^^ShapeType2D).size();

    struct Shape2D
    {
        ShapeType2D type;

        union
        {
            float radius;
            math::Vec2 halfSize;
        };

        static Shape2D MakeCircle(float r)
        {
            return {.type = ShapeType2D::Circle, .radius = r};
        }

        static Shape2D MakeBox(float x, float y)
        {
            return {.type = ShapeType2D::Box, .halfSize = {x / 2, y / 2}};
        }
    };

    struct Body2D
    {
        Transform2D transform;
        Kinematics2D kinematics;

        Shape2D shape;

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