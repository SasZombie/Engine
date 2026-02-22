#pragma once
#include <vector>

#include "AABBTree.hpp"
#include "Primitives.hpp"

namespace sas
{
    struct CollisionInfo;
    struct PhysicsSettings
    {
        float gravity = 500.f;
        float dragCoeff = 0.47f;
        float groundFriction = 0.98f;
        float wallFriction = 0.98f;
    };

    struct Contact
    {
        uint32_t bodyA;
        uint32_t bodyB;

        math::Vec2 normal;

        float depth;
    };

    struct BodyHandle;

    class PhysicsWorld
    {
    public:
        PhysicsSettings settings;
        uint32_t idCounter = 0;

    private:
        Rectangle boundaries;
        AABBTree root;

    public:
        // Cashe locality
        // World keeps body
        std::vector<Body> bodies;

        // Found this funny ahh pattern
        std::vector<int> sparse;
        std::vector<int> collisionFlags;
        std::vector<uint32_t> dense;
        std::vector<uint32_t> freeIDs;
        std::vector<uint32_t> activeIDs;

        std::vector<Contact> contacts;

        // Visualizing hitboxes
        // Not Optimized
        void DrawDebug(const DrawCallback &cb) const noexcept;

        BodyHandle CreateBody(Shape shape, const Transform &trans, uint32_t options = Flags::Active | Flags::RigidBody) noexcept;
        BodyHandle CreateBody(Shape shape, const Transform &trans, const Kinematics &kin, uint32_t options = Flags::Active | Flags::RigidBody) noexcept;

        void AddToCollisionPool(Body &body) noexcept;
        void RemoveFromCollisionPool(Body &body) noexcept;
        void Step(float dt) noexcept;

        [[nodiscard]] bool BodyExists(uint32_t id) const noexcept;
        [[nodiscard]] bool IsBodyInCollision(uint32_t id) const noexcept;
        [[nodiscard]] Body &GetBody(uint32_t id) noexcept;
        [[nodiscard]] std::vector<CollisionInfo> GetAllCollisions(uint32_t id) noexcept;

        void RemoveBody(const BodyHandle &handle) noexcept;
        void RemoveBody(uint32_t bodyID) noexcept;

        void Clear() noexcept;

        PhysicsWorld(Rectangle dims) noexcept;
        ~PhysicsWorld() noexcept = default;

    private:
        void ApplyForces(Body &obj) const noexcept;

        void Integrate(Body &obj, float dt) const noexcept;

        void ResolveConstraints(Body &obj, float dt) const noexcept;
        void CheckCollisionCircleCircle(Body &obj, Body& other) noexcept;
        void CheckCollisionDispatcher(Body &obj) noexcept;
        void CheckCollisionBoxBox(Body &obj, Body& other) noexcept;
        void CheckCollisionCircleBox(Body &obj, Body& other) noexcept;
        void CheckCollisionBoxCircle(Body &obj, Body& other) noexcept;

        void ResolveColision(Body& obj, Body& other, math::Vec2 normal, float overlap) noexcept;
        void UpdateCollisionFlags() noexcept;

        void Reset(Body &obj) const noexcept;

        [[nodiscard]] uint32_t GetNextId() noexcept;

        BodyHandle CreateBodyFull(Shape shape, const Transform &trans, const Kinematics &kin, uint32_t options) noexcept;

        void ResolveBroadLower(Body &obj, float wall) const noexcept;
        void ResolveBroadHigher(Body &obj, float wall) const noexcept;

        void ResolveBroadCeil(Body &obj, float wall) const noexcept;
        void ResolveBroadGround(Body &obj, float wall) const noexcept;
    };

    class BodyHandle
    {
    private:
        uint32_t id;
        PhysicsWorld *world;

    public:
        BodyHandle(uint32_t bodyID, PhysicsWorld *pworld)
            : id(bodyID), world(pworld)
        {
        }

        Body *operator->()
        {
            return &world->GetBody(id);
        }

        bool isValid() const noexcept
        {
            return world->BodyExists(id);
        }

        Body *get() const
        {
            return &world->GetBody(id);
        }

        [[nodiscard]] bool IsColliding() const noexcept
        {
            return world->IsBodyInCollision(id);
        }

        [[nodiscard]] std::vector<CollisionInfo> GetCollisions() const noexcept
        {
            return world->GetAllCollisions(id);
        }

        void SetActive() noexcept
        {
            auto &b = world->GetBody(id);

            if (!(b.flags & Flags::Active))
            {
                b.flags |= Flags::Active;

                SetCollisionOn();
            }
        }

        void SetInactive() noexcept
        {
            auto &b = world->GetBody(id);

            if (b.flags & Flags::Active)
            {
                b.flags &= ~Flags::Active;
                SetCollisionOff();
            }
        }

        void SetRigidBodyOn() noexcept
        {
            auto &b = world->GetBody(id);

            if (!(b.flags & Flags::RigidBody))
            {
                b.flags |= Flags::RigidBody;
            }
        }

        void SetRigidBodyOff() noexcept
        {
            auto &b = world->GetBody(id);

            if (!(b.flags & Flags::RigidBody))
            {
                b.flags |= Flags::RigidBody;
            }
        }

        void SetCollisionOff() noexcept
        {
            auto &b = world->GetBody(id);
            b.collisionMask = 0;
            world->RemoveFromCollisionPool(b);
        }

        void SetCollisionOn() noexcept
        {
            auto &b = world->GetBody(id);
            world->AddToCollisionPool(b);
        }

        void SetMask(uint32_t mask) noexcept
        {
            auto &b = world->GetBody(id);

            b.collisionMask = (b.collisionMask & 0x0000FFFF) | (mask & 0xFFFF0000);
        }

        void SetLayer(uint32_t layerBits) noexcept
        {
            auto &b = world->GetBody(id);
            b.collisionMask = (b.collisionMask & 0xFFFF0000) | (layerBits & 0x0000FFFF);
        }

        void SetCollision(uint32_t layer, uint32_t mask) noexcept
        {
            SetMask(mask);
            SetLayer(layer);
        }

        ~BodyHandle() = default;
    };

    struct CollisionInfo
    {
        BodyHandle other;

        math::Vec2 normal;

        float depth;
    };

} // namespace sas
