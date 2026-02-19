#pragma once
#include <vector>

#include "AABBTree.hpp"
#include "Primitives.hpp"

namespace sas
{
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
    class PhysicsWorld;
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

        //Found this funny ahh pattern
        std::vector<int> sparse;
        std::vector<uint32_t> dense;
        std::vector<uint32_t> freeIDs;

        std::vector<Contact> contacts;

        // Visualizing hitboxes
        // Not Optimized
        void DrawDebug(const DrawCallback &cb) const noexcept;

        BodyHandle CreateBody(Shape shape, const Transform &trans) noexcept;

        void addToCollisionPool(const Body &body) noexcept;
        void Step(float dt) noexcept;

        bool BodyExists(uint32_t id) const noexcept;
        Body &GetBody(uint32_t id) noexcept;

        void RemoveBody(const BodyHandle& handle) noexcept;
        void RemoveBody(uint32_t bodyID) noexcept;

        void Clear() noexcept;

        PhysicsWorld(Rectangle dims) noexcept;
        ~PhysicsWorld() noexcept = default;

    private:
        void ApplyForces(Body &obj) const noexcept;

        void Integrate(Body &obj, float dt) const noexcept;

        void ResolveConstraints(Body &obj, float dt) const noexcept;
        void CheckCollision(Body &obj) noexcept;

        void Reset(Body &obj) const noexcept;

        uint32_t GetNextId() noexcept;

        void ResolveBroadLower(Body &obj, float wall) const noexcept;
        void ResolveBroadHigher(Body &obj, float wall) const noexcept;

        void ResolveBroadCeil(Body &obj, float wall) const noexcept;
        void ResolveBroadGround(Body &obj, float wall) const noexcept;
    };

    struct BodyHandle
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

        ~BodyHandle() = default;
    };

} // namespace sas
