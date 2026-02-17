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

    class PhysicsWorld
    {
    public:
        PhysicsSettings settings;

        void addToCollisionPool(const Body& body) noexcept;
        void Step(std::vector<Body> &objects, float dt) const noexcept;

        PhysicsWorld(Rectangle dims) noexcept;
        ~PhysicsWorld() noexcept = default;

    private:
        Rectangle boundaries;
        AABBTree root;
        void ApplyForces(Body &obj) const noexcept;

        void Integrate(Body &obj, float dt) const noexcept;

        void ResolveConstraints(Body &obj, float dt) const noexcept;
        void Reset(Body &obj) const noexcept;

        void ResolveBroadLower(Body &obj, float wall) const noexcept;
        void ResolveBroadHigher(Body &obj, float wall) const noexcept;
    };

} // namespace sas
