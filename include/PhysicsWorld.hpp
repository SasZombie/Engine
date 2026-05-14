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
        void drawDebug(const DrawCallback &cb) const noexcept;

        BodyHandle createBody(Shape shape, const Transform &trans, uint32_t options = Flags::Active | Flags::RigidBodyDynamic) noexcept;
        BodyHandle createBody(Shape shape, const Transform &trans, const Kinematics &kin, uint32_t options = Flags::Active | Flags::RigidBodyDynamic) noexcept;


        void addToCollisionPool(Body &body) noexcept;
        void removeFromCollisionPool(Body &body) noexcept;
        void step(float dt) noexcept;

        [[nodiscard]] bool bodyExists(uint32_t id) const noexcept;
        [[nodiscard]] bool isBodyInCollision(uint32_t id) const noexcept;
        [[nodiscard]] Body &getBody(uint32_t id) noexcept;
        [[nodiscard]] std::vector<CollisionInfo> getAllCollisions(uint32_t id) noexcept;

        void removeBody(const BodyHandle &handle) noexcept;
        void removeBody(uint32_t bodyID) noexcept;

        void clear() noexcept;

        PhysicsWorld() noexcept = default;
        ~PhysicsWorld() noexcept = default;
        
    private:
        void applyForces(Body &obj) const noexcept;

        void integrate(Body &obj, float dt) const noexcept;

        void checkCollisionCircleCircle(Body &obj, Body &other) noexcept;
        void checkCollisionDispatcher(Body &obj) noexcept;
        void checkCollisionBoxBox(Body &obj, Body &other) noexcept;
        void checkCollisionCircleBox(Body &obj, Body &other) noexcept;
        void checkCollisionBoxCircle(Body &obj, Body &other) noexcept;

        void resolveColision(Body &obj, Body &other, math::Vec2 normal, float overlap, const std::pair<math::Vec2, math::Vec2>& rotComp) noexcept;
        void updateCollisionFlags() noexcept;

        void reset(Body &obj) const noexcept;

        [[nodiscard]] uint32_t getNextId() noexcept;

        BodyHandle createBodyFull(Shape shape, const Transform &trans, const Kinematics &kin, uint32_t options) noexcept;

        using CollisionFunc = void (PhysicsWorld::*)(Body &, Body &);
        static inline const CollisionFunc DispatchTable[2][2] = {
            {&sas::PhysicsWorld::checkCollisionCircleCircle, &sas::PhysicsWorld::checkCollisionCircleBox},
            {&sas::PhysicsWorld::checkCollisionBoxCircle,    &sas::PhysicsWorld::checkCollisionBoxBox}
        };
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
            return &world->getBody(id);
        }

        bool isValid() const noexcept
        {
            return world->bodyExists(id);
        }

        Body *get() const
        {
            return &world->getBody(id);
        }

        [[nodiscard]] bool isColliding() const noexcept
        {
            return world->isBodyInCollision(id);
        }

        [[nodiscard]] std::vector<CollisionInfo> getCollisions() const noexcept
        {
            return world->getAllCollisions(id);
        }

        void setActive() noexcept
        {
            auto &b = world->getBody(id);

            if (!(b.flags & Flags::Active))
            {
                b.flags |= Flags::Active;

                setCollisionOn();
            }
        }

        void setInactive() noexcept
        {
            auto &b = world->getBody(id);

            if (b.flags & Flags::Active)
            {
                b.flags &= ~Flags::Active;
                setCollisionOff();
            }
        }

        void setRigidBodyOn() noexcept
        {
            auto &b = world->getBody(id);

            if (!(b.flags & Flags::RigidBodyDynamic))
            {
                b.flags |= Flags::RigidBodyDynamic;
            }
        }

        void setRigidBodyOff() noexcept
        {
            auto &b = world->getBody(id);

            if (!(b.flags & Flags::RigidBodyDynamic))
            {
                b.flags |= Flags::RigidBodyDynamic;
            }
        }

        void setCollisionOff() noexcept
        {
            auto &b = world->getBody(id);
            b.collisionMask = 0;
            world->removeFromCollisionPool(b);
        }

        void setCollisionOn() noexcept
        {
            auto &b = world->getBody(id);
            world->addToCollisionPool(b);
        }

        void setMask(uint32_t mask) noexcept
        {
            auto &b = world->getBody(id);

            b.collisionMask = (b.collisionMask & 0x0000FFFF) | (mask & 0xFFFF0000);
        }

        void setLayer(uint32_t layerBits) noexcept
        {
            auto &b = world->getBody(id);
            b.collisionMask = (b.collisionMask & 0xFFFF0000) | (layerBits & 0x0000FFFF);
        }

        void setCollision(uint32_t layer, uint32_t mask) noexcept
        {
            setMask(mask);
            setLayer(layer);
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
