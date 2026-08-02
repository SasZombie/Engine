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

    struct ContactPoint
    {
        math::Vec2 position;
        float depth;
    };

    struct Contact
    {
        uint32_t bodyA;
        uint32_t bodyB;

        math::Vec2 normal;
        ContactPoint points[2];
        uint32_t pointCount = 0;
        float overlap;
    };

    struct BodyHandle;

    class PhysicsWorld
    {
    public:
        PhysicsSettings settings;
        uint32_t idCounter = 0;

    private:
        float deltaTime;
        float timeAccumulator = 0;
        AABBTree root;

    public:
        // Cashe locality
        // World keeps body
        std::vector<Body> bodies;

        std::vector<int> collisionFlags;
        // Found this funny ahh pattern
        std::vector<int> sparse;
        std::vector<uint32_t> dense;
        std::vector<uint32_t> freeIDs;
        std::vector<uint32_t> activeIDs;

        std::vector<Contact> contacts;
        std::vector<Contact> frameContacts;

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

        [[nodiscard]] Body &getBodyUnsafe(uint32_t id) noexcept;
        [[nodiscard]] const Body &getBodyUnsafe(uint32_t id) const noexcept;

        [[nodiscard]] Body* getBody(uint32_t id) noexcept;
        [[nodiscard]] const Body* getBody(uint32_t id) const noexcept;

        [[nodiscard]] std::vector<CollisionInfo> getAllCollisions(uint32_t id) noexcept;

        void removeBody(const BodyHandle &handle) noexcept;
        void removeBody(uint32_t bodyID) noexcept;

        void clear() noexcept;

        PhysicsWorld() noexcept = default;
        ~PhysicsWorld() noexcept = default;
        
    private:
        void step() noexcept;

        void applyForces(Body &obj) const noexcept;

        void integrateVelocity(Body &obj) const noexcept;
        void integratePosition(Body &obj) const noexcept;

        void checkCollisionCircleCircle(Body &obj, Body &other) noexcept;
        void checkCollisionDispatcher(Body &obj) noexcept;
        void checkCollisionBoxBox(Body &obj, Body &other) noexcept;
        void checkCollisionCircleBox(Body &obj, Body &other) noexcept;
        void checkCollisionBoxCircle(Body &obj, Body &other) noexcept;

        void resolveCollision(Contact& contact) noexcept;
        void correctPosition(Contact& contact) noexcept;
        void updateCollisionFlags() noexcept;

        void reset(Body &obj) const noexcept;

        //This is unchecked
        [[nodiscard]] const Body& getBodyFromSparse(uint32_t id) const noexcept;
        //This is unchecked
        [[nodiscard]] Body& getBodyFromSparse(uint32_t id) noexcept;

        [[nodiscard]] uint32_t getNextId() noexcept;

        BodyHandle createBodyFull(Shape shape, const Transform &trans, const Kinematics &kin, uint32_t options) noexcept;

        void initializeBodyPhysics(Body& body) noexcept;
        void setupCollision(Body& body) noexcept;
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
            return &world->getBodyUnsafe(id);
        }

        bool isValid() const noexcept
        {
            return world->bodyExists(id);
        }

        Body *get() const
        {
            return &world->getBodyUnsafe(id);
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
            auto &b = world->getBodyUnsafe(id);

            if (!(b.flags & Flags::Active))
            {
                b.flags |= Flags::Active;

                setCollisionOn();
            }
        }

        void setInactive() noexcept
        {
            auto &b = world->getBodyUnsafe(id);

            if (b.flags & Flags::Active)
            {
                b.flags &= ~Flags::Active;
                setCollisionOff();
            }
        }

        void setRigidBodyOn() noexcept
        {
            auto &b = world->getBodyUnsafe(id);

            if (!(b.flags & Flags::RigidBodyDynamic))
            {
                b.flags |= Flags::RigidBodyDynamic;
            }
        }

        void setRigidBodyOff() noexcept
        {
            auto &b = world->getBodyUnsafe(id);

            if (!(b.flags & Flags::RigidBodyDynamic))
            {
                b.flags |= Flags::RigidBodyDynamic;
            }
        }

        void setCollisionOff() noexcept
        {
            auto &b = world->getBodyUnsafe(id);
            b.collisionMask = 0;
            world->removeFromCollisionPool(b);
        }

        void setCollisionOn() noexcept
        {
            auto &b = world->getBodyUnsafe(id);
            world->addToCollisionPool(b);
        }

        void setMask(uint32_t mask) noexcept
        {
            auto &b = world->getBodyUnsafe(id);

            b.collisionMask = (b.collisionMask & 0x0000FFFF) | (mask & 0xFFFF0000);
        }

        void setLayer(uint32_t layerBits) noexcept
        {
            auto &b = world->getBodyUnsafe(id);
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
