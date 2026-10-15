#pragma once
#include <vector>

#include "Traits.hpp"
#include "AABBTree.hpp"
#include "Primitives.hpp"
#include "PhysicsSettings.hpp"

namespace sas
{
    enum struct DeviceOptions
    {
        CPU,
        GPU
    };

    template <DimensionalTraits Traits>
    struct CollisionInfo;

    template <DimensionalTraits Traits>
    struct BodyHandle;

    template <DimensionalTraits Traits>
    class PhysicsWorld
    {
    public:
        using Body = typename Traits::Body;
        using Shape = typename Traits::Shape;
        using Contact = typename Traits::Contact;
        using AABBTree = typename Traits::AABBTree;
        using Transform = typename Traits::Transform;
        using Kinematics = typename Traits::Kinematics;
        using DrawCallback = typename Traits::DrawCallback;

        PhysicsSettings settings;
        uint32_t idCounter = 0;
        
    private:
        DeviceOptions deviceOptions; 
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
        void drawDebug(const DrawCallback &cb) const noexcept
        {
            root.draw(cb);
        }

        BodyHandle<Traits> createBody(Shape shape, const Transform &trans, uint32_t options = Flags::Active | Flags::RigidBodyDynamic) noexcept;
        BodyHandle<Traits> createBody(Shape shape, const Transform &trans, const Kinematics &kin, uint32_t options = Flags::Active | Flags::RigidBodyDynamic) noexcept;

        void addToCollisionPool(Body &body) noexcept;
        void removeFromCollisionPool(Body &body) noexcept;

        void step(float dt) noexcept;

        [[nodiscard]] bool bodyExists(uint32_t id) const noexcept;
        [[nodiscard]] bool isBodyInCollision(uint32_t id) const noexcept
        {
            return collisionFlags[id] != 0;
        }

        [[nodiscard]] Body &getBodyUnsafe(uint32_t id) noexcept
        {
            return getBodyFromSparse(id);
        }
        [[nodiscard]] const Body &getBodyUnsafe(uint32_t id) const noexcept
        {
            return getBodyFromSparse(id);
        }

        [[nodiscard]] Body *getBody(uint32_t id) noexcept;
        [[nodiscard]] const Body *getBody(uint32_t id) const noexcept;

        [[nodiscard]] std::vector<CollisionInfo<Traits>> getAllCollisions(uint32_t id) noexcept;

        void removeBody(const BodyHandle<Traits> &handle) noexcept;
        void removeBody(uint32_t bodyID) noexcept;

        void clear() noexcept;

        PhysicsWorld(DeviceOptions devOptions = DeviceOptions::CPU) noexcept
            : deviceOptions(devOptions)
        {
#if !HAS_CUDA
            if(deviceOptions == DeviceOptions::GPU)
            {
                deviceOptions = DeviceOptions::CPU;
                BaseLogger::warn("Trying to do gpu work without cuda! Fallback on cpu");
            }
#endif
        }
        ~PhysicsWorld() noexcept = default;

    private:
        BodyHandle<Traits> createBodyFull(Shape shape, const Transform &trans, const Kinematics &kin, uint32_t options) noexcept;

        void step() noexcept;
        void updateCollisionFlags() noexcept;
        void reset(Body &obj) const noexcept;

        void broadPhaseCpu() noexcept;
        void broadPhaseGpu() noexcept;

        void setupCollision(Body &body) noexcept;
        // This is unchecked
        [[nodiscard]] const Body &getBodyFromSparse(uint32_t id) const noexcept
        {
            return bodies[sparse[id]];
        }
        // This is unchecked
        [[nodiscard]] Body &getBodyFromSparse(uint32_t id) noexcept
        {
            return bodies[sparse[id]];
        }

        [[nodiscard]] uint32_t getNextId() noexcept;
    };

    using PhysicsWorld2D = PhysicsWorld<Traits2D>;
    using PhysicsWorld3D = PhysicsWorld<Traits3D>;

    extern template class PhysicsWorld<Traits2D>;
    extern template class PhysicsWorld<Traits3D>;

    template <DimensionalTraits Traits>
    class BodyHandle
    {
    private:
        using Body = typename Traits::Body;

        uint32_t id;
        PhysicsWorld<Traits> *world;

    public:
        BodyHandle(uint32_t bodyID, PhysicsWorld<Traits> *pworld)
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

        [[nodiscard]] std::vector<CollisionInfo<Traits>> getCollisions() const noexcept
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

    using BodyHandle2D = BodyHandle<Traits2D>;
    using BodyHandle3D = BodyHandle<Traits3D>;

    template <DimensionalTraits Traits>
    struct CollisionInfo
    {
        BodyHandle<Traits> other;

        typename Traits::Vector normal;

        float depth;
    };

    using CollisionInfo2D = CollisionInfo<Traits2D>;
    using CollisionInfo3D = CollisionInfo<Traits3D>;

} // namespace sas
