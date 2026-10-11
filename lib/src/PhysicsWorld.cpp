#include "PhysicsWorld.hpp"

#include "Util.hpp"

#include <utility>
#include <algorithm>

template <sas::DimensionalTraits Traits>
sas::BodyHandle<Traits> sas::PhysicsWorld<Traits>::createBody(Shape shape, const Transform &trans, uint32_t options) noexcept
{
    return createBodyFull(shape, trans, {}, options);
}
template <sas::DimensionalTraits Traits>
sas::BodyHandle<Traits> sas::PhysicsWorld<Traits>::createBody(Shape shape, const Transform &trans, const Kinematics &kin, uint32_t options) noexcept
{
    return createBodyFull(shape, trans, kin, options);
}

// Default
// Flags::Active | Flags::RigidBody
template <sas::DimensionalTraits Traits>
sas::BodyHandle<Traits> sas::PhysicsWorld<Traits>::createBodyFull(Shape shape, const Transform &trans, const Kinematics &kin, uint32_t options) noexcept
{
    uint32_t newID = getNextId();
    uint32_t internalIndex = static_cast<uint32_t>(bodies.size());

    if (newID >= sparse.size())
    {
        sparse.resize(newID + 1, -1);
        collisionFlags.resize(newID + 1, 0);
    }

    Body newBody{trans, kin, shape, newID, options, 0};

    Traits::initializeBodyPhysics(newBody);

    setupCollision(newBody);

    sparse[newID] = internalIndex;
    dense.emplace_back(newID);
    bodies.push_back(newBody);

    return {newID, this};
}

template <sas::DimensionalTraits Traits>
void sas::PhysicsWorld<Traits>::setupCollision(Body &body) noexcept
{
    constexpr uint32_t typeMask = Flags::RigidBodyDynamic | Flags::RigidBodyStatic | Flags::RigidBodyKinematic | Flags::Trigger;

    if (body.hasFlag(Flags::Active) && (body.hasFlag(typeMask)))
    {
        body.collisionMask = Flags::Layer1 | Flags::Mask1;
        activeIDs.push_back(body.bodyID);
        addToCollisionPool(body);
    }
}

template <sas::DimensionalTraits Traits>
void sas::PhysicsWorld<Traits>::removeBody(const BodyHandle<Traits> &handle) noexcept
{
    removeBody(handle.get()->bodyID);
}

template <sas::DimensionalTraits Traits>
void sas::PhysicsWorld<Traits>::removeBody(uint32_t bodyID) noexcept
{
    if (bodies.empty())
        return;

    if (!bodyExists(bodyID))
        return;
    int indToRemove = sparse[bodyID];
    int lastIndex = static_cast<int>(bodies.size()) - 1;

    uint32_t lastID = dense[lastIndex];

    if (indToRemove != lastIndex)
    {
        bodies[indToRemove].flags = 0;
        bodies[indToRemove] = std::move(bodies[lastIndex]);

        sparse[lastID] = indToRemove;
        dense[indToRemove] = lastID;

        root.updateObject(bodies[indToRemove], 0.f);
    }

    auto it = std::find(activeIDs.begin(), activeIDs.end(), bodyID);

    if (it != activeIDs.end())
    {
        *it = std::move(activeIDs.back());
        activeIDs.pop_back();
    }

    bodies.pop_back();
    dense.pop_back();

    sparse[bodyID] = -1;

    freeIDs.push_back(bodyID);
    root.remove(bodyID);
}
template <sas::DimensionalTraits Traits>
uint32_t sas::PhysicsWorld<Traits>::getNextId() noexcept
{
    if (!freeIDs.empty())
    {
        uint32_t recycledID = freeIDs.back();
        freeIDs.pop_back();
        return recycledID;
    }

    return idCounter++;
}

template <sas::DimensionalTraits Traits>
void sas::PhysicsWorld<Traits>::step(float realDeltaTime) noexcept
{
    if (realDeltaTime > 0.25f)
        realDeltaTime = 0.25f;

    timeAccumulator += realDeltaTime;

    deltaTime = settings.physicsDt;

    frameContacts.clear();
    frameContacts.reserve(contacts.capacity());

    std::fill(collisionFlags.begin(), collisionFlags.end(), 0);

    while (timeAccumulator >= settings.physicsDt)
    {
        step();
        timeAccumulator -= settings.physicsDt;
    }
}

// MARK: Step
template <sas::DimensionalTraits Traits>
void sas::PhysicsWorld<Traits>::step() noexcept
{
    contacts.clear();

    for (uint32_t id : activeIDs)
    {
        Body &obj = bodies[sparse[id]];

        if (!(obj.flags & (Flags::RigidBodyStatic | Flags::RigidBodyKinematic | Flags::Trigger)))
        {
            if (obj.kinematics.inverseMass > 0)
            {
                Traits::applyForces(obj, settings);
                Traits::integrateVelocity(obj, deltaTime);
            }
        }
    }

    for (uint32_t id : activeIDs)
    {
        Body &obj = bodies[sparse[id]];

        if (!(obj.flags & Flags::RigidBodyStatic))
        {
            Traits::integratePosition(obj, deltaTime);
        }
    }

    for (uint32_t id : activeIDs)
    {
        Body &obj = bodies[sparse[id]];

        if (obj.flags & Flags::InCollisionPool)
        {
            const float velocityLength = obj.kinematics.velocity.length();
            const float predictiveMargin = std::max(2.0f, velocityLength * deltaTime * 3.0f);
            root.updateObject(obj, predictiveMargin);
        }
    }

    for (uint32_t id : activeIDs)
    {
        Body &obj = bodies[sparse[id]];
        if (obj.hasFlag(Flags::RigidBodyStatic) || obj.hasFlag(Flags::Trigger))
        {
            obj.kinematics.reset();
            continue;
        }

        std::vector<uint32_t> potentialCollisions;

        root.query(computeTightAABB2D(obj), potentialCollisions);

        for (uint32_t otherID : potentialCollisions)
        {
            auto &other = bodies[sparse[otherID]];

            bool otherIsSkipped = other.hasFlag(Flags::RigidBodyStatic) || other.hasFlag(Flags::Trigger);

            if (!otherIsSkipped && obj.bodyID >= otherID)
                continue;

            uint32_t objLayer = obj.collisionMask & 0x0000FFFF;
            uint32_t objMask = (obj.collisionMask & 0xFFFF0000) >> 16;

            uint32_t otherLayer = other.collisionMask & 0x0000FFFF;
            uint32_t otherMask = (other.collisionMask & 0xFFFF0000) >> 16;

            if (!(objMask & otherLayer) || !(otherMask & objLayer))
                continue;

            Traits::checkCollisionDispatcher(obj, other, contacts);
        }
        
    }

    const auto &shouldResolveCollision = [&](const Contact2D &contact)
    {
        const Body &bodyA = getBodyFromSparse(contact.bodyA);
        const Body &bodyB = getBodyFromSparse(contact.bodyB);
        if (bodyA.hasFlag(Flags::Trigger) || bodyB.hasFlag(Flags::Trigger))
        {
            return false;
        }

        return true;
    };

    constexpr size_t solverIterations = 10;
    for (size_t i = 0; i < solverIterations; ++i)
    {
        for (auto &contact : contacts)
        {
            if (shouldResolveCollision(contact))
            {
                Body &obj = getBodyFromSparse(contact.bodyA);
                Body &other = getBodyFromSparse(contact.bodyB);

                Traits::resolveCollision(obj, other, contact);
            }
        }
    }

    for (auto &c : contacts)
    {
        if (shouldResolveCollision(c))
        {
            Body &obj = getBodyFromSparse(c.bodyA);
            Body &other = getBodyFromSparse(c.bodyB);

            Traits::correctPosition(obj, other, c);
        }
    }

    for (uint32_t id : activeIDs)
    {
        Body &obj = bodies[sparse[id]];
        reset(obj);
    }

    updateCollisionFlags();
}

template <sas::DimensionalTraits Traits>
void sas::PhysicsWorld<Traits>::updateCollisionFlags() noexcept
{
    for (const auto &contact : contacts)
    {
        collisionFlags[contact.bodyA] = 1;
        collisionFlags[contact.bodyB] = 1;

        frameContacts.emplace_back(contact);
    }
}

template <sas::DimensionalTraits Traits>
void sas::PhysicsWorld<Traits>::addToCollisionPool(Body &body) noexcept
{
    if (!(body.flags & Flags::InCollisionPool))
    {
        body.flags |= Flags::InCollisionPool;

        root.insert(body.bodyID, Traits::computeTightAABB(body));
    }
}

template <sas::DimensionalTraits Traits>
void sas::PhysicsWorld<Traits>::removeFromCollisionPool(Body &body) noexcept
{
    if (body.flags & Flags::InCollisionPool)
    {
        body.flags &= ~Flags::InCollisionPool;

        root.remove(body.bodyID);
    }
}
template <sas::DimensionalTraits Traits>
void sas::PhysicsWorld<Traits>::reset(Body &obj) const noexcept
{
    obj.kinematics.acceleration = {0, 0};
}

template <sas::DimensionalTraits Traits>
bool sas::PhysicsWorld<Traits>::bodyExists(uint32_t id) const noexcept
{
    if (id >= sparse.size())
    {
        return false;
    }

    if (sparse[id] == -1)
    {
        return false;
    }

    return true;
}

template <sas::DimensionalTraits Traits>
typename Traits::Body *sas::PhysicsWorld<Traits>::getBody(uint32_t id) noexcept
{
    if (bodyExists(id)) [[likely]]
    {
        return &getBodyFromSparse(id);
    }

    return nullptr;
}

template <sas::DimensionalTraits Traits>
const typename Traits::Body *sas::PhysicsWorld<Traits>::getBody(uint32_t id) const noexcept
{
    if (bodyExists(id)) [[likely]]
    {
        return &getBodyFromSparse(id);
    }

    return nullptr;
}

template <sas::DimensionalTraits Traits>
std::vector<sas::CollisionInfo<Traits>> sas::PhysicsWorld<Traits>::getAllCollisions(uint32_t id) noexcept
{
    if (!isBodyInCollision(id))
        return {};

    std::vector<sas::CollisionInfo<Traits>> collisions;

    collisions.reserve(8);

    for (const auto &contact : frameContacts)
    {
        if (contact.bodyA == id)
        {
            collisions.emplace_back(BodyHandle<Traits>{contact.bodyB, this}, contact.normal, contact.overlap);
        }
        else if (contact.bodyB == id)
        {
            collisions.emplace_back(BodyHandle<Traits>{contact.bodyA, this}, -1 * contact.normal, contact.overlap);
        }
    }

    return collisions;
}

template <sas::DimensionalTraits Traits>
void sas::PhysicsWorld<Traits>::clear() noexcept
{
    root.clear();
    bodies.clear();
    bodies.clear();
    sparse.clear();
    collisionFlags.clear();
    dense.clear();
    freeIDs.clear();
    activeIDs.clear();
    contacts.clear();

    idCounter = 0;
}

template class sas::PhysicsWorld<sas::Traits2D>;
// template class sas::PhysicsWorld<sas::Traits3D>;