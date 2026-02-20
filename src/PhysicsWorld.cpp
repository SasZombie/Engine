#include "PhysicsWorld.hpp"

#include "Util.hpp"

sas::PhysicsWorld::PhysicsWorld(Rectangle dims) noexcept
    : boundaries(dims)
{
}

sas::BodyHandle sas::PhysicsWorld::CreateBody(Shape shape, const Transform &trans) noexcept
{
    uint32_t newID = GetNextId();
    uint32_t internalIndex = static_cast<uint32_t>(bodies.size());

    if (newID >= sparse.size())
    {
        sparse.resize(newID + 1, -1);
        collisionFlags.resize(newID + 1, 0);
    }

    bodies.emplace_back(trans, Kinematics{}, shape, newID);

    sparse[newID] = internalIndex;
    dense.emplace_back(newID);

    return {newID, this};
}

void sas::PhysicsWorld::RemoveBody(uint32_t bodyID) noexcept
{
    if (bodies.empty())
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
    }

    bodies.pop_back();
    dense.pop_back();

    sparse[bodyID] = -1;

    freeIDs.push_back(bodyID);
    root.remove(bodyID);
}

uint32_t sas::PhysicsWorld::GetNextId() noexcept
{
    if (!freeIDs.empty())
    {
        uint32_t recycledID = freeIDs.back();
        freeIDs.pop_back();
        return recycledID;
    }

    return idCounter++;
}

void sas::PhysicsWorld::Step(float dt) noexcept
{
    contacts.clear();
    for (auto &obj : bodies)
    {
        if (obj.kinematics.inverseMass > 0.f)
        {
            ApplyForces(obj);
            Integrate(obj, dt);
            ResolveConstraints(obj, dt);
            Reset(obj);

            const float velocityLength = obj.kinematics.velocity.length();

            const float predictiveMargin = std::max(2.0f, velocityLength * dt * 3.0f);
            if (obj.flags & Filter::Active)
            {
                root.UpdateObject(obj, predictiveMargin);
            }
        }
    }

    for (auto &obj : bodies)
    {
        if (obj.flags & Filter::Active)
        {
            CheckCollision(obj);
        }
    }

    UpdateCollisionFlags();
}

void sas::PhysicsWorld::CheckCollision(Body &obj) noexcept
{
    if (floatAlmostEqual(obj.kinematics.inverseMass, 0.0f) && obj.kinematics.velocity.lengthSq() < 0.01f)
        return;

    std::vector<uint32_t> potentialCollisions;

    root.Query(ComputeTightAABB(obj), potentialCollisions);

    for (uint32_t otherID : potentialCollisions)
    {
        if (obj.bodyID >= otherID)
            continue;
        auto &other = bodies[otherID];

        float dx = obj.transform.position.x - other.transform.position.x;
        float dy = obj.transform.position.y - other.transform.position.y;
        float distanceSq = dx * dx + dy * dy;
        float combinedRad = obj.shape.radius + other.shape.radius;

        if (distanceSq > combinedRad * combinedRad)
            continue;

        float distance = std::sqrt(distanceSq);
        math::Vec2 normal = (distance > 0.0001f)
                                ? math::Vec2(dx / distance, dy / distance)
                                : math::Vec2(0, 1);

        math::Vec2 relVel = obj.kinematics.velocity - other.kinematics.velocity;
        float velAlongNormal = math::dotProduct(relVel, normal);

        float overlap = combinedRad - distance;
        float totalInvMass = obj.kinematics.inverseMass + other.kinematics.inverseMass;

        if (totalInvMass > 0.0f)
        {
            math::Vec2 correction = normal * (overlap / totalInvMass) * 0.5f;
            obj.transform.position = obj.transform.position + correction * obj.kinematics.inverseMass;
            other.transform.position = other.transform.position - correction * other.kinematics.inverseMass;
        }

        if (velAlongNormal < 0)
        {
            float e = std::min(obj.kinematics.restituition, other.kinematics.restituition);
            float j = -(1.0f + e) * velAlongNormal;
            j /= totalInvMass;

            math::Vec2 impulse = normal * j;
            obj.kinematics.velocity = obj.kinematics.velocity + impulse * obj.kinematics.inverseMass;
            other.kinematics.velocity = other.kinematics.velocity - impulse * other.kinematics.inverseMass;
        }

        contacts.emplace_back(obj.bodyID, otherID, normal, 1);
    }
}


void sas::PhysicsWorld::UpdateCollisionFlags() noexcept
{

    std::fill(collisionFlags.begin(), collisionFlags.end(), 0);

    for (const auto &contact : contacts)
    {
        collisionFlags[contact.bodyA] = 1;
        collisionFlags[contact.bodyB] = 1;
    }
}

void sas::PhysicsWorld::Clear() noexcept
{
    root.Clear();
    bodies.clear();
    idCounter = 0;
}

void sas::PhysicsWorld::ApplyForces(Body &obj) const noexcept
{
    float dragForceY = obj.kinematics.velocity.y * settings.dragCoeff;
    obj.kinematics.acceleration.y += settings.gravity - (dragForceY * obj.kinematics.inverseMass);

    float dragForceX = obj.kinematics.velocity.x * settings.dragCoeff;
    obj.kinematics.acceleration.x += -1 * (dragForceX * obj.kinematics.inverseMass);
}

void sas::PhysicsWorld::Integrate(Body &obj, float dt) const noexcept
{
    obj.kinematics.velocity = obj.kinematics.velocity + obj.kinematics.acceleration * dt;
    obj.transform.position = obj.transform.position + obj.kinematics.velocity * dt;
}

void sas::PhysicsWorld::ResolveConstraints(Body &obj, float dt) const noexcept
{
    float groundLevel = boundaries.Height - obj.shape.radius;

    ResolveBroadGround(obj, groundLevel);
    ResolveBroadCeil(obj, boundaries.y + obj.shape.radius);

    float wall = boundaries.Width - obj.shape.radius;
    ResolveBroadHigher(obj, wall);
    ResolveBroadLower(obj, boundaries.x + obj.shape.radius);
}

void sas::PhysicsWorld::ResolveBroadGround(Body &obj, float wall) const noexcept
{
    if (obj.transform.position.y >= wall)
    {
        obj.transform.position.y = wall;
        if (obj.kinematics.velocity.y > 0)
        {
            obj.kinematics.velocity.y *= -obj.kinematics.restituition;
        }

        if (std::abs(obj.kinematics.velocity.x) > 0)
        {
            obj.kinematics.velocity.x *= settings.groundFriction;
            if (std::abs(obj.kinematics.velocity.x) < 2.f)
            {
                obj.kinematics.velocity.x = 0;
            }
        }

        // jittering
        if (std::abs(obj.kinematics.velocity.y) < 0.1f)
        {
            obj.kinematics.velocity.y = 0;
        }
    }
}

void sas::PhysicsWorld::ResolveBroadCeil(Body &obj, float wall) const noexcept
{
    if (obj.transform.position.y <= wall)
    {
        obj.transform.position.y = wall;
        if (obj.kinematics.velocity.y < 0)
        {
            obj.kinematics.velocity.y *= -obj.kinematics.restituition;
        }

        if (std::abs(obj.kinematics.velocity.x) > 0)
        {
            obj.kinematics.velocity.x *= settings.groundFriction;
            if (std::abs(obj.kinematics.velocity.x) < 2.f)
            {
                obj.kinematics.velocity.x = 0;
            }
        }

        // jittering
        if (std::abs(obj.kinematics.velocity.y) < 0.1f)
        {
            obj.kinematics.velocity.y = 0;
        }
    }
}

void sas::PhysicsWorld::ResolveBroadLower(Body &obj, float wall) const noexcept
{
    if (obj.transform.position.x <= wall)
    {
        obj.transform.position.x = wall;
        if (obj.kinematics.velocity.x < 0)
        {
            obj.kinematics.velocity.x *= -obj.kinematics.restituition;
        }

        if (std::abs(obj.kinematics.velocity.y) > 0.1f)
        {
            obj.kinematics.velocity.y *= settings.wallFriction;
        }
        // jittering
        if (std::abs(obj.kinematics.velocity.x) < 0.1f)
        {
            obj.kinematics.velocity.x = 0;
        }
    }
}

void sas::PhysicsWorld::ResolveBroadHigher(Body &obj, float wall) const noexcept
{
    if (obj.transform.position.x >= wall)
    {
        obj.transform.position.x = wall;

        if (obj.kinematics.velocity.x > 0)
        {
            obj.kinematics.velocity.x *= -obj.kinematics.restituition;
        }
        if (std::abs(obj.kinematics.velocity.y) > 0.1f)
        {
            obj.kinematics.velocity.y *= settings.wallFriction;
        }
        // jittering
        if (std::abs(obj.kinematics.velocity.x) < 0.1f)
        {
            obj.kinematics.velocity.x = 0;
        }
    }
}

void sas::PhysicsWorld::AddToCollisionPool(const Body &body) noexcept
{
    root.insert(body.bodyID, ComputeFatAABB(body));
}

void sas::PhysicsWorld::RemoveFromCollisionPool(const Body &body) noexcept
{
    root.remove(body.bodyID);
}

void sas::PhysicsWorld::Reset(Body &obj) const noexcept
{
    obj.kinematics.acceleration = {0, 0};
}

// TODO
bool sas::PhysicsWorld::BodyExists(uint32_t id) const noexcept
{
    return false;
}

bool sas::PhysicsWorld::IsBodyInCollision(uint32_t id) const noexcept
{
    return collisionFlags[id] != 0;
}

sas::Body &sas::PhysicsWorld::GetBody(uint32_t id) noexcept
{
    return bodies[sparse[id]];
}

std::vector<sas::CollisionInfo> sas::PhysicsWorld::GetAllCollisions(uint32_t id) noexcept
{
    if (!IsBodyInCollision(id))
        return {};

    std::vector<sas::CollisionInfo> collisions;

    collisions.reserve(8);

    for (const auto &contact : contacts)
    {
        if (contact.bodyA == id)
        {
            collisions.emplace_back(BodyHandle{contact.bodyB, this}, contact.normal, contact.depth);
        }
        else if (contact.bodyB == id)
        {
            collisions.emplace_back(BodyHandle{contact.bodyA, this}, -1 * contact.normal, contact.depth);
        }
    }

    return collisions;
}

void sas::PhysicsWorld::RemoveBody(const BodyHandle &handle) noexcept
{
    RemoveBody(handle.get()->bodyID);
}

void sas::PhysicsWorld::DrawDebug(const DrawCallback &cb) const noexcept
{
    root.Draw(cb);
}