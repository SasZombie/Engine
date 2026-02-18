#include "PhysicsWorld.hpp"

#include "Util.hpp"

sas::PhysicsWorld::PhysicsWorld(Rectangle dims) noexcept
    : boundaries(dims)
{
}
void sas::PhysicsWorld::DrawDebug(const DrawCallback &cb) const noexcept
{
    root.Draw(cb);
}
void sas::PhysicsWorld::addToCollisionPool(const Body &body) noexcept
{
    root.insert(body.bodyID, ComputeFatAABB(body));
}

void sas::PhysicsWorld::Step(std::vector<Body> &objects, float dt) noexcept
{
    for (auto &obj : objects)
    {
        obj.isColliding = false;
        if (obj.kinematics.inverseMass > 0.f)
        {
            ApplyForces(obj);
            Integrate(obj, dt);
            ResolveConstraints(obj, dt);
            Reset(obj);

            const float velocityLength = obj.kinematics.velocity.length();

            const float predictiveMargin = std::max(2.0f, velocityLength * dt * 3.0f);
            root.UpdateObject(obj, predictiveMargin);
        }
    }

    for (auto &obj : objects)
    {
        CheckCollision(objects, obj);
    }
}

void sas::PhysicsWorld::CheckCollision(std::vector<Body> &objects, Body &obj) noexcept
{
    if (floatAlmostEqual(obj.kinematics.inverseMass, 0.0f) && obj.kinematics.velocity.lengthSq() < 0.01f)
        return;

    std::vector<uint32_t> potentialCollisions;

    root.Query(ComputeTightAABB(obj), potentialCollisions);

    for (uint32_t otherID : potentialCollisions)
    {
        if (obj.bodyID >= otherID)
            continue;
        auto &other = objects[otherID];

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
    }
}

void sas::PhysicsWorld::Clear() noexcept
{
    root.Clear();
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

void sas::PhysicsWorld::Reset(Body &obj) const noexcept
{
    obj.kinematics.acceleration = {0, 0};
}