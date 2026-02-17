#include "PhysicsWorld.hpp"

#include <iostream>

sas::PhysicsWorld::PhysicsWorld(Rectangle dims) noexcept
    : boundaries(dims)
{
}
void sas::PhysicsWorld::addToCollisionPool(const Body &body) noexcept
{
    root.insert(body);
}
void sas::PhysicsWorld::Step(std::vector<Body> &objects, float dt)  noexcept
{
    for (auto &obj : objects)
    {
        if (obj.kinematics.inverseMass > 0.f)
        {
            ApplyForces(obj);
            Integrate(obj, dt);
            ResolveConstraints(obj, dt);

            Reset(obj);
        }

        root.UpdateObject(obj);
        root.Draw();
        std::vector<uint32_t> potentialCollisions;
        root.Query(ComputeFatAABB(obj), potentialCollisions);

        for (uint32_t other : potentialCollisions)
        {
            if (obj.bodyID == other)
                continue;

            // Narrow phase
            float dx = obj.transform.position.x - objects[other].transform.position.x;
            float dy = obj.transform.position.y - objects[other].transform.position.y;
            float distanceSq = dx * dx + dy * dy;
            float combinedRad = obj.shape.radius + objects[other].shape.radius;

            if (distanceSq <= combinedRad * combinedRad)
            {
                obj.isColliding = true;
                objects[other].isColliding = true;
            }
        }
    }
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
    if (obj.transform.position.y >= groundLevel)
    {
        obj.transform.position.y = groundLevel;
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

    float wall = boundaries.Width - obj.shape.radius;
    ResolveBroadHigher(obj, wall);
    ResolveBroadLower(obj, boundaries.x + obj.shape.radius);
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