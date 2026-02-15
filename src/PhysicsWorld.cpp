#include "PhysicsWorld.hpp"

void sas::PhysicsWorld::Step(std::vector<Body> &objects, float dt) const noexcept
{
    for (auto &obj : objects)
    {
        if (obj.kinematics && obj.kinematics->inverseMass > 0.f)
        {
            ApplyForces(obj);
            Integrate(obj, dt);
            ResolveConstraints(obj);
        }
    }
}



sas::PhysicsWorld::PhysicsWorld(Rectangle dims) noexcept
    : boundaries(dims)
{

}

void sas::PhysicsWorld::ApplyForces(Body &obj) const noexcept
{
    float dragForce = obj.kinematics->velocity.y * settings.dragCoeff;

    obj.kinematics->acceleration.y = settings.gravity - (dragForce * obj.kinematics->inverseMass);
}

void sas::PhysicsWorld::Integrate(Body &obj, float dt) const noexcept
{
    obj.kinematics->velocity.y += obj.kinematics->acceleration.y * dt;
    obj.transform.position.y += obj.kinematics->velocity.y * dt;
}

void sas::PhysicsWorld::ResolveConstraints(Body &obj) const noexcept
{   
    float groundLevel = boundaries.Height - obj.shape.radius;
    if (obj.transform.position.y >= groundLevel)
    {
        obj.transform.position.y = groundLevel;
        if (obj.kinematics->velocity.y > 0)
        {
            obj.kinematics->velocity.y *= - obj.kinematics->restituition;
        }
        // jittering
        if (std::abs(obj.kinematics->velocity.y) < 0.1f)
        {
            obj.kinematics->velocity.y = 0;
        }
    }
}
