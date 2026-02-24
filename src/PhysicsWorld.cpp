#include "PhysicsWorld.hpp"

#include "Util.hpp"

#include <utility>
#include <algorithm>

sas::PhysicsWorld::PhysicsWorld(Rectangle dims) noexcept
    : boundaries(dims)
{
}

sas::BodyHandle sas::PhysicsWorld::CreateBody(Shape shape, const Transform &trans, uint32_t options) noexcept
{
    return CreateBodyFull(shape, trans, {}, options);
}

sas::BodyHandle sas::PhysicsWorld::CreateBody(Shape shape, const Transform &trans, const Kinematics &kin, uint32_t options) noexcept
{
    return CreateBodyFull(shape, trans, {}, options);
}

// Default
// Flags::Active | Flags::RigidBody
sas::BodyHandle sas::PhysicsWorld::CreateBodyFull(Shape shape, const Transform &trans, const Kinematics &kin, uint32_t options) noexcept
{
    uint32_t newID = GetNextId();
    uint32_t internalIndex = static_cast<uint32_t>(bodies.size());

    if (newID >= sparse.size())
    {
        sparse.resize(newID + 1, -1);
        collisionFlags.resize(newID + 1, 0);
    }

    Body newBody{trans, kin, shape, newID, options, 0};

    sparse[newID] = internalIndex;
    dense.emplace_back(newID);

    uint32_t mask = Flags::RigidBody | Flags::Static;

    if ((options & Flags::Active) && (options & mask))
    {
        newBody.collisionMask = Flags::Layer1 | Flags::Mask1;

        activeIDs.push_back(newID);
        AddToCollisionPool(newBody);
    }

    if (shape.type == ShapeType::Box)
    {
        float mass = (kin.inverseMass > 0) ? 1.0f / kin.inverseMass : 0.0f;
        newBody.kinematics.inertia = (1.0f / 12.0f) * mass * (2 * shape.halfSize.x * shape.halfSize.x + 2 * shape.halfSize.y * shape.halfSize.y);
        newBody.kinematics.inverseInertia = (newBody.kinematics.inertia > 0) ? 1.0f / newBody.kinematics.inertia : 0.0f;
    }

    bodies.push_back(newBody);

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

        root.UpdateObject(bodies[indToRemove], 0.f);
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
    for (uint32_t id : activeIDs)
    {
        Body &obj = bodies[sparse[id]];

        bool isRigid = obj.flags & Flags::RigidBody;

        if (isRigid && (obj.kinematics.inverseMass > 0.f))
        {
            ApplyForces(obj);
            Integrate(obj, dt);
            ResolveConstraints(obj, dt);
            Reset(obj);
        }

        const float velocityLength = obj.kinematics.velocity.length();

        const float predictiveMargin = std::max(2.0f, velocityLength * dt * 3.0f);

        if (obj.flags & Flags::InCollisionPool)
        {
            root.UpdateObject(obj, predictiveMargin);
        }
    }

    for (uint32_t id : activeIDs)
    {
        Body &obj = bodies[sparse[id]];

        CheckCollisionDispatcher(obj);
    }

    UpdateCollisionFlags();
}
// TODO:This sounds interesting
// matrix[shapeA.type][shapeB.type](shapeA, shapeB)

void sas::PhysicsWorld::CheckCollisionDispatcher(Body &obj) noexcept
{
    // Forcing static objects to to have 0 vel
    // And 0 inverse mass otherwise
    if (obj.flags & Flags::Static)
    {
        obj.kinematics.velocity = {0, 0};
        obj.kinematics.inverseMass = 0;
        return;
    }

    std::vector<uint32_t> potentialCollisions;

    root.Query(ComputeTightAABB(obj), potentialCollisions);

    for (uint32_t otherID : potentialCollisions)
    {
        auto &other = bodies[sparse[otherID]];

        bool otherIsStatic = (other.flags & Flags::Static);

        if (!otherIsStatic && obj.bodyID >= otherID)
            continue;

        uint32_t objLayer = obj.collisionMask & 0x0000FFFF;
        uint32_t objMask = (obj.collisionMask & 0xFFFF0000) >> 16;

        uint32_t otherLayer = other.collisionMask & 0x0000FFFF;
        uint32_t otherMask = (other.collisionMask & 0xFFFF0000) >> 16;

        if (!(objMask & otherLayer) || !(otherMask & objLayer))
            continue;

        // This will crash if obj.shape.type is greater than the types
        // So if the user somehow updates the shape type with some random value
        // GG
        (this->*DispatchTable[static_cast<int>(obj.shape.type)][static_cast<int>(other.shape.type)])(obj, other);
    }
}

void sas::PhysicsWorld::CheckCollisionCircleCircle(Body &obj, Body &other) noexcept
{
    float dx = obj.transform.position.x - other.transform.position.x;
    float dy = obj.transform.position.y - other.transform.position.y;
    float distanceSq = dx * dx + dy * dy;
    // For circles scale x = scale y. If not then it is a bug
    float combinedRad = obj.shape.radius * obj.transform.scale.x + other.shape.radius * other.transform.scale.x;

    if (distanceSq > combinedRad * combinedRad)
        return;

    float distance = std::sqrt(distanceSq);
    math::Vec2 normal = (distance > 0.0001f)
                            ? math::Vec2(dx / distance, dy / distance)
                            : math::Vec2(0, 1);

    float overlap = combinedRad - distance;

    ResolveColision(obj, other, normal, overlap, {{0, 0}, {0, 0}});
}

struct BoxCorners
{
    sas::math::Vec2 v[4];
};

static BoxCorners GetBoxCorners(const sas::Body &b)
{
    float hx = b.shape.halfSize.x * b.transform.scale.x;
    float hy = b.shape.halfSize.y * b.transform.scale.y;
    float cosA = std::cos(b.transform.rotation);
    float sinA = std::sin(b.transform.rotation);

    auto rotate = [&](float px, float py)
    {
        return sas::math::Vec2{
            b.transform.position.x + (px * cosA - py * sinA),
            b.transform.position.y + (px * sinA + py * cosA)};
    };

    return {rotate(-hx, -hy), rotate(hx, -hy), rotate(hx, hy), rotate(-hx, hy)};
}

void sas::PhysicsWorld::CheckCollisionBoxBox(Body &obj, Body &other) noexcept
{
    math::Vec2 axes[4];
    float rotA = obj.transform.rotation;
    float rotB = other.transform.rotation;

    axes[0] = {std::cos(rotA), std::sin(rotA)};
    axes[1] = {-std::sin(rotA), std::cos(rotA)};
    axes[2] = {std::cos(rotB), std::sin(rotB)};
    axes[3] = {-std::sin(rotB), std::cos(rotB)};

    float minOverlap = std::numeric_limits<float>::max();
    math::Vec2 mtvAxis;

    BoxCorners cornersA = GetBoxCorners(obj);
    BoxCorners cornersB = GetBoxCorners(other);

    for (int i = 0; i < 4; i++)
    {
        math::Vec2 axis = axes[i];

        auto project = [&](const BoxCorners &corners)
        {
            float min = math::dotProduct(corners.v[0], axis);
            float max = min;
            for (int j = 1; j < 4; j++)
            {
                float p = math::dotProduct(corners.v[j], axis);
                min = std::min(min, p);
                max = std::max(max, p);
            }
            return std::pair{min, max};
        };

        auto [minA, maxA] = project(cornersA);
        auto [minB, maxB] = project(cornersB);

        float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (overlap <= 0)
            return;

        if (overlap < minOverlap)
        {
            minOverlap = overlap;
            mtvAxis = axis;
        }
    }

    math::Vec2 d = obj.transform.position - other.transform.position;
    if (math::dotProduct(d, mtvAxis) < 0)
    {
        mtvAxis = mtvAxis * -1.0f;
    }

    math::Vec2 contactPoint;

    auto GetDeepestPoint = [&](const BoxCorners &corners, const math::Vec2 &normal)
    {
        float minDot = std::numeric_limits<float>::max();
        math::Vec2 deepest;
        for (int i = 0; i < 4; i++)
        {
            float dot = math::dotProduct(corners.v[i], normal);
            if (dot < minDot)
            {
                minDot = dot;
                deepest = corners.v[i];
            }
        }
        return deepest;
    };

    contactPoint = GetDeepestPoint(cornersA, mtvAxis);

    math::Vec2 rA = contactPoint - obj.transform.position;
    math::Vec2 rB = contactPoint - other.transform.position;

    ResolveColision(obj, other, mtvAxis, minOverlap, {rA, rB});
}
void sas::PhysicsWorld::CheckCollisionCircleBox(Body &circle, Body &box) noexcept
{
    float r = circle.shape.radius * std::max(circle.transform.scale.x, circle.transform.scale.y);
    float hx = box.shape.halfSize.x * box.transform.scale.x;
    float hy = box.shape.halfSize.y * box.transform.scale.y;

    math::Vec2 d = circle.transform.position - box.transform.position;

    float cosA = std::cos(-box.transform.rotation);
    float sinA = std::sin(-box.transform.rotation);

    math::Vec2 localPos = {
        d.x * cosA - d.y * sinA,
        d.x * sinA + d.y * cosA};

    math::Vec2 closest = localPos;
    closest.x = std::clamp(closest.x, -hx, hx);
    closest.y = std::clamp(closest.y, -hy, hy);

    math::Vec2 localNormalVec = localPos - closest;
    float distSq = localNormalVec.lengthSq();

    if (distSq > r * r)
        return;

    float dist = std::sqrt(distSq);
    math::Vec2 localNormal;
    float overlap;

    if (dist > 0.0001f)
    {
        localNormal = localNormalVec / dist;
        overlap = r - dist;
    }
    else
    {
        float px = hx - std::abs(localPos.x);
        float py = hy - std::abs(localPos.y);

        if (px < py)
        {
            localNormal = (localPos.x > 0) ? math::Vec2(1, 0) : math::Vec2(-1, 0);
            overlap = r + px;
        }
        else
        {
            localNormal = (localPos.y > 0) ? math::Vec2(0, 1) : math::Vec2(0, -1);
            overlap = r + py;
        }
    }

    float cosW = std::cos(box.transform.rotation);
    float sinW = std::sin(box.transform.rotation);

    math::Vec2 worldNormal = {
        localNormal.x * cosW - localNormal.y * sinW,
        localNormal.x * sinW + localNormal.y * cosW};

    ResolveColision(circle, box, worldNormal, overlap, {{0, 0}, {0, 0}});
}

void sas::PhysicsWorld::ResolveColision(Body &obj, Body &other, math::Vec2 normal, float overlap, const std::pair<math::Vec2, math::Vec2>& rotComp) noexcept
{
    math::Vec2 relVel = obj.kinematics.velocity - other.kinematics.velocity;
    float velAlongNormal = math::dotProduct(relVel, normal);
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

        math::Vec2 r = {1, 2};

        float crossN = r * normal;

        obj.kinematics.angularVelocity += crossN * (j * obj.kinematics.inverseMass);
    }

    contacts.emplace_back(obj.bodyID, other.bodyID, normal, overlap);
}

void sas::PhysicsWorld::CheckCollisionBoxCircle(Body &obj, Body &other) noexcept
{
    CheckCollisionCircleBox(other, obj);
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
    obj.transform.rotation = obj.transform.rotation + obj.kinematics.angularVelocity * dt;
}

void sas::PhysicsWorld::ResolveConstraints(Body &obj, float dt) const noexcept
{
    float groundLevel = boundaries.Height - obj.shape.radius * obj.transform.scale.x;

    ResolveBroadGround(obj, groundLevel);
    ResolveBroadCeil(obj, boundaries.y + obj.shape.radius * obj.transform.scale.x);

    float wall = boundaries.Width - obj.shape.radius * obj.transform.scale.x;
    ResolveBroadHigher(obj, wall);
    ResolveBroadLower(obj, boundaries.x + obj.shape.radius * obj.transform.scale.x);
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

void sas::PhysicsWorld::AddToCollisionPool(Body &body) noexcept
{
    if (!(body.flags & Flags::InCollisionPool))
    {
        body.flags |= Flags::InCollisionPool;

        root.insert(body.bodyID, ComputeFatAABB(body));
    }
}

void sas::PhysicsWorld::RemoveFromCollisionPool(Body &body) noexcept
{
    if (body.flags & Flags::InCollisionPool)
    {
        body.flags &= ~Flags::InCollisionPool;

        root.remove(body.bodyID);
    }
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

void sas::PhysicsWorld::Clear() noexcept
{
    root.Clear();
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