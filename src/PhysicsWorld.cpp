#include "PhysicsWorld.hpp"

#include "Util.hpp"

#include <utility>
#include <algorithm>

sas::BodyHandle sas::PhysicsWorld::createBody(Shape shape, const Transform &trans, uint32_t options) noexcept
{
    return createBodyFull(shape, trans, {}, options);
}

sas::BodyHandle sas::PhysicsWorld::createBody(Shape shape, const Transform &trans, const Kinematics &kin, uint32_t options) noexcept
{
    return createBodyFull(shape, trans, kin, options);
}

// Default
// Flags::Active | Flags::RigidBody
sas::BodyHandle sas::PhysicsWorld::createBodyFull(Shape shape, const Transform &trans, const Kinematics &kin, uint32_t options) noexcept
{
    uint32_t newID = getNextId();
    uint32_t internalIndex = static_cast<uint32_t>(bodies.size());

    if (newID >= sparse.size())
    {
        sparse.resize(newID + 1, -1);
        collisionFlags.resize(newID + 1, 0);
    }

    Body newBody{trans, kin, shape, newID, options, 0};

    sparse[newID] = internalIndex;
    dense.emplace_back(newID);

    uint32_t mask = Flags::RigidBodyDynamic | Flags::RigidBodyStatic;

    if ((options & Flags::Active) && (options & mask))
    {
        newBody.collisionMask = Flags::Layer1 | Flags::Mask1;

        activeIDs.push_back(newID);
        addToCollisionPool(newBody);
    }

    if (!(options & Flags::BodyFlags::RigidBodyStatic))
    {
        if (shape.type == ShapeType::Box)
        {
            float mass = (kin.inverseMass > 0) ? 1.0f / kin.inverseMass : 0.0f;

            float hx = shape.halfSize.x * trans.scale.x;
            float hy = shape.halfSize.y * trans.scale.y;

            newBody.kinematics.inertia = (1.0f / 3.0f) * mass * (hx * hx + hy * hy);

            newBody.kinematics.inverseInertia = (newBody.kinematics.inertia > 0) ? 1.0f / newBody.kinematics.inertia : 0.0f;
        }
        else if (shape.type == ShapeType::Circle)
        {
            float mass = (kin.inverseMass > 0) ? 1.0f / kin.inverseMass : 0.0f;

            float r = shape.radius * trans.scale.x;
            newBody.kinematics.inertia = 0.5f * mass * (r * r);

            newBody.kinematics.inverseInertia = (newBody.kinematics.inertia > 0) ? 1.0f / newBody.kinematics.inertia : 0.0f;
        }
    }

    bodies.push_back(newBody);

    return {newID, this};
}

void sas::PhysicsWorld::removeBody(uint32_t bodyID) noexcept
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

uint32_t sas::PhysicsWorld::getNextId() noexcept
{
    if (!freeIDs.empty())
    {
        uint32_t recycledID = freeIDs.back();
        freeIDs.pop_back();
        return recycledID;
    }

    return idCounter++;
}

// MARK: Step
void sas::PhysicsWorld::step(float dt) noexcept
{
    deltaTime = dt;
    contacts.clear();
    for (uint32_t id : activeIDs)
    {
        Body &obj = bodies[sparse[id]];

        bool isRigid = obj.flags & Flags::RigidBodyDynamic;

        if (isRigid && (obj.kinematics.inverseMass > 0))
        {
            applyForces(obj);
        }
    }

    for (uint32_t id : activeIDs)
    {
        integrateVelocity(bodies[sparse[id]]);
        integratePosition(bodies[sparse[id]]);
    }

    for (uint32_t id : activeIDs)
    {
        Body &obj = bodies[sparse[id]];

        const float velocityLength = obj.kinematics.velocity.length();
        // 3 frames prediction
        const float predictiveMargin = std::max(2.0f, velocityLength * dt * 3.0f);

        if (obj.flags & Flags::InCollisionPool)
        {
            root.updateObject(obj, predictiveMargin);
        }
    }

    for (uint32_t id : activeIDs)
    {
        Body &obj = bodies[sparse[id]];

        checkCollisionDispatcher(obj);
    }

    constexpr size_t solverIterations = 10;
    for (size_t i = 0; i < solverIterations; ++i)
    {
        for (auto &contact : contacts)
        {
            resolveCollision(contact);
        }
    }

    constexpr size_t positionIterations = 1;
    for (size_t i = 0; i < positionIterations; ++i)
    {
        for (auto &c : contacts)
        {
            correctPosition(c);
        }
    }

    for (uint32_t id : activeIDs)
    {
        reset(bodies[sparse[id]]);
    }

    updateCollisionFlags();
}

void sas::PhysicsWorld::checkCollisionDispatcher(Body &obj) noexcept
{
    // Forcing static objects to to have 0 vel
    // And 0 inverse mass otherwise
    if (obj.flags & Flags::RigidBodyStatic)
    {
        obj.kinematics.reset();
        return;
    }

    std::vector<uint32_t> potentialCollisions;

    root.query(computeTightAABB(obj), potentialCollisions);

    for (uint32_t otherID : potentialCollisions)
    {
        auto &other = bodies[sparse[otherID]];

        bool otherIsStatic = (other.flags & Flags::RigidBodyStatic);

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
        (this->*DispatchTable[static_cast<int>(obj.shape.type)][static_cast<int>(other.shape.type)])(obj, other);
    }
}

void sas::PhysicsWorld::checkCollisionCircleCircle(Body &obj, Body &other) noexcept
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

    Contact cont;

    cont.bodyA = obj.bodyID;
    cont.bodyB = other.bodyID;

    cont.normal = normal;
    cont.overlap = overlap;
    cont.pointCount = 0;

    contacts.emplace_back(cont);

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
void sas::PhysicsWorld::checkCollisionBoxBox(Body &obj, Body &other) noexcept
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

    // --- 1. SEPARATING AXIS THEOREM (SAT) ---
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
        if (overlap <= 0.0f)
            return; // Separating axis found, early exit

        if (overlap < minOverlap)
        {
            minOverlap = overlap;
            mtvAxis = axis;
        }
    }

    // Ensure mtvAxis always points from 'other' (B) to 'obj' (A)
    math::Vec2 d = obj.transform.position - other.transform.position;
    if (math::dotProduct(d, mtvAxis) < 0)
    {
        mtvAxis = mtvAxis * -1.0f;
    }

    // --- 2. MANIFOLD GENERATION (Sutherland-Hodgman) ---
    struct Edge
    {
        math::Vec2 v1, v2, maxVertex;
    };

    // Helper: Finds the edge most perpendicular to a given normal
    auto GetBestEdge = [](const BoxCorners &corners, const math::Vec2 &normal) -> Edge
    {
        float maxDot = -std::numeric_limits<float>::max();
        int index = 0;
        for (int i = 0; i < 4; i++)
        {
            float dot = math::dotProduct(corners.v[i], normal);
            if (dot > maxDot)
            {
                maxDot = dot;
                index = i;
            }
        }

        math::Vec2 v = corners.v[index];
        math::Vec2 vPrev = corners.v[(index - 1 + 4) % 4];
        math::Vec2 vNext = corners.v[(index + 1) % 4];

        math::Vec2 leftEdge = v - vPrev;
        leftEdge = leftEdge * (1.0f / leftEdge.length());

        math::Vec2 rightEdge = v - vNext;
        rightEdge = rightEdge * (1.0f / rightEdge.length());

        if (math::dotProduct(leftEdge, normal) <= math::dotProduct(rightEdge, normal))
        {
            return {vPrev, v, v};
        }
        else
        {
            return {v, vNext, v};
        }
    };

    // Find incident and reference edges based on MTV
    Edge edgeA = GetBestEdge(cornersA, mtvAxis * -1.0f); // A opposes MTV
    Edge edgeB = GetBestEdge(cornersB, mtvAxis);         // B aligns with MTV

    math::Vec2 edgeADir = edgeA.v2 - edgeA.v1;
    edgeADir = edgeADir * (1.0f / edgeADir.length());

    math::Vec2 edgeBDir = edgeB.v2 - edgeB.v1;
    edgeBDir = edgeBDir * (1.0f / edgeBDir.length());

    Edge refEdge, incEdge;
    bool flip = false;

    // The reference face is the one most perpendicular to the MTV
    if (std::abs(math::dotProduct(edgeADir, mtvAxis)) <= std::abs(math::dotProduct(edgeBDir, mtvAxis)))
    {
        refEdge = edgeA;
        incEdge = edgeB;
        flip = false; // A is reference
    }
    else
    {
        refEdge = edgeB;
        incEdge = edgeA;
        flip = true; // B is reference
    }

    // Reference planes for clipping
    math::Vec2 refDir = refEdge.v2 - refEdge.v1;
    refDir = refDir * (1.0f / refDir.length());
    float offset1 = math::dotProduct(refDir, refEdge.v1);
    float offset2 = math::dotProduct(refDir * -1.0f, refEdge.v2);

    // Helper: Clips a line segment against a plane
    auto ClipSegmentToLine = [](std::vector<math::Vec2> &vOut, const std::vector<math::Vec2> &vIn,
                                const math::Vec2 &normal, float offset)
    {
        vOut.clear();
        if (vIn.size() < 2)
            return;

        float d0 = math::dotProduct(normal, vIn[0]) - offset;
        float d1 = math::dotProduct(normal, vIn[1]) - offset;

        if (d0 <= 0.0f)
            vOut.push_back(vIn[0]);
        if (d1 <= 0.0f)
            vOut.push_back(vIn[1]);

        if (d0 * d1 < 0.0f)
        {
            math::Vec2 interp = vIn[0] + (vIn[1] - vIn[0]) * (d0 / (d0 - d1));
            vOut.push_back(interp);
        }
    };

    std::vector<math::Vec2> incPoints = {incEdge.v1, incEdge.v2};
    std::vector<math::Vec2> clipped1, finalPoints;

    // Clip incident edge against the adjacent side planes of the reference edge
    ClipSegmentToLine(clipped1, incPoints, refDir * -1.0f, -offset1);
    ClipSegmentToLine(finalPoints, clipped1, refDir, -offset2);

    // Find the actual normal of the reference face
    math::Vec2 refNormal = {-refDir.y, refDir.x};

    // Ensure refNormal points in the right direction
    if (flip)
    {
        if (math::dotProduct(refNormal, mtvAxis) < 0)
            refNormal = refNormal * -1.0f;
    }
    else
    {
        if (math::dotProduct(refNormal, mtvAxis) > 0)
            refNormal = refNormal * -1.0f;
    }

    float refOffset = math::dotProduct(refNormal, refEdge.maxVertex);

    // --- 3. STORE CONTACT MANIFOLD ---
    Contact contact;
    contact.bodyA = obj.bodyID;
    contact.bodyB = other.bodyID;
    contact.normal = mtvAxis;
    contact.overlap = minOverlap; // Legacy overlap, kept for safety
    contact.pointCount = 0;

    for (const auto &point : finalPoints)
    {
        // Subtract point projection FROM the offset to get a positive depth 
        float depth = refOffset - math::dotProduct(refNormal, point);

        // If depth is positive, it's penetrating
        if (depth >= 0.0f && contact.pointCount < 2)
        {
            contact.points[contact.pointCount].position = point;
            contact.points[contact.pointCount].depth = depth;
            contact.pointCount++;
        }
    }

    if (contact.pointCount > 0)
    {
        contacts.push_back(contact);
    }
}

// void sas::PhysicsWorld::checkCollisionBoxBox(Body &obj, Body &other) noexcept
// {
//     math::Vec2 axes[4];
//     float rotA = obj.transform.rotation;
//     float rotB = other.transform.rotation;

//     axes[0] = {std::cos(rotA), std::sin(rotA)};
//     axes[1] = {-std::sin(rotA), std::cos(rotA)};
//     axes[2] = {std::cos(rotB), std::sin(rotB)};
//     axes[3] = {-std::sin(rotB), std::cos(rotB)};

//     float minOverlap = std::numeric_limits<float>::max();
//     math::Vec2 mtvAxis;

//     BoxCorners cornersA = GetBoxCorners(obj);
//     BoxCorners cornersB = GetBoxCorners(other);

//     for (int i = 0; i < 4; i++)
//     {
//         math::Vec2 axis = axes[i];

//         auto project = [&](const BoxCorners &corners)
//         {
//             float min = math::dotProduct(corners.v[0], axis);
//             float max = min;
//             for (int j = 1; j < 4; j++)
//             {
//                 float p = math::dotProduct(corners.v[j], axis);
//                 min = std::min(min, p);
//                 max = std::max(max, p);
//             }
//             return std::pair{min, max};
//         };

//         auto [minA, maxA] = project(cornersA);
//         auto [minB, maxB] = project(cornersB);

//         float overlap = std::min(maxA, maxB) - std::max(minA, minB);
//         if (overlap <= 0)
//             return;

//         if (overlap < minOverlap)
//         {
//             minOverlap = overlap;
//             mtvAxis = axis;
//         }
//     }

//     math::Vec2 d = obj.transform.position - other.transform.position;
//     if (math::dotProduct(d, mtvAxis) < 0)
//     {
//         mtvAxis = mtvAxis * -1.0f;
//     }

//     math::Vec2 contactPoint;

//     auto GetDeepestPoint = [&](const BoxCorners &corners, const math::Vec2 &normal)
//     {
//         float minDot = std::numeric_limits<float>::max();
//         math::Vec2 deepest;
//         for (int i = 0; i < 4; i++)
//         {
//             float dot = math::dotProduct(corners.v[i], normal);
//             if (dot < minDot)
//             {
//                 minDot = dot;
//                 deepest = corners.v[i];
//             }
//         }
//         return deepest;
//     };

//     contactPoint = GetDeepestPoint(cornersA, mtvAxis);

//     math::Vec2 rA = contactPoint - obj.transform.position;
//     math::Vec2 rB = contactPoint - other.transform.position;

//     Contact cont;

//     cont.bodyA = obj.bodyID;
//     cont.bodyB = other.bodyID;

//     cont.normal = mtvAxis;
//     cont.overlap = minOverlap;
//     cont.pointCount = 0;

//     contacts.emplace_back(cont);
// }
void sas::PhysicsWorld::checkCollisionCircleBox(Body &circle, Body &box) noexcept
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

    Contact cont;

    cont.bodyA = circle.bodyID;
    cont.bodyB = box.bodyID;

    cont.normal = worldNormal;
    cont.overlap = overlap;
    cont.pointCount = 0;

    contacts.emplace_back(cont);
}

// void sas::PhysicsWorld::resolveCollision(Contact &contact) noexcept
// {
//     Body &obj = bodies[sparse[contact.bodyA]];
//     Body &other = bodies[sparse[contact.bodyB]];

//     math::Vec2 relVel = obj.kinematics.velocity - other.kinematics.velocity;
//     float velAlongNormal = math::dotProduct(relVel, contact.normal);
//     float totalInvMass = obj.kinematics.inverseMass + other.kinematics.inverseMass;

//     if (velAlongNormal < 0)
//     {
//         float e = std::min(obj.kinematics.restituition, other.kinematics.restituition);
//         float j = -(1.0f + e) * velAlongNormal;
//         j /= totalInvMass;

//         math::Vec2 impulse = contact.normal * j;
//         obj.kinematics.velocity = obj.kinematics.velocity + impulse * obj.kinematics.inverseMass;
//         other.kinematics.velocity = other.kinematics.velocity - impulse * other.kinematics.inverseMass;

//         // math::Vec2 r = {1, 2};

//         // float crossN = r * normal;

//         // obj.kinematics.angularVelocity += crossN * (j * obj.kinematics.inverseMass);
//     }
// }

void sas::PhysicsWorld::resolveCollision(Contact &contact) noexcept
{
    Body &obj = bodies[sparse[contact.bodyA]];
    Body &other = bodies[sparse[contact.bodyB]];

    float totalInvMass = obj.kinematics.inverseMass + other.kinematics.inverseMass;
    
    // Early exit if both objects are static/have infinite mass
    if (totalInvMass <= 0.0f) return;

    for (uint32_t i = 0; i < contact.pointCount; i++)
    {
        // 1. Calculate relative velocity (Linear only)
        math::Vec2 relVel = obj.kinematics.velocity - other.kinematics.velocity;
        float velAlongNormal = math::dotProduct(relVel, contact.normal);

        // 2. If objects are already moving apart, skip this contact point
        if (velAlongNormal >= 0.0f) continue; 

        // 3. Calculate Restitution (Bounce)
        float e = std::min(obj.kinematics.restituition, other.kinematics.restituition);
        
        if (std::abs(velAlongNormal) < 10.0f) 
        {
            e = 0.0f; 
        }

        // 4. Calculate Impulse Scalar (j)
        float j = -(1.0f + e) * velAlongNormal;
        j /= totalInvMass;

        // DO NOT divide j by contact.pointCount here! 
        // In a sequential impulse solver, velocities update immediately, 
        // so the math naturally handles the distribution across points.

        // 5. Apply Linear Impulse
        math::Vec2 impulse = contact.normal * j;
        
        obj.kinematics.velocity = obj.kinematics.velocity + impulse * obj.kinematics.inverseMass;
        other.kinematics.velocity = other.kinematics.velocity - impulse * other.kinematics.inverseMass;
    }
}
#if 0
void sas::PhysicsWorld::correctPosition(Contact &contact) noexcept
{
    Body &obj = bodies[sparse[contact.bodyA]];
    Body &other = bodies[sparse[contact.bodyB]];

    float totalInvMass = obj.kinematics.inverseMass + other.kinematics.inverseMass;

    if (totalInvMass <= 0.0f)
        return;

    const float percent = 0.8f;
    const float slop = 0.01f;

    float penetration = std::max(contact.overlap - slop, 0.0f);

    math::Vec2 correction = contact.normal * (penetration / totalInvMass) * percent;

    obj.transform.position = obj.transform.position + correction * obj.kinematics.inverseMass;
    other.transform.position = other.transform.position - correction * other.kinematics.inverseMass;
}
#else
void sas::PhysicsWorld::correctPosition(Contact &contact) noexcept
{
    Body &obj = bodies[sparse[contact.bodyA]];
    Body &other = bodies[sparse[contact.bodyB]];

    float totalInvMass = obj.kinematics.inverseMass + other.kinematics.inverseMass;
    
    if (totalInvMass <= 0.0f) return;

    const float percent = 0.2f; 
    
    const float slop = 0.5f;   

    for (uint32_t i = 0; i < contact.pointCount; i++)
    {
        float penetration = contact.points[i].depth - slop;
        
        
        if (penetration <= 0.0f) continue; 

        
        float p = (penetration / totalInvMass) * percent;
        
        p *= contact.pointCount; 

        math::Vec2 correction = contact.normal * p;

        obj.transform.position = obj.transform.position + correction * obj.kinematics.inverseMass;
        other.transform.position = other.transform.position - correction * other.kinematics.inverseMass;
    }
}
#endif

void sas::PhysicsWorld::checkCollisionBoxCircle(Body &obj, Body &other) noexcept
{
    checkCollisionCircleBox(other, obj);
}

void sas::PhysicsWorld::updateCollisionFlags() noexcept
{
    std::fill(collisionFlags.begin(), collisionFlags.end(), 0);

    for (const auto &contact : contacts)
    {
        collisionFlags[contact.bodyA] = 1;
        collisionFlags[contact.bodyB] = 1;
    }
}

void sas::PhysicsWorld::applyForces(Body &obj) const noexcept
{
    float dragForceY = obj.kinematics.velocity.y * settings.dragCoeff;
    obj.kinematics.acceleration.y += settings.gravity - (dragForceY * obj.kinematics.inverseMass);

    float dragForceX = obj.kinematics.velocity.x * settings.dragCoeff;
    obj.kinematics.acceleration.x += -1 * (dragForceX * obj.kinematics.inverseMass);
}

void sas::PhysicsWorld::integrateVelocity(Body &obj) const noexcept
{
    obj.kinematics.velocity = obj.kinematics.velocity + obj.kinematics.acceleration * deltaTime;
}

void sas::PhysicsWorld::integratePosition(Body &obj) const noexcept
{
    obj.transform.position = obj.transform.position + obj.kinematics.velocity * deltaTime;
    obj.transform.rotation = obj.transform.rotation + obj.kinematics.angularVelocity * deltaTime;
}

void sas::PhysicsWorld::addToCollisionPool(Body &body) noexcept
{
    if (!(body.flags & Flags::InCollisionPool))
    {
        body.flags |= Flags::InCollisionPool;

        root.insert(body.bodyID, computeFatAABB(body));
    }
}

void sas::PhysicsWorld::removeFromCollisionPool(Body &body) noexcept
{
    if (body.flags & Flags::InCollisionPool)
    {
        body.flags &= ~Flags::InCollisionPool;

        root.remove(body.bodyID);
    }
}

void sas::PhysicsWorld::reset(Body &obj) const noexcept
{
    obj.kinematics.acceleration = {0, 0};
}

// TODO
bool sas::PhysicsWorld::bodyExists(uint32_t id) const noexcept
{
    return false;
}

bool sas::PhysicsWorld::isBodyInCollision(uint32_t id) const noexcept
{
    return collisionFlags[id] != 0;
}

sas::Body &sas::PhysicsWorld::getBody(uint32_t id) noexcept
{
    return bodies[sparse[id]];
}

std::vector<sas::CollisionInfo> sas::PhysicsWorld::getAllCollisions(uint32_t id) noexcept
{
    if (!isBodyInCollision(id))
        return {};

    std::vector<sas::CollisionInfo> collisions;

    collisions.reserve(8);

    for (const auto &contact : contacts)
    {
        if (contact.bodyA == id)
        {
            collisions.emplace_back(BodyHandle{contact.bodyB, this}, contact.normal, contact.overlap);
        }
        else if (contact.bodyB == id)
        {
            collisions.emplace_back(BodyHandle{contact.bodyA, this}, -1 * contact.normal, contact.overlap);
        }
    }

    return collisions;
}

void sas::PhysicsWorld::removeBody(const BodyHandle &handle) noexcept
{
    removeBody(handle.get()->bodyID);
}

void sas::PhysicsWorld::drawDebug(const DrawCallback &cb) const noexcept
{
    root.draw(cb);
}

void sas::PhysicsWorld::clear() noexcept
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