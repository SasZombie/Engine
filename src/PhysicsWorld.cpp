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

    uint32_t mask = Flags::RigidBodyDynamic | Flags::RigidBodyStatic | Flags::RigidBodyKinematic;

    if ((options & Flags::Active) && (options & mask))
    {
        newBody.collisionMask = Flags::Layer1 | Flags::Mask1;

        activeIDs.push_back(newID);
        addToCollisionPool(newBody);
    }

    if (options & Flags::BodyFlags::RigidBodyDynamic)
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
    else if ((options & Flags::BodyFlags::RigidBodyStatic) || (options & Flags::BodyFlags::RigidBodyKinematic))
    {
        newBody.kinematics.inverseMass = 0.f;
        newBody.kinematics.inverseInertia = 0.f;
        newBody.kinematics.inertia = 0.f;
        newBody.kinematics.pushForce = 0.f;
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

static constexpr float physicsDt = 1.0f / 120.0f;

void sas::PhysicsWorld::step(float realDeltaTime) noexcept
{
    if (realDeltaTime > 0.25f)
        realDeltaTime = 0.25f;

    timeAccumulator += realDeltaTime;

    deltaTime = physicsDt;

    while (timeAccumulator >= physicsDt)
    {
        step();
        timeAccumulator -= physicsDt;
    }
}

// MARK: Step
void sas::PhysicsWorld::step() noexcept
{
    contacts.clear();

    for (uint32_t id : activeIDs)
    {
        Body &obj = bodies[sparse[id]];

        if (!(obj.flags & (Flags::RigidBodyStatic | Flags::RigidBodyKinematic)))
        {
            if (obj.kinematics.inverseMass > 0)
            {
                applyForces(obj);
                integrateVelocity(obj);
            }
        }
    }

    for (uint32_t id : activeIDs)
    {
        Body &obj = bodies[sparse[id]];

        if (!(obj.flags & Flags::RigidBodyStatic))
        {

            integratePosition(obj);
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

    for (auto &c : contacts)
    {
        correctPosition(c);
    }

    for (uint32_t id : activeIDs)
    {
        Body &obj = bodies[sparse[id]];
        reset(obj);
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

    float radiusA = obj.shape.radius * obj.transform.scale.x;
    float radiusB = other.shape.radius * other.transform.scale.x;
    float combinedRad = radiusA + radiusB;

    if (distanceSq > combinedRad * combinedRad)
        return;

    float distance = std::sqrt(distanceSq);

    math::Vec2 normal = (distance > 0.0001f)
                            ? math::Vec2(dx / distance, dy / distance)
                            : math::Vec2(0.0f, 1.0f);

    float overlap = combinedRad - distance;

    math::Vec2 contactPoint = {
        other.transform.position.x + normal.x * radiusB,
        other.transform.position.y + normal.y * radiusB};

    Contact cont;

    cont.bodyA = obj.bodyID;
    cont.bodyB = other.bodyID;
    cont.normal = normal;
    cont.overlap = overlap;

    cont.pointCount = 1;
    cont.points[0].position = contactPoint;
    cont.points[0].depth = overlap;

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

    struct Edge
    {
        math::Vec2 v1, v2, maxVertex;
    };

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

    Edge edgeA = GetBestEdge(cornersA, mtvAxis * -1.0f);
    Edge edgeB = GetBestEdge(cornersB, mtvAxis);

    math::Vec2 edgeADir = edgeA.v2 - edgeA.v1;
    edgeADir = edgeADir * (1.0f / edgeADir.length());

    math::Vec2 edgeBDir = edgeB.v2 - edgeB.v1;
    edgeBDir = edgeBDir * (1.0f / edgeBDir.length());

    Edge refEdge, incEdge;
    bool flip = false;

    if (std::abs(math::dotProduct(edgeADir, mtvAxis)) <= std::abs(math::dotProduct(edgeBDir, mtvAxis)))
    {
        refEdge = edgeA;
        incEdge = edgeB;
        flip = false;
    }
    else
    {
        refEdge = edgeB;
        incEdge = edgeA;
        flip = true;
    }

    math::Vec2 refDir = refEdge.v2 - refEdge.v1;
    refDir = refDir * (1.0f / refDir.length());
    float offset1 = math::dotProduct(refDir, refEdge.v1);
    float offset2 = math::dotProduct(refDir * -1.0f, refEdge.v2);

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

    ClipSegmentToLine(clipped1, incPoints, refDir * -1.0f, -offset1);
    ClipSegmentToLine(finalPoints, clipped1, refDir, -offset2);

    math::Vec2 refNormal = {-refDir.y, refDir.x};

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

    Contact contact;
    contact.bodyA = obj.bodyID;
    contact.bodyB = other.bodyID;
    contact.normal = mtvAxis;
    contact.overlap = minOverlap;
    contact.pointCount = 0;

    for (const auto &point : finalPoints)
    {
        float depth = refOffset - math::dotProduct(refNormal, point);

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

    math::Vec2 localContactPoint = closest;

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
            localContactPoint.x = (localPos.x > 0) ? hx : -hx;
        }
        else
        {
            localNormal = (localPos.y > 0) ? math::Vec2(0, 1) : math::Vec2(0, -1);
            overlap = r + py;
            localContactPoint.y = (localPos.y > 0) ? hy : -hy;
        }
    }

    float cosW = std::cos(box.transform.rotation);
    float sinW = std::sin(box.transform.rotation);

    math::Vec2 worldNormal = {
        localNormal.x * cosW - localNormal.y * sinW,
        localNormal.x * sinW + localNormal.y * cosW};

    math::Vec2 worldContactPoint = {
        localContactPoint.x * cosW - localContactPoint.y * sinW + box.transform.position.x,
        localContactPoint.x * sinW + localContactPoint.y * cosW + box.transform.position.y};

    Contact cont;
    cont.bodyA = circle.bodyID;
    cont.bodyB = box.bodyID;

    cont.normal = worldNormal;
    cont.overlap = overlap;

    cont.pointCount = 1;
    cont.points[0].position = worldContactPoint;
    cont.points[0].depth = overlap;

    contacts.emplace_back(cont);
}

void sas::PhysicsWorld::resolveCollision(Contact &contact) noexcept
{
    Body &obj = bodies[sparse[contact.bodyA]];
    Body &other = bodies[sparse[contact.bodyB]];

    float totalInvMass = obj.kinematics.inverseMass + other.kinematics.inverseMass;
    if (totalInvMass <= 0.0f)
        return;

    auto crossV = [](const math::Vec2 &a, const math::Vec2 &b)
    { return a.x * b.y - a.y * b.x; };
    auto crossZ = [](float w, const math::Vec2 &r)
    { return math::Vec2{-w * r.y, w * r.x}; };

    float maxVelAlongNormal = 0.0f;
    for (uint32_t i = 0; i < contact.pointCount; i++)
    {
        math::Vec2 rA = contact.points[i].position - obj.transform.position;
        math::Vec2 rB = contact.points[i].position - other.transform.position;

        math::Vec2 velA = obj.kinematics.velocity + crossZ(obj.kinematics.angularVelocity, rA);
        math::Vec2 velB = other.kinematics.velocity + crossZ(other.kinematics.angularVelocity, rB);

        float vel = math::dotProduct(velA - velB, contact.normal);
        if (vel < maxVelAlongNormal)
            maxVelAlongNormal = vel;
    }

    if (maxVelAlongNormal < -10.0f)
    {
        math::Vec2 averagePoint = {0.0f, 0.0f};
        for (uint32_t i = 0; i < contact.pointCount; i++)
        {
            averagePoint = averagePoint + contact.points[i].position;
        }
        averagePoint = averagePoint * (1.0f / contact.pointCount);

        math::Vec2 rA = averagePoint - obj.transform.position;
        math::Vec2 rB = averagePoint - other.transform.position;

        math::Vec2 velA = obj.kinematics.velocity + crossZ(obj.kinematics.angularVelocity, rA);
        math::Vec2 velB = other.kinematics.velocity + crossZ(other.kinematics.angularVelocity, rB);
        float velAlongNormal = math::dotProduct(velA - velB, contact.normal);

        if (velAlongNormal >= 0.0f)
            return;

        float raCrossN = crossV(rA, contact.normal);
        float rbCrossN = crossV(rB, contact.normal);

        float invMassSum = obj.kinematics.inverseMass + other.kinematics.inverseMass +
                           (raCrossN * raCrossN) * obj.kinematics.inverseInertia +
                           (rbCrossN * rbCrossN) * other.kinematics.inverseInertia;

        float e = std::max(obj.kinematics.restituition, other.kinematics.restituition);
        if (obj.kinematics.inverseMass > 0.0f && other.kinematics.inverseMass > 0.0f)
        {
            e = std::min(e * 1.2f, 1.0f);
        }

        float j = -(1.0f + e) * velAlongNormal / invMassSum;
        bool aIsKinematic = (obj.flags & Flags::RigidBodyKinematic);
        bool bIsKinematic = (other.flags & Flags::RigidBodyKinematic);

        if (aIsKinematic || bIsKinematic)
        {
            const Body &pusher = aIsKinematic ? obj : other;
            const Body &pushed = aIsKinematic ? obj : other;

            float dynamicMass = (pushed.kinematics.inverseMass > 0.0f)
                                    ? 1.0f / pushed.kinematics.inverseMass
                                    : 1.0f;

            float forceScale = pusher.kinematics.pushForce / (pusher.kinematics.pushForce + dynamicMass);

            j *= forceScale;
        }

        math::Vec2 impulse = contact.normal * j;

        obj.kinematics.velocity = obj.kinematics.velocity + impulse * obj.kinematics.inverseMass;
        obj.kinematics.angularVelocity += raCrossN * j * obj.kinematics.inverseInertia;
        other.kinematics.velocity = other.kinematics.velocity - impulse * other.kinematics.inverseMass;
        other.kinematics.angularVelocity -= rbCrossN * j * other.kinematics.inverseInertia;

        return;
    }

    for (uint32_t i = 0; i < contact.pointCount; i++)
    {
        math::Vec2 rA = contact.points[i].position - obj.transform.position;
        math::Vec2 rB = contact.points[i].position - other.transform.position;

        math::Vec2 velA = obj.kinematics.velocity + crossZ(obj.kinematics.angularVelocity, rA);
        math::Vec2 velB = other.kinematics.velocity + crossZ(other.kinematics.angularVelocity, rB);

        float velAlongNormal = math::dotProduct(velA - velB, contact.normal);

        if (velAlongNormal >= 0.0f)
            continue;

        float raCrossN = crossV(rA, contact.normal);
        float rbCrossN = crossV(rB, contact.normal);

        float invMassSum = obj.kinematics.inverseMass + other.kinematics.inverseMass +
                           (raCrossN * raCrossN) * obj.kinematics.inverseInertia +
                           (rbCrossN * rbCrossN) * other.kinematics.inverseInertia;

        if (invMassSum <= 0.0f)
            continue;

        float j = -1.0f * velAlongNormal / invMassSum;

        math::Vec2 impulse = contact.normal * j;

        obj.kinematics.velocity = obj.kinematics.velocity + impulse * obj.kinematics.inverseMass;
        obj.kinematics.angularVelocity += raCrossN * j * obj.kinematics.inverseInertia;

        other.kinematics.velocity = other.kinematics.velocity - impulse * other.kinematics.inverseMass;
        other.kinematics.angularVelocity -= rbCrossN * j * other.kinematics.inverseInertia;
    }
}

void sas::PhysicsWorld::correctPosition(Contact &contact) noexcept
{
    Body &obj = bodies[sparse[contact.bodyA]];
    Body &other = bodies[sparse[contact.bodyB]];

    const float percent = 0.4f;
    const float slop = 0.5f;

    float invMassA = (obj.flags & Flags::RigidBodyKinematic) ? 1.0f : obj.kinematics.inverseMass;
    float invInertiaA = (obj.flags & Flags::RigidBodyKinematic) ? 0.0f : obj.kinematics.inverseInertia;

    float invMassB = (other.flags & Flags::RigidBodyKinematic) ? 1.0f : other.kinematics.inverseMass;
    float invInertiaB = (other.flags & Flags::RigidBodyKinematic) ? 0.0f : other.kinematics.inverseInertia;

    for (uint32_t i = 0; i < contact.pointCount; i++)
    {
        math::Vec2 rA = contact.points[i].position - obj.transform.position;
        math::Vec2 rB = contact.points[i].position - other.transform.position;

        auto crossV = [](const math::Vec2 &a, const math::Vec2 &b)
        { return a.x * b.y - a.y * b.x; };

        float raCrossN = crossV(rA, contact.normal);
        float rbCrossN = crossV(rB, contact.normal);

        float invMassSum = invMassA + invMassB +
                           (raCrossN * raCrossN) * invInertiaA +
                           (rbCrossN * rbCrossN) * invInertiaB;

        if (invMassSum <= 0.0f)
            continue;

        float penetration = contact.points[i].depth - slop;
        if (penetration <= 0.0f)
            continue;

        float p = (penetration / invMassSum) * percent;
        p /= contact.pointCount;

        math::Vec2 correction = contact.normal * p;

        obj.transform.position = obj.transform.position + correction * invMassA;
        obj.transform.rotation += raCrossN * p * invInertiaA;

        other.transform.position = other.transform.position - correction * invMassB;
        other.transform.rotation -= rbCrossN * p * invInertiaB;
    }
}

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

    const float angularDamping = 0.98f;

    // We raise it to the power of (deltaTime * 60) so it is framerate independent
    float dampingThisFrame = std::pow(angularDamping, deltaTime * 60.0f);

    obj.kinematics.angularVelocity *= dampingThisFrame;
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