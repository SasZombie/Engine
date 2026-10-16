#include "Traits.hpp"
#include "PhysicsSettings.hpp"
#include <algorithm>
#include <utility>
#include "Logger.hpp"

void sas::Traits2D::initializeBodyPhysics(Body &body) noexcept
{
    if (body.hasFlag(Flags::RigidBodyDynamic))
    {
        const float mass = (body.kinematics.inverseMass > 0) ? 1.0f / body.kinematics.inverseMass : 0.0f;

        switch (body.shape.type)
        {
        case ShapeType2D::Box:
        {
            float hx = body.shape.halfSize.x * body.transform.scale.x;
            float hy = body.shape.halfSize.y * body.transform.scale.y;
            body.kinematics.inertia = (1.0f / 3.0f) * mass * (hx * hx + hy * hy);
            break;
        }
        case ShapeType2D::Circle:
        {
            float r = body.shape.radius * body.transform.scale.x;
            body.kinematics.inertia = 0.5f * mass * (r * r);
            break;
        }
        default:
            body.kinematics.inertia = 0.f;
            break;
        }

        body.kinematics.inverseInertia = (body.kinematics.inertia > 0) ? 1.0f / body.kinematics.inertia : 0.0f;

        return;
    }

    body.kinematics.inverseMass = 0.0f;
    body.kinematics.inertia = 0.0f;
    body.kinematics.inverseInertia = 0.0f;

    if (body.hasFlag(Flags::BodyFlags::Trigger))
    {
        body.kinematics.pushForce = 0.f;
    }
    else
    {
        body.kinematics.pushForce = body.kinematics.pushForce > 0.0f ? body.kinematics.pushForce : 70.0f;
    }

    if (body.hasFlag(Flags::BodyFlags::RigidBodyStatic))
    {
        body.kinematics.velocity = {0.0f, 0.0f};
        body.kinematics.angularVelocity = 0.0f;
        body.kinematics.pushForce = 0.0f;
    }
}

void sas::Traits2D::applyForces(Body &obj, const PhysicsSettings &settings) noexcept
{
    float dragForceY = obj.kinematics.velocity.y * settings.dragCoeff;
    obj.kinematics.acceleration.y += (obj.gravityScale * settings.gravity.y) - (dragForceY * obj.kinematics.inverseMass);

    float dragForceX = obj.kinematics.velocity.x * settings.dragCoeff;
    obj.kinematics.acceleration.x += (obj.gravityScale * settings.gravity.x) - (dragForceX * obj.kinematics.inverseMass);
}

void sas::Traits2D::integrateVelocity(Body &obj, float dt) noexcept
{
    obj.kinematics.velocity = obj.kinematics.velocity + obj.kinematics.acceleration * dt;

    const float angularDamping = 0.98f;

    // We raise it to the power of (deltaTime * 60) so it is framerate independent
    float dampingThisFrame = std::pow(angularDamping, dt * 60.0f);

    obj.kinematics.angularVelocity *= dampingThisFrame;
}

void sas::Traits2D::integratePosition(Body &obj, float dt) noexcept
{
    obj.transform.position = obj.transform.position + obj.kinematics.velocity * dt;
    obj.transform.rotation = obj.transform.rotation + obj.kinematics.angularVelocity * dt;
}

void sas::Traits2D::resolveCollision(Body &obj, Body &other, Contact &contact) noexcept
{
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

        float e = std::max(obj.kinematics.restitution, other.kinematics.restitution);
        if (obj.kinematics.inverseMass > 0.0f && other.kinematics.inverseMass > 0.0f)
        {
            e = std::min(e * 1.2f, 1.0f);
        }

        float j = -(1.0f + e) * velAlongNormal / invMassSum;
        bool aIsKinematic = (obj.flags & Flags::RigidBodyKinematic);
        bool bIsKinematic = (other.flags & Flags::RigidBodyKinematic);

        if (aIsKinematic || bIsKinematic)
        {
            const Body2D &pusher = aIsKinematic ? obj : other;
            const Body2D &pushed = aIsKinematic ? obj : other;

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
    }
}

void sas::Traits2D::correctPosition(Body &obj, Body &other, Contact &contact) noexcept
{
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

bool sas::Traits2D::testCircleCircle(const Body &obj, const Body &other, Contact &outContact) noexcept
{
    float dx = obj.transform.position.x - other.transform.position.x;
    float dy = obj.transform.position.y - other.transform.position.y;
    float distanceSq = dx * dx + dy * dy;

    float radiusA = obj.shape.radius * obj.transform.scale.x;
    float radiusB = other.shape.radius * other.transform.scale.x;
    float combinedRad = radiusA + radiusB;

    if (distanceSq > combinedRad * combinedRad)
        return false;

    float distance = std::sqrt(distanceSq);

    math::Vec2 normal = (distance > 0.0001f)
                            ? math::Vec2(dx / distance, dy / distance)
                            : math::Vec2(0.0f, 1.0f);

    float overlap = combinedRad - distance;

    math::Vec2 contactPoint = {
        other.transform.position.x + normal.x * radiusB,
        other.transform.position.y + normal.y * radiusB};

    outContact.bodyA = obj.bodyID;
    outContact.bodyB = other.bodyID;
    outContact.normal = normal;
    outContact.overlap = overlap;

    outContact.pointCount = 1;
    outContact.points[0].position = contactPoint;
    outContact.points[0].depth = overlap;

    return true;
}

bool sas::Traits2D::testCircleBox(const Body &circle, const Body &box, Contact &outContact) noexcept
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
        return false;

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

    outContact.bodyA = circle.bodyID;
    outContact.bodyB = box.bodyID;
    outContact.normal = worldNormal;
    outContact.overlap = overlap;
    outContact.pointCount = 1;
    outContact.points[0].position = worldContactPoint;
    outContact.points[0].depth = overlap;

    return true;
}

struct BoxCorners
{
    sas::math::Vec2 v[4];
};

static BoxCorners GetBoxCorners(const sas::Body2D &b)
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

bool sas::Traits2D::testBoxBox(const Body &obj, const Body &other, Contact &outContact) noexcept
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
            return false;

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

    auto getBestEdge = [](const BoxCorners &corners, const math::Vec2 &normal) -> Edge
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

    Edge edgeA = getBestEdge(cornersA, mtvAxis * -1.0f);
    Edge edgeB = getBestEdge(cornersB, mtvAxis);

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

    auto clipSegmentToLine = [](std::vector<math::Vec2> &vOut, const std::vector<math::Vec2> &vIn,
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

    clipSegmentToLine(clipped1, incPoints, refDir * -1.0f, -offset1);
    clipSegmentToLine(finalPoints, clipped1, refDir, -offset2);

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

    outContact.bodyA = obj.bodyID;
    outContact.bodyB = other.bodyID;
    outContact.normal = mtvAxis;
    outContact.overlap = minOverlap;
    outContact.pointCount = 0;

    for (const auto &point : finalPoints)
    {
        float depth = refOffset - math::dotProduct(refNormal, point);

        if (depth >= 0.0f && outContact.pointCount < 2)
        {
            outContact.points[outContact.pointCount].position = point;
            outContact.points[outContact.pointCount].depth = depth;
            outContact.pointCount++;
        }
    }

    if (outContact.pointCount == 0)
    {
        return false;
    }
    return true;
}

void sas::Traits3D::initializeBodyPhysics(Body &body) noexcept
{
    const float mass = (body.kinematics.inverseMass > 0) ? 1.0f / body.kinematics.inverseMass : 0.0f;
    const float inversMass = body.kinematics.inverseMass;
    math::Mat3 inverseInertiaTensorLocal;

    if (body.hasFlag(Flags::RigidBodyDynamic))
    {

        switch (body.shape.type)
        {
        case ShapeType3D::Orthotope:
        {
            float c = (1.0f / 3.0f) * mass;

            float Ixx = c * (body.shape.halfSize.y * body.shape.halfSize.y + body.shape.halfSize.z * body.shape.halfSize.z);
            float Iyy = c * (body.shape.halfSize.x * body.shape.halfSize.x + body.shape.halfSize.z * body.shape.halfSize.z);
            float Izz = c * (body.shape.halfSize.x * body.shape.halfSize.x + body.shape.halfSize.y * body.shape.halfSize.y);

            inverseInertiaTensorLocal(0, 0) = 1.0f / Ixx;
            inverseInertiaTensorLocal(1, 1) = 1.0f / Iyy;
            inverseInertiaTensorLocal(2, 2) = 1.0f / Izz;

            break;
        }
        case ShapeType3D::Sphere:
        {
            float invI = (2.5f * inversMass) / (body.shape.radius * body.shape.radius);

            inverseInertiaTensorLocal(0, 0) = invI;
            inverseInertiaTensorLocal(1, 1) = invI;
            inverseInertiaTensorLocal(2, 2) = invI;

            break;
        }
        case ShapeType3D::Cilinder:
        {

            float invSide = inversMass / (0.25f * body.shape.radHeight.x * body.shape.radHeight.x + (1.0f / 3.0f) * body.shape.radHeight * body.shape.radHeight);
            float invAxial = (2.0f * inversMass) / (body.shape.radHeight.x * body.shape.radHeight.x);

            inverseInertiaTensorLocal(0, 0) = invSide;
            inverseInertiaTensorLocal(1, 1) = invAxial;
            inverseInertiaTensorLocal(2, 2) = invSide;
            break;
        }
        default:

            BaseLogger::error(std::string("Trying to create an unimplemented Shape ") + enumToString(body.shape.type));
            std::unreachable();
        }

        body.kinematics.inverseInertiaTensorLocal = inverseInertiaTensorLocal;
        return;
    }

    body.kinematics.inverseMass = 0.0f;
    body.kinematics.inverseInertiaTensorLocal = inverseInertiaTensorLocal;

    if (body.hasFlag(Flags::BodyFlags::Trigger))
    {
        body.kinematics.pushForce = 0.f;
    }
    else
    {
        body.kinematics.pushForce = math::floatAlmostEqual(body.kinematics.pushForce, 0) ? body.kinematics.pushForce : 0.5f;
    }

    if (body.hasFlag(Flags::BodyFlags::RigidBodyStatic))
    {
        body.kinematics.velocity = math::Vec3{0.f};
        body.kinematics.angularVelocity = math::Vec3{0.f};
    }
}

void sas::Traits3D::applyForces(Body &obj, const PhysicsSettings &settings) noexcept
{
    math::Vec3 gravityAccel = settings.gravity * obj.gravityScale;
    math::Vec3 dragAccel = obj.kinematics.velocity * (settings.dragCoeff * obj.kinematics.inverseMass);

    obj.kinematics.acceleration += gravityAccel - dragAccel;

    // TODO:
    //  math::Vec3 angularDragTorque = obj.kinematics.angularVelocity * settings.angularDragCoeff;
    //  math::Vec3 angularDragAccel = obj.kinematics.inverseInertiaTensorWorld * angularDragTorque;

    // obj.kinematics.angularVelocity -= angularDragAccel * deltaTime;
}

void sas::Traits3D::integrateVelocity(Body &obj, float dt) noexcept
{
    obj.kinematics.velocity = obj.kinematics.velocity + obj.kinematics.acceleration * dt;
}

void sas::Traits3D::integratePosition(Body &obj, float dt) noexcept
{
    obj.transform.position += obj.kinematics.velocity * dt;

    const math::Vec3 w = obj.kinematics.angularVelocity;
    const float wLenSq = w.lengthSq();

    if (wLenSq > 1e-8f)
    {
        const float wSpeed = std::sqrt(wLenSq);
        const float angle = wSpeed * dt;

        math::Vec3 axis = w * (1.0f / wSpeed);
        math::Quaterion deltaQ(axis, angle);

        obj.transform.quaterion = (deltaQ * obj.transform.quaterion).norm();
        obj.transform.rotMatrix = obj.transform.quaterion.toMat4();

        math::Mat3 R = obj.transform.quaterion.toMat3();
        obj.kinematics.inverseInertiaTensorWorld = R * obj.kinematics.inverseInertiaTensorLocal * R.transposed();
    }
}

void sas::Traits3D::resolveCollision(Body &obj, Body &other, Contact &contact) noexcept
{
    float totalInvMass = obj.kinematics.inverseMass + other.kinematics.inverseMass;
    if (totalInvMass <= 0.0f)
        return;

    for (uint32_t i = 0; i < contact.pointCount; ++i)
    {
        math::Vec3 contactPoint = contact.points[i].position;

        math::Vec3 rA = contactPoint - obj.transform.position;
        math::Vec3 rB = contactPoint - other.transform.position;

        math::Vec3 velA = obj.kinematics.velocity + obj.kinematics.angularVelocity * rA;
        math::Vec3 velB = other.kinematics.velocity + other.kinematics.angularVelocity * rB;

        math::Vec3 relativeVel = velA - velB;
        float velAlongNormal = math::dotProduct(relativeVel, contact.normal);

        if (velAlongNormal >= 0.0f)
            continue;

        math::Vec3 raCrossN = rA * contact.normal;
        math::Vec3 rbCrossN = rB * contact.normal;

        math::Vec3 angInertiaA = obj.kinematics.inverseInertiaTensorWorld * raCrossN;
        math::Vec3 angInertiaB = other.kinematics.inverseInertiaTensorWorld * rbCrossN;

        float angularFactorA = math::dotProduct(raCrossN, angInertiaA);
        float angularFactorB = math::dotProduct(rbCrossN, angInertiaB);

        float invMassSum = totalInvMass + angularFactorA + angularFactorB;
        if (invMassSum <= 0.0f)
            continue;

        float e = std::max(obj.kinematics.restitution, other.kinematics.restitution);

        float j = -(1.0f + e) * velAlongNormal / invMassSum;

        math::Vec3 impulse = contact.normal * j;

        obj.kinematics.velocity += impulse * obj.kinematics.inverseMass;
        other.kinematics.velocity -= impulse * other.kinematics.inverseMass;

        obj.kinematics.angularVelocity += obj.kinematics.inverseInertiaTensorWorld * (raCrossN * j);
        other.kinematics.angularVelocity -= other.kinematics.inverseInertiaTensorWorld * (rbCrossN * j);
    }
}

void sas::Traits3D::correctPosition(Body &obj, Body &other, Contact &contact) noexcept
{
    float totalInvMass = obj.kinematics.inverseMass + other.kinematics.inverseMass;
    if (totalInvMass <= 0.0f)
        return;

    const float percent = 0.4f;
    const float slop = 0.01f;
    float penetration = std::max(contact.overlap - slop, 0.0f);

    math::Vec3 correction = contact.normal * (penetration / totalInvMass * percent);

    obj.transform.position = obj.transform.position + correction * obj.kinematics.inverseMass;
    other.transform.position = other.transform.position - correction * other.kinematics.inverseMass;
}

bool sas::Traits3D::SphereSphere(const Body &obj, const Body &other, Contact &outContact) noexcept
{
    float dx = obj.transform.position.x - other.transform.position.x;
    float dy = obj.transform.position.y - other.transform.position.y;
    float dz = obj.transform.position.z - other.transform.position.z;
    float distanceSq = dx * dx + dy * dy + dz * dz;

    float radiusA = obj.shape.radius * obj.transform.scale.x;
    float radiusB = other.shape.radius * other.transform.scale.x;
    float combinedRad = radiusA + radiusB;

    if (distanceSq > combinedRad * combinedRad)
        return false;

    float distance = std::sqrt(distanceSq);

    math::Vec3 normal = (distance > 0.0001f)
                            ? math::Vec3(dx / distance, dy / distance, dz / distance)
                            : math::Vec3(0.0f, 1.0f, 0.f);

    float overlap = combinedRad - distance;

    math::Vec3 contactPoint = obj.transform.position + normal * (radiusA - overlap * 0.5f);

    outContact.bodyA = obj.bodyID;
    outContact.bodyB = other.bodyID;
    outContact.normal = normal;
    outContact.overlap = overlap;

    outContact.pointCount = 1;
    outContact.points[0].position = contactPoint;
    outContact.points[0].depth = overlap;

    return true;
}

bool sas::Traits3D::SphereOrthotope(const Body &sphereBody, const Body &boxBody, Contact &outContact) noexcept
{
    float sphereRadius = sphereBody.shape.radius * sphereBody.transform.scale.x;

    math::Vec3 boxHalfExtents = {
        boxBody.shape.halfSize.x * boxBody.transform.scale.x,
        boxBody.shape.halfSize.y * boxBody.transform.scale.y,
        boxBody.shape.halfSize.z * boxBody.transform.scale.z};

    math::Vec3 deltaWorld = sphereBody.transform.position - boxBody.transform.position;

    math::Mat3 boxRot = boxBody.transform.quaterion.toMat3();
    math::Mat3 invBoxRot = boxRot.transposed();
    math::Vec3 localSpherePos = invBoxRot * deltaWorld;

    math::Vec3 closestLocal = {
        std::clamp(localSpherePos.x, -boxHalfExtents.x, boxHalfExtents.x),
        std::clamp(localSpherePos.y, -boxHalfExtents.y, boxHalfExtents.y),
        std::clamp(localSpherePos.z, -boxHalfExtents.z, boxHalfExtents.z)};

    math::Vec3 localDiff = localSpherePos - closestLocal;
    float distSq = localDiff.x * localDiff.x + localDiff.y * localDiff.y + localDiff.z * localDiff.z;

    math::Vec3 localNormal;
    float overlap = 0.0f;

    if (distSq > 1e-8f)
    {
        if (distSq > sphereRadius * sphereRadius)
            return false;

        float dist = std::sqrt(distSq);
        localNormal = localDiff * (1.0f / dist);
        overlap = sphereRadius - dist;
    }
    else
    {
        const float distToPositiveX = boxHalfExtents.x - localSpherePos.x;
        const float distToNegativeX = boxHalfExtents.x + localSpherePos.x;
        const float distToPositiveY = boxHalfExtents.y - localSpherePos.y;
        const float distToNegativeY = boxHalfExtents.y + localSpherePos.y;
        const float distToPositiveZ = boxHalfExtents.z - localSpherePos.z;
        const float distToNegativeZ = boxHalfExtents.z + localSpherePos.z;

        float minDist = distToPositiveX;
        localNormal = math::Vec3(1.0f, 0.0f, 0.0f);

        if (distToNegativeX < minDist)
        {
            minDist = distToNegativeX;
            localNormal = math::Vec3(-1.0f, 0.0f, 0.0f);
        }
        if (distToPositiveY < minDist)
        {
            minDist = distToPositiveY;
            localNormal = math::Vec3(0.0f, 1.0f, 0.0f);
        }
        if (distToNegativeY < minDist)
        {
            minDist = distToNegativeY;
            localNormal = math::Vec3(0.0f, -1.0f, 0.0f);
        }
        if (distToPositiveZ < minDist)
        {
            minDist = distToPositiveZ;
            localNormal = math::Vec3(0.0f, 0.0f, 1.0f);
        }
        if (distToNegativeZ < minDist)
        {
            minDist = distToNegativeZ;
            localNormal = math::Vec3(0.0f, 0.0f, -1.0f);
        }

        overlap = sphereRadius + minDist;
        closestLocal = localSpherePos - localNormal * minDist;
    }

    // Local -> Wordl
    math::Vec3 normalWorld = boxRot * localNormal;
    math::Vec3 contactPointWorld = boxBody.transform.position + (boxRot * closestLocal);

    outContact.bodyA = sphereBody.bodyID;
    outContact.bodyB = boxBody.bodyID;
    outContact.normal = normalWorld;
    outContact.overlap = overlap;

    outContact.pointCount = 1;
    outContact.points[0].position = contactPointWorld;
    outContact.points[0].depth = overlap;

    return true;
}

bool sas::Traits3D::SphereCilinder(const Body &obj, const Body &other, Contact &outContact) noexcept
{
    float sphereRadius = obj.shape.radius * obj.transform.scale.x;

    float cylRadius = other.shape.radHeight.x * other.transform.scale.x;
    float cylHalfHeight = other.shape.radHeight.y * other.transform.scale.y;

    math::Vec3 deltaWorld = obj.transform.position - other.transform.position;

    math::Mat3 cylRot = other.transform.quaterion.toMat3();
    math::Mat3 invCylRot = cylRot.transposed();
    math::Vec3 localSphere = invCylRot * deltaWorld;

    math::Vec3 closestLocal;
    closestLocal.y = std::clamp(localSphere.y, -cylHalfHeight, cylHalfHeight);

    float xzDistSq = localSphere.x * localSphere.x + localSphere.z * localSphere.z;

    math::Vec3 localNormal;
    float overlap = 0.0f;

    bool insideY = (localSphere.y >= -cylHalfHeight && localSphere.y <= cylHalfHeight);
    bool insideRadial = (xzDistSq <= cylRadius * cylRadius);

    if (insideY && insideRadial)
    {
        float distToTop = cylHalfHeight - localSphere.y;
        float distToBottom = localSphere.y + cylHalfHeight;
        float distToCap = std::min(distToTop, distToBottom);

        float xzDist = std::sqrt(xzDistSq);
        float distToWall = cylRadius - xzDist;

        if (distToCap < distToWall)
        {
            if (distToTop < distToBottom)
            {
                localNormal = math::Vec3(0.0f, 1.0f, 0.0f);
                closestLocal = math::Vec3(localSphere.x, cylHalfHeight, localSphere.z);
                overlap = sphereRadius + distToTop;
            }
            else
            {
                localNormal = math::Vec3(0.0f, -1.0f, 0.0f);
                closestLocal = math::Vec3(localSphere.x, -cylHalfHeight, localSphere.z);
                overlap = sphereRadius + distToBottom;
            }
        }
        else
        {
            math::Vec3 radialDir = (xzDist > 1e-4f)
                                       ? math::Vec3(localSphere.x / xzDist, 0.0f, localSphere.z / xzDist)
                                       : math::Vec3(1.0f, 0.0f, 0.0f);

            localNormal = radialDir;
            closestLocal = math::Vec3(radialDir.x * cylRadius, localSphere.y, radialDir.z * cylRadius);
            overlap = sphereRadius + distToWall;
        }
    }
    else
    {
        if (xzDistSq > cylRadius * cylRadius)
        {
            float xzDist = std::sqrt(xzDistSq);
            closestLocal.x = (localSphere.x / xzDist) * cylRadius;
            closestLocal.z = (localSphere.z / xzDist) * cylRadius;
        }
        else
        {
            closestLocal.x = localSphere.x;
            closestLocal.z = localSphere.z;
        }

        math::Vec3 localDiff = localSphere - closestLocal;
        float distSq = localDiff.x * localDiff.x + localDiff.y * localDiff.y + localDiff.z * localDiff.z;

        if (distSq > sphereRadius * sphereRadius)
            return false;

        float dist = std::sqrt(distSq);
        localNormal = (dist > 1e-4f) ? (localDiff * (1.0f / dist)) : math::Vec3(0.0f, 1.0f, 0.0f);
        overlap = sphereRadius - dist;
    }

    math::Vec3 normalWorld = cylRot * localNormal;
    math::Vec3 contactPointWorld = other.transform.position + (cylRot * closestLocal);

    outContact.bodyA = obj.bodyID;
    outContact.bodyB = other.bodyID;
    outContact.normal = normalWorld;
    outContact.overlap = overlap;

    outContact.pointCount = 1;
    outContact.points[0].position = contactPointWorld;
    outContact.points[0].depth = overlap;

    return true;
}

// OOB. Nu s-a cerut rotatia. In aceest caz nu este necesara o implementare "completa" cu
// Contact Manifolds
bool sas::Traits3D::OrthopeOrthope(const Body &obj, const Body &other, Contact &outContact) noexcept
{
    math::Vec3 posA = obj.transform.position;
    math::Vec3 posB = other.transform.position;

    math::Vec3 extA = {
        obj.shape.halfSize.x * obj.transform.scale.x,
        obj.shape.halfSize.y * obj.transform.scale.y,
        obj.shape.halfSize.z * obj.transform.scale.z};

    math::Vec3 extB = {
        other.shape.halfSize.x * other.transform.scale.x,
        other.shape.halfSize.y * other.transform.scale.y,
        other.shape.halfSize.z * other.transform.scale.z};

    math::Mat3 rotA = obj.transform.quaterion.toMat3();
    math::Mat3 rotB = other.transform.quaterion.toMat3();

    math::Vec3 uA[3] = {
        math::Vec3{rotA(0, 0), rotA(1, 0), rotA(2, 0)},
        math::Vec3{rotA(0, 1), rotA(1, 1), rotA(2, 1)},
        math::Vec3{rotA(0, 2), rotA(1, 2), rotA(2, 2)}};

    math::Vec3 uB[3] = {
        math::Vec3{rotB(0, 0), rotB(1, 0), rotB(2, 0)},
        math::Vec3{rotB(0, 1), rotB(1, 1), rotB(2, 1)},
        math::Vec3{rotB(0, 2), rotB(1, 2), rotB(2, 2)}};

    math::Vec3 delta = posA - posB;

    float minOverlap = std::numeric_limits<float>::max();
    math::Vec3 bestAxis(0.0f, 1.0f, 0.0f);

    // 15 axe
    auto testAxis = [&](math::Vec3 axis) -> bool
    {
        float lenSq = axis.lengthSq();
        if (lenSq < 1e-6f)
            return true;

        axis = axis * (1.0f / std::sqrt(lenSq));

        float rA = extA.x * std::abs(math::dotProduct(uA[0], axis)) +
                   extA.y * std::abs(math::dotProduct(uA[1], axis)) +
                   extA.z * std::abs(math::dotProduct(uA[2], axis));

        float rB = extB.x * std::abs(math::dotProduct(uB[0], axis)) +
                   extB.y * std::abs(math::dotProduct(uB[1], axis)) +
                   extB.z * std::abs(math::dotProduct(uB[2], axis));

        float dist = std::abs(math::dotProduct(delta, axis));
        float overlap = (rA + rB) - dist;

        if (overlap < 0.0f)
            return false;

        if (overlap < minOverlap)
        {
            minOverlap = overlap;

            if (math::dotProduct(delta, axis) < 0.0f)
            {
                axis = -axis;
            }
            bestAxis = axis;
        }

        return true;
    };

    for (int i = 0; i < 3; ++i)
        if (!testAxis(uA[i]))
            return false;

    for (int i = 0; i < 3; ++i)
        if (!testAxis(uB[i]))
            return false;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            math::Vec3 crossAxis = uA[i] * uB[j];
            if (!testAxis(crossAxis))
                return false;
        }
    }

    math::Vec3 ptA = posA;
    for (int i = 0; i < 3; ++i)
    {
        float sign = (math::dotProduct(uA[i], -bestAxis) >= 0.0f) ? 1.0f : -1.0f;
        ptA += uA[i] * (sign * extA.data[i]);
    }

    math::Vec3 ptB = posB;
    for (int i = 0; i < 3; ++i)
    {
        float sign = (math::dotProduct(uB[i], bestAxis) >= 0.0f) ? 1.0f : -1.0f;
        ptB += uB[i] * (sign * extB.data[i]);
    }

    math::Vec3 contactPoint = (ptA + ptB) * 0.5f;

    outContact.bodyA = obj.bodyID;
    outContact.bodyB = other.bodyID;
    outContact.normal = bestAxis;
    outContact.overlap = minOverlap;

    outContact.pointCount = 1;
    outContact.points[0].position = contactPoint;
    outContact.points[0].depth = minOverlap;

    return true;
}
#if 0
[[nodiscard]] static bool GJK(const sas::Body3D &obj, const sas::Body3D &other) noexcept
{
    using namespace sas;
    // 1 direction vector
    math::Vec3 d = (obj.transform.position - other.transform.position) + math::Vec3(0.001f, 0.001f, 0.001f);

    // 2 find Sc(d) = Sa(d) - Sb(-d)
    // Sa(d) furthest point from d
    // Sb(d) furthest point from d in opp direction
    // Both have analytical formulas ---> No vertex loops needed
    // If more shpaes are added, this should be a function pointer instead of this

    auto signOf = [](float f) noexcept
    {
        return std::copysign(1.0f, f);
    };

    auto computerSD = [signOf](const Body3D &body, const math::Vec3 &objD) noexcept -> math::Vec3
    {
        switch (body.shape.type)
        {
        case ShapeType3D::Orthotope:
        {
            const auto &scaledHalfsObj = body.shape.halfSize * body.transform.scale;
            return {
                signOf(objD.x) * scaledHalfsObj.x,
                signOf(objD.y) * scaledHalfsObj.y,
                signOf(objD.z) * scaledHalfsObj.z,
            };
        }
        case ShapeType3D::Cilinder:
        {
            float xzLenSq = objD.x * objD.x + objD.z * objD.z;
            float px = 0.0f, pz = 0.0f;

            if (xzLenSq > 1e-8f)
            {
                float xzLen = std::sqrt(xzLenSq);
                px = (objD.x / xzLen) * body.shape.radHeight.x;
                pz = (objD.z / xzLen) * body.shape.radHeight.x;
            }
            else
            {
                px = body.shape.radHeight.x;
                pz = 0.0f;
            }

            float py = objD.y >= 0.0f ? body.shape.radHeight.y : -body.shape.radHeight.y;
            math::Vec3 scaled = math::Vec3{px, py, pz} * body.transform.scale;
            return scaled;
        }
        default:
            BaseLogger::error(std::string("Passed to GJK a shape that is not known") + enumToString(body.shape.type));
            std::unreachable();
        }
    };

    const auto supportD = [&obj, &other, computerSD, signOf](const math::Vec3 &currentD) noexcept
    {
        const auto &localObjD = obj.transform.quaterion.inverse() * currentD;
        const math::Vec3 Sda = obj.transform.quaterion.norm() * computerSD(obj, localObjD) + obj.transform.position;

        const auto &localOtherD = other.transform.quaterion.inverse() * currentD;
        const math::Vec3 Sdb = other.transform.quaterion.norm() * computerSD(other, -localOtherD) + other.transform.position;

        return Sda - Sdb;
    };

    struct Simplex
    {
        std::array<sas::math::Vec3, 4> points;
        int size = 0;

        void push_front(const sas::math::Vec3 &point)
        {
            points = {point, points[0], points[1], points[2]};
            size = std::min(size + 1, 4);
        }
    };

    Simplex s;

    const auto doSimplex = [&](this auto &&self) noexcept -> bool
    {
        switch (s.size)
        {
        case 2:
        {
            sas::math::Vec3 a = s.points[0];
            sas::math::Vec3 b = s.points[1];
            sas::math::Vec3 ab = b - a;
            sas::math::Vec3 ao = -a;

            if (dotProduct(ab, ao) > 0.0f)
            {
                d = (ab * ao) * ab;
                if (d.lengthSq() < 1e-6f)
                {
                    sas::math::Vec3 up(0, 1, 0);
                    d = ab * up;
                    if (d.lengthSq() < 1e-6f)
                    {
                        d = ab * sas::math::Vec3(1, 0, 0);
                    }
                }
            }
            else
            {
                s.points = {a, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
                s.size = 1;
                d = ao;
            }
            return false;
        }

        case 3:
        {
            sas::math::Vec3 a = s.points[0];
            sas::math::Vec3 b = s.points[1];
            sas::math::Vec3 c = s.points[2];

            sas::math::Vec3 ab = b - a;
            sas::math::Vec3 ac = c - a;
            sas::math::Vec3 ao = -a;

            sas::math::Vec3 abc = ab * ac;

            if (dotProduct(abc * ac, ao) > 0.0f)
            {
                if (dotProduct(ac, ao) > 0.0f)
                {
                    s.points = {a, c, {0, 0, 0}, {0, 0, 0}};
                    s.size = 2;
                    return self();
                }
                else
                {
                    s.points = {a, b, {0, 0, 0}, {0, 0, 0}};
                    s.size = 2;
                    return self();
                }
            }
            else
            {
                if (dotProduct(ab * abc, ao) > 0.0f)
                {
                    s.points = {a, b, {0, 0, 0}, {0, 0, 0}};
                    s.size = 2;
                    return self();
                }
                else
                {
                    if (dotProduct(abc, ao) > 0.0f)
                    {
                        d = abc;
                    }
                    else
                    {
                        s.points = {a, c, b, {0, 0, 0}};
                        d = -abc;
                    }
                }
            }
            return false;
        }

        case 4:
        {
            sas::math::Vec3 A = s.points[0];
            sas::math::Vec3 B = s.points[1];
            sas::math::Vec3 C = s.points[2];
            sas::math::Vec3 D = s.points[3];

            sas::math::Vec3 ab = B - A;
            sas::math::Vec3 ac = C - A;
            sas::math::Vec3 ad = D - A;
            sas::math::Vec3 ao = -A;

            sas::math::Vec3 abc = ab * ac;
            sas::math::Vec3 acd = ac * ad;
            sas::math::Vec3 adb = ad * ab;

            if (dotProduct(abc, ad) > 0.0f)
                abc = -abc;
            if (dotProduct(acd, ab) > 0.0f)
                acd = -acd;
            if (dotProduct(adb, ac) > 0.0f)
                adb = -adb;

            if (dotProduct(abc, ao) > 0.0f)
            {
                s.points = {A, B, C, {0, 0, 0}};
                s.size = 3;
                return self();
            }
            if (dotProduct(acd, ao) > 0.0f)
            {
                s.points = {A, C, D, {0, 0, 0}};
                s.size = 3;
                return self();
            }
            if (dotProduct(adb, ao) > 0.0f)
            {
                s.points = {A, D, B, {0, 0, 0}};
                s.size = 3;
                return self();
            }

            return true;
        }

        default:
            BaseLogger::error("Do simplex has a weird value: " + std::to_string(s.size));
            std::unreachable();
            return false;
        }
    };

    s.points[0] = supportD(d);
    d = -s.points[0];

    constexpr size_t maxIterations = 20;
    size_t counter = 0;

    while (counter < maxIterations)
    {
        BaseLogger::log("Trying to find with counter = " + std::to_string(counter));
        ++counter;
        const auto &p = supportD(d);
        if (math::dotProduct(p, d) < -1e-6)
        {
            return false;
        }

        s.push_front(p);
        if (doSimplex())
        {
            return true;
        }
    }

    return false;
};
#else
//==================================================================================================

struct Simplex
{
    std::array<sas::math::Vec3, 4> points;
    int size = 0;

    void push_front(const sas::math::Vec3 &point)
    {
        points = {point, points[0], points[1], points[2]};
        size = std::min(size + 1, 4);
    }
};

[[nodiscard]] static sas::math::Vec3 localSupport(const sas::Shape3D &shape, const sas::math::Vec3 &localDir) noexcept
{
    if (shape.type == sas::ShapeType3D::Orthotope)
    {
        return {
            localDir.x >= 0.0f ? shape.halfSize.x : -shape.halfSize.x,
            localDir.y >= 0.0f ? shape.halfSize.y : -shape.halfSize.y,
            localDir.z >= 0.0f ? shape.halfSize.z : -shape.halfSize.z};
    }
    else
    {
        float xzLenSq = localDir.x * localDir.x + localDir.z * localDir.z;
        float px = 0.0f, pz = 0.0f;

        if (xzLenSq > 1e-8f)
        {
            float xzLen = std::sqrt(xzLenSq);
            px = (localDir.x / xzLen) * shape.radHeight.x;
            pz = (localDir.z / xzLen) * shape.radHeight.x;
        }
        else
        {
            px = shape.radHeight.x;
            pz = 0.0f;
        }

        float py = localDir.y >= 0.0f ? shape.radHeight.y : -shape.radHeight.y;
        return {px, py, pz};
    }
};

[[nodiscard]] static sas::math::Vec3 supportFunction(const sas::Shape3D &shape, const sas::Transform3D &tf, const sas::math::Vec3 &globalDir) noexcept
{
    using namespace sas;
    math::Vec3 localDir = tf.quaterion.inverse() * globalDir;

    localDir.x = !math::floatAlmostEqual(tf.scale.x, 0.0f) ? localDir.x / tf.scale.x : 0.0f;
    localDir.y = !math::floatAlmostEqual(tf.scale.y, 0.0f) ? localDir.y / tf.scale.y : 0.0f;
    localDir.z = !math::floatAlmostEqual(tf.scale.z, 0.0f) ? localDir.z / tf.scale.z : 0.0f;

    math::Vec3 localPoint = localSupport(shape, localDir);

    localPoint.x *= tf.scale.x;
    localPoint.y *= tf.scale.y;
    localPoint.z *= tf.scale.z;

    return tf.position + (tf.quaterion * localPoint);
};

[[nodiscard]] static sas::math::Vec3 distanceM(const sas::Shape3D &s1, const sas::Transform3D &trf1,
                                               const sas::Shape3D &s2, const sas::Transform3D &trf2,
                                               const sas::math::Vec3 &globalDir) noexcept
{
    sas::math::Vec3 p1 = supportFunction(s1, trf1, globalDir);
    sas::math::Vec3 p2 = supportFunction(s2, trf2, -globalDir);
    return p1 - p2;
};
[[nodiscard]] static std::optional<Simplex> GJK(const sas::Body3D &body1, const sas::Body3D &body2) noexcept
{
    using namespace sas;

    const sas::Shape3D &shape1 = body1.shape;
    const sas::Transform3D &tf1 = body1.transform;
    const sas::Shape3D &shape2 = body2.shape;
    const sas::Transform3D &tf2 = body2.transform;

    if (shape1.type != sas::ShapeType3D::Cilinder && shape1.type != sas::ShapeType3D::Orthotope) [[unlikely]]
    {
        BaseLogger::warn(std::string("Shape 1 is not supported atm. The collisions might be wrong") + sas::enumToString(shape1.type));
    }

    if (shape2.type != sas::ShapeType3D::Cilinder && shape2.type != sas::ShapeType3D::Orthotope) [[unlikely]]
    {
        BaseLogger::warn(std::string("Shape 2 is not supported atm. The collisions might be wrong") + sas::enumToString(shape2.type));
    }

    math::Vec3 dir = (tf2.position - tf1.position) + math::Vec3(0.001f, 0.001f, 0.001f);

    Simplex simplex;
    simplex.push_front(distanceM(shape1, tf1, shape2, tf2, dir));
    dir = -simplex.points[0];

    auto solveSimplex = [&](this auto &&self) noexcept
    {
        switch (simplex.size)
        {
        case 2:
        {
            math::Vec3 a = simplex.points[0];
            math::Vec3 b = simplex.points[1];
            math::Vec3 ab = b - a;
            math::Vec3 ao = -a;

            if (dotProduct(ab, ao) > 0.0f)
            {
                dir = (ab * ao) * ab;
                if (dir.lengthSq() < 1e-6f)
                {
                    sas::math::Vec3 up(0, 1, 0);
                    dir = ab * up;
                    if (dir.lengthSq() < 1e-6f)
                        dir = ab * sas::math::Vec3(1, 0, 0);
                }
            }
            else
            {
                simplex.points = {a, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
                simplex.size = 1;
                dir = ao;
            }
            return false;
        }
        case 3:
        {
            math::Vec3 a = simplex.points[0];
            math::Vec3 b = simplex.points[1];
            math::Vec3 c = simplex.points[2];

            math::Vec3 ab = b - a;
            math::Vec3 ac = c - a;
            math::Vec3 ao = -a;

            math::Vec3 abc = ab * ac;

            if (dotProduct(abc * ac, ao) > 0.0f)
            {
                if (dotProduct(ac, ao) > 0.0f)
                {
                    simplex.points = {a, c, {0, 0, 0}, {0, 0, 0}};
                    simplex.size = 2;
                    return self();
                }
                else
                {
                    simplex.points = {a, b, {0, 0, 0}, {0, 0, 0}};
                    simplex.size = 2;
                    return self();
                }
            }
            else
            {
                if (dotProduct(ab * abc, ao) > 0.0f)
                {
                    simplex.points = {a, b, {0, 0, 0}, {0, 0, 0}};
                    simplex.size = 2;
                    return self();
                }
                else
                {
                    if (dotProduct(abc, ao) > 0.0f)
                    {
                        dir = abc;
                    }
                    else
                    {
                        simplex.points = {a, c, b, {0, 0, 0}};
                        dir = -abc;
                    }
                }
            }
            return false;
        }
        case 4:
        {
            math::Vec3 a = simplex.points[0];
            math::Vec3 b = simplex.points[1];
            math::Vec3 c = simplex.points[2];
            math::Vec3 d = simplex.points[3];

            math::Vec3 ab = b - a;
            math::Vec3 ac = c - a;
            math::Vec3 ad = d - a;
            math::Vec3 ao = -a;

            math::Vec3 abc = ab * ac;
            math::Vec3 acd = ac * ad;
            math::Vec3 adb = ad * ab;

            if (dotProduct(abc, ad) > 0.0f)
                abc = -abc;
            if (dotProduct(acd, ab) > 0.0f)
                acd = -acd;
            if (dotProduct(adb, ac) > 0.0f)
                adb = -adb;

            if (dotProduct(abc, ao) > 0.0f)
            {
                simplex.points = {a, b, c, {0, 0, 0}};
                simplex.size = 3;
                return self();
            }
            if (dotProduct(acd, ao) > 0.0f)
            {
                simplex.points = {a, c, d, {0, 0, 0}};
                simplex.size = 3;
                return self();
            }
            if (dotProduct(adb, ao) > 0.0f)
            {
                simplex.points = {a, d, b, {0, 0, 0}};
                simplex.size = 3;
                return self();
            }

            return true;
        }
        default:
            BaseLogger::error("Simplex Size is unkkown" + std::to_string(simplex.size));
            std::unreachable();
            return false;
        }
    };

    const int MAX_ITERATIONS = 64;

    for (int i = 0; i < MAX_ITERATIONS; ++i)
    {
        math::Vec3 p = distanceM(shape1, tf1, shape2, tf2, dir);

        if (dotProduct(p, dir) <= 0.0f)
        {
            return std::nullopt;
        }

        for (int j = 0; j < simplex.size; ++j)
        {
            if ((simplex.points[j] - p).lengthSq() < 1e-5f)
            {
                return std::nullopt;
            }
        }

        simplex.push_front(p);

        if (solveSimplex())
        {
            return simplex;
        }
    }

    return std::nullopt;
}
#endif

static void EPA(const sas::Body3D &obj, const sas::Body3D &other, const Simplex &simplex, sas::Contact3D &outContact) noexcept
{
    using namespace sas;
    struct Face
    {
        math::Vec3 x;
        math::Vec3 y;
        math::Vec3 z;
        math::Vec3 normal{0.f};
        bool visible = false;
    };

    struct UndirectedEdge
    {
        math::Vec3 minV;
        math::Vec3 maxV;
        math::Vec3 originalFrom;
        math::Vec3 originalTo;

        static UndirectedEdge make(const math::Vec3 &a, const math::Vec3 &b)
        {
            if (a < b)
            {
                return {a, b, a, b};
            }
            return {b, a, a, b};
        }

        bool equals(const UndirectedEdge &o) const
        {
            return minV == o.minV && maxV == o.maxV;
        }
    };

    std::vector<UndirectedEdge> horizon;

    auto addOrRemoveEdge = [&](const math::Vec3 &a, const math::Vec3 &b)
    {
        UndirectedEdge edge = UndirectedEdge::make(a, b);

        for (size_t horizonInd = 0; horizonInd < horizon.size(); ++horizonInd)
        {
            if (horizon[horizonInd].equals(edge))
            {
                horizon[horizonInd] = horizon.back();
                horizon.pop_back();
                return;
            }
        }

        horizon.push_back(edge);
    };

    const math::Vec3 A = simplex.points[0];
    const math::Vec3 B = simplex.points[1];
    const math::Vec3 C = simplex.points[2];
    const math::Vec3 D = simplex.points[3];

    const math::Vec3 center = (A + B + C + D) * 0.25f;

    std::vector<Face> faces = {
        {A, B, C},
        {A, C, D},
        {A, D, B},
        {B, C, D}};

    static constexpr size_t maxIterations = 100;
    for (size_t i = 0; i < maxIterations; ++i)
    {

        if (faces.empty())
        {
            BaseLogger::warn("Epa deleted all faces");
            break;
        }
        float minimDistance = std::numeric_limits<float>::max();
        // This will be invalidate when I
        // No longer need closestFace
        Face *closestFace = &faces[0];

        for (Face &current : faces)
        {
            const auto &edge1 = current.x - current.z;
            const auto &edge2 = current.y - current.z;

            auto perpNorm = (edge1 * edge2).normalized();

            if (math::dotProduct(perpNorm, current.x - center) < 1e-4f)
            {
                perpNorm = -perpNorm;
            }
            current.normal = perpNorm;

            float distance = std::abs(math::dotProduct(perpNorm, current.x));

            if (distance < minimDistance)
            {
                minimDistance = distance;
                closestFace = &current;
            }
        }

        const auto &newPoint = distanceM(obj.shape, obj.transform, other.shape, other.transform, closestFace->normal);
        const float newDistance = math::dotProduct(newPoint, closestFace->normal);

        const float comparedDist = newDistance - minimDistance;

        if (comparedDist < 1e-3 || i == maxIterations - 1)
        {
            ContactPoint3D point{newPoint, newDistance};
            outContact.normal = -closestFace->normal;
            outContact.points[0] = point;
            outContact.pointCount = 1;
            outContact.overlap = newDistance;

            BaseLogger::log("Finished with: " + std::to_string(faces.size()) + " " + std::to_string(horizon.size()));

            return;
        }

        horizon.clear();

        for (Face &current : faces)
        {

            if (math::dotProduct(newPoint - current.x, current.normal) > 0.0f)
            {
                current.visible = true;

                addOrRemoveEdge(current.x, current.y);
                addOrRemoveEdge(current.y, current.z);
                addOrRemoveEdge(current.z, current.x);
            }
        }

        std::erase_if(faces, [](const Face &face)
                      { return face.visible; });

        for (const auto &edge : horizon)
        {
            faces.push_back({edge.originalFrom, edge.originalTo, newPoint});
        }
    }

    BaseLogger::warn("Finished EPA with empty stuff");
    return;
}

bool sas::Traits3D::GJKEPA(const sas::Body3D &obj, const sas::Body3D &other, sas::Contact3D &outContact) noexcept
{
    const auto &simplex = GJK(obj, other);

    if (simplex)
    {
        EPA(obj, other, *simplex, outContact);
        return true;
    }

    return false;
}