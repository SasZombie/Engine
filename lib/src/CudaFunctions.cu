#include "CudaFunctions.hpp"

#include "BodyCore.hpp"
#include "Contact.hpp"
#include <array>
#include <cstdio>
#include <cuda_runtime.h>

__global__ void helloKernel()
{
    printf("GPU executing from Physics Engine!\n");
}

void sas::initCuda() noexcept
{
    cudaSetDevice(0);
    cudaFree(0);
    size_t freeMem = 0, totalMem = 0;
    cudaError_t memErr = cudaMemGetInfo(&freeMem, &totalMem);
    if (memErr != cudaSuccess)
    {
        std::fprintf(stderr, "[CUDA MEM INFO ERROR]: %s\n", cudaGetErrorString(memErr));
    }
    else
    {
        std::printf("Device free: %zu MB / total: %zu MB\n",
                    freeMem / (1024 * 1024), totalMem / (1024 * 1024));
    }

    cudaGetLastError();
    cudaError_t launchErr = cudaGetLastError();
    if (launchErr != cudaSuccess)
    {
        std::fprintf(stderr, "[CUDA LAUNCH ERROR]: %s\n", cudaGetErrorString(launchErr));
        return;
    }

    cudaError_t syncErr = cudaDeviceSynchronize();
    if (syncErr != cudaSuccess)
    {
        std::fprintf(stderr, "[CUDA SYNC ERROR]: %s\n", cudaGetErrorString(syncErr));
        return;
    }

    std::fflush(stdout);
}

__host__ __device__ bool hasFlag(sas::Body3D &obj, uint32_t flag) noexcept
{
    return (obj.flags & flag) != 0;
}

__device__ bool checkSphereSphere(
    const sas::Body3D &obj,
    const sas::Body3D &other,
    sas::Contact3D *contacts,
    int *contactCount,
    size_t maxContacts)
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

    float distance = sqrtf(distanceSq);

    sas::math::Vec3 normal = (distance > 1e-4f)
                                 ? sas::math::Vec3(dx / distance, dy / distance, dz / distance)
                                 : sas::math::Vec3(0.0f, 1.0f, 0.0f);

    float overlap = combinedRad - distance;
    sas::math::Vec3 contactPoint = obj.transform.position + normal * (radiusA - overlap * 0.5f);

    sas::Contact3D outContact;
    outContact.bodyA = obj.bodyID;
    outContact.bodyB = other.bodyID;
    outContact.normal = normal;
    outContact.overlap = overlap;
    outContact.pointCount = 1;
    outContact.points[0].position = contactPoint;
    outContact.points[0].depth = overlap;

    int slot = atomicAdd(contactCount, 1);
    if (slot < maxContacts)
    {
        contacts[slot] = outContact;
        return true;
    }

    return false;
}

__device__ bool checkSphereOrthotope(const sas::Body3D &sphereBody, const sas::Body3D &boxBody, sas::Contact3D *contacts,
                                     int *contactCount, size_t maxContacts) noexcept
{
    float sphereRadius = sphereBody.shape.radius * sphereBody.transform.scale.x;

    sas::math::Vec3 boxHalfExtents = {
        boxBody.shape.halfSize.x * boxBody.transform.scale.x,
        boxBody.shape.halfSize.y * boxBody.transform.scale.y,
        boxBody.shape.halfSize.z * boxBody.transform.scale.z};

    sas::math::Vec3 deltaWorld = sphereBody.transform.position - boxBody.transform.position;

    sas::math::Mat3 boxRot = boxBody.transform.quaterion.toMat3();
    sas::math::Mat3 invBoxRot = boxRot.transposed();
    sas::math::Vec3 localSpherePos = invBoxRot * deltaWorld;

    sas::math::Vec3 closestLocal = {
        std::clamp(localSpherePos.x, -boxHalfExtents.x, boxHalfExtents.x),
        std::clamp(localSpherePos.y, -boxHalfExtents.y, boxHalfExtents.y),
        std::clamp(localSpherePos.z, -boxHalfExtents.z, boxHalfExtents.z)};

    sas::math::Vec3 localDiff = localSpherePos - closestLocal;
    float distSq = localDiff.x * localDiff.x + localDiff.y * localDiff.y + localDiff.z * localDiff.z;

    sas::math::Vec3 localNormal;
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
        localNormal = sas::math::Vec3(1.0f, 0.0f, 0.0f);

        if (distToNegativeX < minDist)
        {
            minDist = distToNegativeX;
            localNormal = sas::math::Vec3(-1.0f, 0.0f, 0.0f);
        }
        if (distToPositiveY < minDist)
        {
            minDist = distToPositiveY;
            localNormal = sas::math::Vec3(0.0f, 1.0f, 0.0f);
        }
        if (distToNegativeY < minDist)
        {
            minDist = distToNegativeY;
            localNormal = sas::math::Vec3(0.0f, -1.0f, 0.0f);
        }
        if (distToPositiveZ < minDist)
        {
            minDist = distToPositiveZ;
            localNormal = sas::math::Vec3(0.0f, 0.0f, 1.0f);
        }
        if (distToNegativeZ < minDist)
        {
            minDist = distToNegativeZ;
            localNormal = sas::math::Vec3(0.0f, 0.0f, -1.0f);
        }

        overlap = sphereRadius + minDist;
        closestLocal = localSpherePos - localNormal * minDist;
    }

    // Local -> Wordl
    sas::math::Vec3 normalWorld = boxRot * localNormal;
    sas::math::Vec3 contactPointWorld = boxBody.transform.position + (boxRot * closestLocal);

    sas::Contact3D outContact;
    outContact.bodyA = sphereBody.bodyID;
    outContact.bodyB = boxBody.bodyID;
    outContact.normal = normalWorld;
    outContact.overlap = overlap;

    outContact.pointCount = 1;
    outContact.points[0].position = contactPointWorld;
    outContact.points[0].depth = overlap;

    int slot = atomicAdd(contactCount, 1);
    if (slot < maxContacts)
    {
        contacts[slot] = outContact;
        return true;
    }

    return false;
}

__device__ bool checkSphereCilinder(const sas::Body3D &obj, const sas::Body3D &other, sas::Contact3D *contacts,
                                    int *contactCount, size_t maxContacts) noexcept
{
    using namespace sas;
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

    sas::Contact3D outContact;
    outContact.bodyA = obj.bodyID;
    outContact.bodyB = other.bodyID;
    outContact.normal = normalWorld;
    outContact.overlap = overlap;

    outContact.pointCount = 1;
    outContact.points[0].position = contactPointWorld;
    outContact.points[0].depth = overlap;

    int slot = atomicAdd(contactCount, 1);
    if (slot < maxContacts)
    {
        contacts[slot] = outContact;
        return true;
    }

    return true;
}

__global__ void narrowPhaseAnalyticKernel(
    const sas::CandidatePair *pairs,
    size_t pairsSize,
    const sas::Body3D *bodies,
    sas::Contact3D *contacts,
    int *contactCount,
    size_t maxContacts)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= pairsSize)
        return;

    const auto &obj = bodies[pairs[i].bodyIndexA];
    const auto &other = bodies[pairs[i].bodyIndexB];

    // They are ordered so slot obj is 100% the sphere
    switch (other.shape.type)
    {
    case sas::ShapeType3D::Sphere:
        checkSphereSphere(obj, other, contacts, contactCount, maxContacts);
        break;
    case sas::ShapeType3D::Orthotope:
        checkSphereOrthotope(obj, other, contacts, contactCount, maxContacts);
        break;
    case sas::ShapeType3D::Cilinder:
        checkSphereCilinder(obj, other, contacts, contactCount, maxContacts);
        break;
    default:
        break;
    }
}

__global__ void narrowPhaseBoxBoxKernel(const sas::CandidatePair *pairs, size_t pairsSize, const sas::Body3D *bodies,
                                        sas::Contact3D *contacts, int *contactCount, size_t maxContacts)

{
    int threadId = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadId >= pairsSize)
        return;

    const auto &obj = bodies[pairs[threadId].bodyIndexA];
    const auto &other = bodies[pairs[threadId].bodyIndexB];
    using namespace sas;

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

    float minOverlap = 1e30f;
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
            return;

    for (int i = 0; i < 3; ++i)
        if (!testAxis(uB[i]))
            return;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            math::Vec3 crossAxis = uA[i] * uB[j];
            if (!testAxis(crossAxis))
                return;
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

    Contact3D outContact;
    outContact.bodyA = obj.bodyID;
    outContact.bodyB = other.bodyID;
    outContact.normal = bestAxis;
    outContact.overlap = minOverlap;

    outContact.pointCount = 1;
    outContact.points[0].position = contactPoint;
    outContact.points[0].depth = minOverlap;

    int slot = atomicAdd(contactCount, 1);
    if (slot < maxContacts)
    {
        contacts[slot] = outContact;
    }
}
//==================================================
struct Simplex
{
    std::array<sas::math::Vec3, 4> points;
    int size = 0;

    __device__ void push_front(const sas::math::Vec3 &point)
    {
        points = {point, points[0], points[1], points[2]};
        size = std::min(size + 1, 4);
    }
};

__device__ sas::math::Vec3 localSupport(const sas::Shape3D &shape, const sas::math::Vec3 &localDir) noexcept
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

__device__ bool cudaFloatAlmosttEqual(float a, float b, float epsilon = 1e-30)
{
    return std::abs(a - b) < epsilon;
}

__device__ sas::math::Vec3 supportFunction(const sas::Shape3D &shape, const sas::Transform3D &tf, const sas::math::Vec3 &globalDir) noexcept
{
    using namespace sas;
    math::Vec3 localDir = tf.quaterion.inverse() * globalDir;

    localDir.x = !cudaFloatAlmosttEqual(tf.scale.x, 0.0f) ? localDir.x / tf.scale.x : 0.0f;
    localDir.y = !cudaFloatAlmosttEqual(tf.scale.y, 0.0f) ? localDir.y / tf.scale.y : 0.0f;
    localDir.z = !cudaFloatAlmosttEqual(tf.scale.z, 0.0f) ? localDir.z / tf.scale.z : 0.0f;

    math::Vec3 localPoint = localSupport(shape, localDir);

    localPoint.x *= tf.scale.x;
    localPoint.y *= tf.scale.y;
    localPoint.z *= tf.scale.z;

    return tf.position + (tf.quaterion * localPoint);
};

__device__ sas::math::Vec3 distanceM(const sas::Shape3D &s1, const sas::Transform3D &trf1,
                                     const sas::Shape3D &s2, const sas::Transform3D &trf2,
                                     const sas::math::Vec3 &globalDir) noexcept
{
    sas::math::Vec3 p1 = supportFunction(s1, trf1, globalDir);
    sas::math::Vec3 p2 = supportFunction(s2, trf2, -globalDir);
    return p1 - p2;
};

__device__ bool solveSimplex(Simplex &simplex, sas::math::Vec3 &dir) noexcept
{
    using namespace sas;
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
                return solveSimplex(simplex, dir);
            }
            else
            {
                simplex.points = {a, b, {0, 0, 0}, {0, 0, 0}};
                simplex.size = 2;
                return solveSimplex(simplex, dir);
            }
        }
        else
        {
            if (dotProduct(ab * abc, ao) > 0.0f)
            {
                simplex.points = {a, b, {0, 0, 0}, {0, 0, 0}};
                simplex.size = 2;
                return solveSimplex(simplex, dir);
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
            return solveSimplex(simplex, dir);
        }
        if (dotProduct(acd, ao) > 0.0f)
        {
            simplex.points = {a, c, d, {0, 0, 0}};
            simplex.size = 3;
            return solveSimplex(simplex, dir);
        }
        if (dotProduct(adb, ao) > 0.0f)
        {
            simplex.points = {a, d, b, {0, 0, 0}};
            simplex.size = 3;
            return solveSimplex(simplex, dir);
        }

        return true;
    }
    default:
        return false;
    }
}
__device__ bool GJK(const sas::Body3D &body1, const sas::Body3D &body2, Simplex &simplex) noexcept
{
    using namespace sas;

    const sas::Shape3D &shape1 = body1.shape;
    const sas::Transform3D &tf1 = body1.transform;
    const sas::Shape3D &shape2 = body2.shape;
    const sas::Transform3D &tf2 = body2.transform;

    math::Vec3 dir = (tf2.position - tf1.position) + math::Vec3(0.001f, 0.001f, 0.001f);

    simplex.push_front(distanceM(shape1, tf1, shape2, tf2, dir));
    dir = -simplex.points[0];

    const int MAX_ITERATIONS = 64;

    for (int i = 0; i < MAX_ITERATIONS; ++i)
    {
        math::Vec3 p = distanceM(shape1, tf1, shape2, tf2, dir);

        if (dotProduct(p, dir) <= 0.0f)
        {
            return false;
        }

        for (int j = 0; j < simplex.size; ++j)
        {
            if ((simplex.points[j] - p).lengthSq() < 1e-5f)
            {
                return false;
            }
        }

        simplex.push_front(p);

        if (solveSimplex(simplex, dir))
        {
            return true;
        }
    }

    return false;
}

template <typename T, size_t Capacity>
struct GpuVector
{
    T data[Capacity];
    int count = 0;

    __host__ __device__ void push_back(const T &item) noexcept
    {
        if (count < static_cast<int>(Capacity))
        {
            data[count++] = item;
        }
    }

    __host__ __device__ void pop_back() noexcept
    {
        if (count > 0)
        {
            --count;
        }
    }

    __host__ __device__ void clear() noexcept { count = 0; }
    __host__ __device__ int size() const noexcept { return count; }
    __host__ __device__ bool empty() const noexcept { return count == 0; }

    __host__ __device__ T &operator[](int idx) noexcept { return data[idx]; }
    __host__ __device__ const T &operator[](int idx) const noexcept { return data[idx]; }

    __host__ __device__ T *begin() noexcept { return data; }
    __host__ __device__ T *end() noexcept { return data + count; }
    __host__ __device__ const T *begin() const noexcept { return data; }
    __host__ __device__ const T *end() const noexcept { return data + count; }

    __host__ __device__ T &back() noexcept
    {
        return data[count - 1];
    }

    __host__ __device__ const T &back() const noexcept
    {
        return data[count - 1];
    }
};

template <typename T, size_t Capacity, typename Predicate>
__host__ __device__ void gpu_erase_if(GpuVector<T, Capacity> &vec, Predicate pred)
{
    int writeIdx = 0;
    for (int readIdx = 0; readIdx < vec.count; ++readIdx)
    {
        if (!pred(vec.data[readIdx]))
        {
            if (writeIdx != readIdx)
            {
                vec.data[writeIdx] = vec.data[readIdx];
            }
            ++writeIdx;
        }
    }
    vec.count = writeIdx;
}

__device__ void EPA(const sas::Body3D &obj, const sas::Body3D &other, const Simplex &simplex, sas::Contact3D *contacts, int *contactCount, size_t maxContacts) noexcept
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

    const math::Vec3 A = simplex.points[0];
    const math::Vec3 B = simplex.points[1];
    const math::Vec3 C = simplex.points[2];
    const math::Vec3 D = simplex.points[3];

    const math::Vec3 center = (A + B + C + D) * 0.25f;

    GpuVector<Face, 128> faces;

    faces.push_back({A, B, C});
    faces.push_back({A, C, D});
    faces.push_back({A, D, B});
    faces.push_back({B, C, D});

    static constexpr size_t maxIterations = 100;
    for (size_t i = 0; i < maxIterations; ++i)
    {

        if (faces.empty())
        {
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
            Contact3D outContact;
            outContact.bodyA = obj.bodyID;
            outContact.bodyB = other.bodyID;

            ContactPoint3D point{newPoint, newDistance};
            outContact.normal = -closestFace->normal;
            outContact.points[0] = point;
            outContact.pointCount = 1;
            outContact.overlap = newDistance;

            int slot = atomicAdd(contactCount, 1);
            if (slot < maxContacts)
            {
                contacts[slot] = outContact;
            }

            return;
        }

        struct UndirectedEdge
        {
            math::Vec3 minV;
            math::Vec3 maxV;
            math::Vec3 originalFrom;
            math::Vec3 originalTo;

            __device__ static UndirectedEdge make(const math::Vec3 &a, const math::Vec3 &b)
            {
                if (a < b)
                {
                    return {a, b, a, b};
                }
                return {b, a, a, b};
            }

            __device__ bool equals(const UndirectedEdge &o) const
            {
                return minV == o.minV && maxV == o.maxV;
            }
        };

        GpuVector<UndirectedEdge, 128> horizon;

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

        gpu_erase_if(faces, [](const Face &face)
                     { return face.visible; });

        for (const auto &edge : horizon)
        {
            faces.push_back({edge.originalFrom, edge.originalTo, newPoint});
        }
    }

    return;
}

__global__ void narrowPhaseGjkEpaKernel(const sas::CandidatePair *pairs, size_t pairsSize, const sas::Body3D *bodies,
                                        sas::Contact3D *contacts, int *contactCount, size_t maxContacts)

{
    int threadId = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadId >= pairsSize)
        return;

    const auto &obj = bodies[pairs[threadId].bodyIndexA];
    const auto &other = bodies[pairs[threadId].bodyIndexB];
    using namespace sas;
    Simplex s;
    if (GJK(obj, other, s))
    {
        sas::Contact3D outContact;
        outContact.bodyA = obj.bodyID;
        outContact.bodyA = other.bodyID;
        EPA(obj, other, s, contacts, contactCount, maxContacts);
    }
}

// MARK: GPU
//==================================================
void sas::launchGpu(const std::vector<CandidatePair> &analyticPairs,
                    const std::vector<CandidatePair> &boxBoxPairs,
                    const std::vector<CandidatePair> &gjkEpaPairs,
                    const std::vector<Body3D> &bodies,
                    std::vector<Contact3D> &outContacts) noexcept
{

    size_t totalPairs = analyticPairs.size() + boxBoxPairs.size() + gjkEpaPairs.size();
    if (totalPairs == 0)
    {
        return;
    }

    CandidatePair *d_allPairs = nullptr;
    cudaMalloc(&d_allPairs, totalPairs * sizeof(CandidatePair));

    size_t off0 = 0;
    size_t off1 = analyticPairs.size();
    size_t off2 = off1 + boxBoxPairs.size();

    if (!analyticPairs.empty())
        cudaMemcpy(d_allPairs + off0, analyticPairs.data(), analyticPairs.size() * sizeof(CandidatePair), cudaMemcpyHostToDevice);
    if (!boxBoxPairs.empty())
        cudaMemcpy(d_allPairs + off1, boxBoxPairs.data(), boxBoxPairs.size() * sizeof(CandidatePair), cudaMemcpyHostToDevice);
    if (!gjkEpaPairs.empty())
        cudaMemcpy(d_allPairs + off2, gjkEpaPairs.data(), gjkEpaPairs.size() * sizeof(CandidatePair), cudaMemcpyHostToDevice);

    Body3D *d_bodies = nullptr;
    cudaMalloc(&d_bodies, bodies.size() * sizeof(Body3D));
    cudaMemcpy(d_bodies, bodies.data(), bodies.size() * sizeof(Body3D), cudaMemcpyHostToDevice);

    const int maxContacts = 65536;
    Contact3D *d_outContacts = nullptr;
    int *d_contactCount = nullptr;
    cudaMalloc(&d_outContacts, maxContacts * sizeof(Contact3D));
    cudaMalloc(&d_contactCount, sizeof(int));
    cudaMemset(d_contactCount, 0, sizeof(int));

    const int blockSize = 128;

    if (!analyticPairs.empty())
    {
        int blocks = (analyticPairs.size() + blockSize - 1) / blockSize;
        narrowPhaseAnalyticKernel<<<blocks, blockSize>>>(
            d_allPairs + off0, analyticPairs.size(), d_bodies, d_outContacts, d_contactCount, maxContacts);
    }

    if (!boxBoxPairs.empty())
    {

        int blocks = (boxBoxPairs.size() + blockSize - 1) / blockSize;
        narrowPhaseBoxBoxKernel<<<blocks, blockSize>>>(
            d_allPairs + off1, boxBoxPairs.size(), d_bodies, d_outContacts, d_contactCount, maxContacts);
    }

    if (!gjkEpaPairs.empty())
    {

        int blocks = (gjkEpaPairs.size() + blockSize - 1) / blockSize;
        narrowPhaseGjkEpaKernel<<<blocks, blockSize>>>(
            d_allPairs + off2, gjkEpaPairs.size(), d_bodies, d_outContacts, d_contactCount, maxContacts);
    }

    cudaError_t err = cudaDeviceSynchronize();
    if (err != cudaSuccess)
    {
        printf("CUDA Kernel Error: %s\n", cudaGetErrorString(err));
    }
    int finalCount = 0;
    cudaMemcpy(&finalCount, d_contactCount, sizeof(int), cudaMemcpyDeviceToHost);
    finalCount = std::min(finalCount, maxContacts);

    if (finalCount > 0)
    {
        size_t prevSize = outContacts.size();
        outContacts.resize(prevSize + finalCount);
        cudaMemcpy(outContacts.data() + prevSize, d_outContacts, finalCount * sizeof(Contact3D), cudaMemcpyDeviceToHost);
    }

    cudaFree(d_allPairs);
    cudaFree(d_bodies);
    cudaFree(d_outContacts);
    cudaFree(d_contactCount);
}

void sas::launchGpu(const std::vector<CandidatePair> &analyticPairs,
                    const std::vector<CandidatePair> &boxBoxPairs,
                    const std::vector<CandidatePair> &gjkEpaPairs,
                    const std::vector<Body2D> &bodies,
                    std::vector<Contact2D> &outContacts) noexcept
{
    // TODO
}
