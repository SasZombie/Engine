#include "CudaFunctions.hpp"

#include "BodyCore.hpp"
#include "Contact.hpp"
#include <array>
#include <cstdio>
#include <cuda_runtime.h>

#define cudaCheck(call)                                           \
    do                                                            \
    {                                                             \
        cudaError_t err = (call);                                 \
        if (err != cudaSuccess)                                   \
        {                                                         \
            fprintf(stderr, "CUDA Error at %s:%d -> %s\n",        \
                    __FILE__, __LINE__, cudaGetErrorString(err)); \
            exit(EXIT_FAILURE);                                   \
        }                                                         \
    } while (0)

// void cudaCheck(cudaError_t err, const std::source_location location = std::source_location::current())
// {
//     if (err != cudaSuccess)
//     {
//         std::fprintf(stderr, "CUDA Error at %s : %s -> %s\n", location.file_name(), location.line(), cudaGetErrorString(err));
//     }
// }

void sas::initCuda() noexcept
{
    cudaSetDevice(0);
    cudaFree(0);
    size_t freeMem = 0, totalMem = 0;

    auto memErr = cudaMemGetInfo(&freeMem, &totalMem);
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

__device__ inline void emitPair(sas::DevicePairBins bins, sas::PairCategory cat, sas::CandidatePair pair)
{
    uint32_t catIdx = static_cast<uint32_t>(cat);

    unsigned int activeMask = __activemask();
#if __CUDA_ARCH__ >= 700
    unsigned int matchMask = __match_any_sync(activeMask, catIdx);
#else
    unsigned int matchMask = activeMask;
#endif

    int leader = __ffs(matchMask) - 1;
    int laneId = threadIdx.x & 31;
    int totalInWarp = __popc(matchMask);
    int rankInWarp = __popc(matchMask & ((1u << laneId) - 1));

    uint32_t baseOffset = 0;
    if (laneId == leader)
    {
        baseOffset = atomicAdd(&bins.counters[catIdx], totalInWarp);
    }
    baseOffset = __shfl_sync(matchMask, baseOffset, leader);

    uint32_t writeIdx = baseOffset + rankInWarp;
    if (writeIdx < bins.maxCapacity)
    {
        bins.pairs[catIdx][writeIdx] = pair;
    }
}

template <typename Traits>
__global__ void queryAndClassifyKernel(const uint32_t *__restrict__ activeIDs, uint32_t numActives,
                                       const uint32_t *__restrict__ sparse, const typename Traits::BodyType *__restrict__ bodies,
                                       const sas::FlatNode<typename Traits::AABBType> *__restrict__ treeNodes, int nodeCount,
                                       sas::DevicePairBins bins)
{
    using namespace sas;
    uint32_t tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid >= numActives)
        return;

    uint32_t idA = activeIDs[tid];
    uint32_t denseIdxA = sparse[idA];
    const auto &objA = bodies[denseIdxA];

    if (hasFlag(objA, Flags::RigidBodyStatic) || objA.hasFlag(Flags::Trigger))
    {
        return;
    }

    auto queryAABB = Traits::computeTightAABB(objA);

    uint32_t objLayer = objA.collisionMask & 0x0000FFFF;
    uint32_t objMask = (objA.collisionMask & 0xFFFF0000) >> 16;

    int index = 0;
    while (index < nodeCount && index >= 0)
    {
        const auto &node = treeNodes[index];

        if (aabbOverlap(node.aabb, queryAABB))
        {
            if (node.objectID >= 0)
            {
                // Leaf hit
                uint32_t otherID = static_cast<uint32_t>(node.objectID);
                uint32_t denseIdxB = sparse[otherID];
                const auto &other = bodies[denseIdxB];

                bool otherIsSkipped = other.hasFlag(Flags::RigidBodyStatic) ||
                                      other.hasFlag(Flags::Trigger);

                if (!otherIsSkipped && objA.bodyID >= otherID)
                {
                    index++;
                    continue;
                }

                uint32_t otherLayer = other.collisionMask & 0x0000FFFF;
                uint32_t otherMask = (other.collisionMask & 0xFFFF0000) >> 16;

                if ((objMask & otherLayer) && (otherMask & objLayer))
                {
                    PairCategory cat = Traits::classifyPair(objA.shape.type, other.shape.type);
                    CandidatePair pair = {objA.bodyID, otherID, denseIdxA, denseIdxB};
                    emitPair(bins, cat, pair);
                }

                index++;
            }
            else
            {
                index++;
            }
        }
        else
        {
            index = node.missOffset;
        }
    }
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
                // return solveSimplex(simplex, dir);
                return false;
            }
            else
            {
                simplex.points = {a, b, {0, 0, 0}, {0, 0, 0}};
                simplex.size = 2;
                // return solveSimplex(simplex, dir);
                return false;
            }
        }
        else
        {
            if (dotProduct(ab * abc, ao) > 0.0f)
            {
                simplex.points = {a, b, {0, 0, 0}, {0, 0, 0}};
                simplex.size = 2;
                // return solveSimplex(simplex, dir);
                return false;
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
            // return solveSimplex(simplex, dir);
            return false;
        }
        if (dotProduct(acd, ao) > 0.0f)
        {
            simplex.points = {a, c, d, {0, 0, 0}};
            simplex.size = 3;
            // return solveSimplex(simplex, dir);
            return false;
        }
        if (dotProduct(adb, ao) > 0.0f)
        {
            simplex.points = {a, d, b, {0, 0, 0}};
            simplex.size = 3;
            // return solveSimplex(simplex, dir);
            return false;
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

    const int MAX_ITERATIONS = 160;

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
constexpr int MAX_EPA_FACES = 48;
constexpr int MAX_EPA_HORIZON = 16;
constexpr int MAX_EPA_VERTS = 32;

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

    GpuVector<Face, MAX_EPA_FACES> faces;

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

        GpuVector<UndirectedEdge, MAX_EPA_HORIZON> horizon;

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

[[nodiscard]] __device__ static inline sas::PairCategory classifyPair(sas::ShapeType3D a, sas::ShapeType3D b) noexcept
{
    if (a > b)
    {
        std::swap(a, b);
    }
    if (a == sas::ShapeType3D::Sphere || b == sas::ShapeType3D::Sphere)
    {
        return sas::PairCategory::Analytic;
    }

    if (a == sas::ShapeType3D::Orthotope && b == sas::ShapeType3D::Orthotope)
    {
        return sas::PairCategory::Sat;
    }

    return sas::PairCategory::GjkEpa;
};

[[nodiscard]] __device__ static inline sas::PairCategory classifyPair(sas::ShapeType2D a, sas::ShapeType2D b) noexcept
{
    return sas::PairCategory::GjkEpa;
};

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
        outContact.bodyB = other.bodyID;
        EPA(obj, other, s, contacts, contactCount, maxContacts);
    }
}

// Holy parameters
template <typename Traits>
__global__ void broadphaseKernel(const uint32_t *__restrict__ activeIDs,
                                 uint32_t numActives, const uint32_t *__restrict__ sparse, const typename Traits::BodyType *__restrict__ bodies,
                                 const sas::FlatNode<typename Traits::AABBType> *__restrict__ treeNodes, int nodeCount, sas::CandidatePair *__restrict__ outPairs,
                                 int *__restrict__ counters, int maxPairsPerCat)
{
    using namespace sas;
    uint32_t tid = blockDim.x * blockIdx.x + threadIdx.x;
    if (tid >= numActives)
        return;

    uint32_t idA = activeIDs[tid];
    uint32_t denseIdxA = sparse[idA];
    const auto &objA = bodies[denseIdxA];

    if (objA.hasFlag(Flags::RigidBodyStatic) || objA.hasFlag(Flags::Trigger))
        return;

    auto queryAABB = Traits::computeTight(objA);
    uint32_t objLayer = objA.collisionMask & 0x0000FFFF;
    uint32_t objMask = (objA.collisionMask & 0xFFFF0000) >> 16;

    int index = 0;
    while (index < nodeCount && index >= 0)
    {
        const auto &node = treeNodes[index];

        if (node.aabb.overlaps(queryAABB))
        {
            if (node.objectID >= 0)
            {
                uint32_t otherID = static_cast<uint32_t>(node.objectID);
                uint32_t denseIdxB = sparse[otherID];
                const auto &other = bodies[denseIdxB];

                bool otherIsSkipped = other.hasFlag(Flags::RigidBodyStatic) || other.hasFlag(Flags::Trigger);
                if (!otherIsSkipped && objA.bodyID >= otherID)
                {
                    index++;
                    continue;
                }

                uint32_t otherLayer = other.collisionMask & 0x0000FFFF;
                uint32_t otherMask = (other.collisionMask & 0xFFFF0000) >> 16;

                if ((objMask & otherLayer) && (otherMask & objLayer))
                {
                    sas::PairCategory cat = classifyPair(objA.shape.type, other.shape.type);
                    int catIdx = static_cast<int>(cat);

                    int slot = atomicAdd(&counters[catIdx], 1);
                    if (slot < maxPairsPerCat)
                    {
                        int globalIndex = catIdx * maxPairsPerCat + slot;
                        outPairs[globalIndex] = sas::CandidatePair{objA.bodyID, otherID, denseIdxA, denseIdxB};
                    }
                }
                index++;
            }
            else
            {
                index++;
            }
        }
        else
        {
            index = node.missOffset;
        }
    }
}

void sas::runCollisionPipeline(sas::CollisionPipelineAlloc &buffers, const std::vector<uint32_t> &activeIDs, const std::vector<int> &sparse, const std::vector<sas::Body3D> &bodies,
                               const std::vector<sas::FlatNode<sas::AABB3D>> &flatTree, std::vector<sas::Contact3D> &outContacts)
{
    if (activeIDs.empty() || flatTree.empty())
        return;

    uint32_t *d_activeIDs = nullptr, *d_sparse = nullptr;
    sas::Body3D *d_bodies = nullptr;
    sas::FlatNode3D *d_flatNodes = nullptr;

    // cudaCheck(cudaMalloc(&buffers.d_activeIDs, activeIDs.size() * sizeof(uint32_t)));
    // cudaCheck(cudaMalloc(&buffers.d_sparse, sparse.size() * sizeof(uint32_t)));
    // cudaCheck(cudaMalloc(&buffers.d_bodies, bodies.size() * sizeof(Body3D)));
    // cudaCheck(cudaMalloc(&buffers.d_flatNodes, flatTree.size() * sizeof(FlatNode3D)));

    cudaCheck(cudaMemcpy(buffers.d_activeIDs, activeIDs.data(), activeIDs.size() * sizeof(uint32_t), cudaMemcpyHostToDevice));
    cudaCheck(cudaMemcpy(buffers.d_sparse, sparse.data(), sparse.size() * sizeof(uint32_t), cudaMemcpyHostToDevice));
    cudaCheck(cudaMemcpy(buffers.d_bodies, bodies.data(), bodies.size() * sizeof(Body3D), cudaMemcpyHostToDevice));
    cudaCheck(cudaMemcpy(buffers.d_flatNodes, flatTree.data(), flatTree.size() * sizeof(FlatNode3D), cudaMemcpyHostToDevice));

    const int maxPairsPerCat = buffers.maxPairs;
    const int maxContacts = buffers.maxContacts;
    const int totalBufferCapacity = maxPairsPerCat * 3;

    // CandidatePair *d_pairs = nullptr;
    // int *d_counters = nullptr;
    // cudaCheck(cudaMalloc(&buffers.d_pairs, totalBufferCapacity * sizeof(CandidatePair)));
    // cudaCheck(cudaMalloc(&buffers.d_counters, 3 * sizeof(int)));
    cudaCheck(cudaMemset(buffers.d_counters, 0, 3 * sizeof(int)));

    // Contact3D *d_outContacts = nullptr;
    // int *d_contactCount = nullptr;
    // cudaCheck(cudaMalloc(&buffers.d_outContacts, maxContacts * sizeof(Contact3D)));
    // cudaCheck(cudaMalloc(&buffers.d_contactCount, sizeof(int)));
    cudaCheck(cudaMemset(buffers.d_contactCount, 0, sizeof(int)));

    const int bpBlockSize = 128;
    int bpBlocks = (activeIDs.size() + bpBlockSize - 1) / bpBlockSize;

    broadphaseKernel<TreeTraits<AABB3D>><<<bpBlocks, bpBlockSize>>>(
        buffers.d_activeIDs, activeIDs.size(), buffers.d_sparse, buffers.d_bodies,
        buffers.d_flatNodes, flatTree.size(),
        buffers.d_pairs, buffers.d_counters, maxPairsPerCat);

    int hostCounters[3] = {0, 0, 0};
    cudaCheck(cudaMemcpy(hostCounters, buffers.d_counters, 3 * sizeof(int), cudaMemcpyDeviceToHost));

    int countAnalytic = std::min(hostCounters[0], maxPairsPerCat);
    int countBoxBox = std::min(hostCounters[1], maxPairsPerCat);
    int countGjkEpa = std::min(hostCounters[2], maxPairsPerCat);

    CandidatePair *d_analyticPairs = buffers.d_pairs + (0 * maxPairsPerCat);
    CandidatePair *d_boxBoxPairs = buffers.d_pairs + (1 * maxPairsPerCat);
    CandidatePair *d_gjkEpaPairs = buffers.d_pairs + (2 * maxPairsPerCat);

    const int npBlockSize = 128;

    if (countAnalytic > 0)
    {
        int blocks = (countAnalytic + npBlockSize - 1) / npBlockSize;
        narrowPhaseAnalyticKernel<<<blocks, npBlockSize>>>(
            d_analyticPairs, countAnalytic, buffers.d_bodies, buffers.d_outContacts, buffers.d_contactCount, maxContacts);
    }

    if (countBoxBox > 0)
    {
        int blocks = (countBoxBox + npBlockSize - 1) / npBlockSize;
        narrowPhaseBoxBoxKernel<<<blocks, npBlockSize>>>(
            d_boxBoxPairs, countBoxBox, buffers.d_bodies, buffers.d_outContacts, buffers.d_contactCount, maxContacts);
    }

    if (countGjkEpa > 0)
    {
        int blocks = (countGjkEpa + npBlockSize - 1) / npBlockSize;
        narrowPhaseGjkEpaKernel<<<blocks, npBlockSize>>>(
            d_gjkEpaPairs, countGjkEpa, buffers.d_bodies, buffers.d_outContacts, buffers.d_contactCount, maxContacts);
    }

    int finalContactCount = 0;
    cudaCheck(cudaMemcpy(&finalContactCount, buffers.d_contactCount, sizeof(int), cudaMemcpyDeviceToHost));
    finalContactCount = std::min(finalContactCount, maxContacts);

    outContacts.resize(finalContactCount);
    if (finalContactCount > 0)
    {
        cudaCheck(cudaMemcpy(outContacts.data(), buffers.d_outContacts, finalContactCount * sizeof(Contact3D), cudaMemcpyDeviceToHost));
    }

    // cudaCheck(cudaFree(d_activeIDs));
    // cudaCheck(cudaFree(d_sparse));
    // cudaCheck(cudaFree(d_bodies));
    // cudaCheck(cudaFree(d_flatNodes));
    // cudaCheck(cudaFree(d_pairs));
    // cudaCheck(cudaFree(d_counters));
    // cudaCheck(cudaFree(d_outContacts));
    // cudaCheck(cudaFree(d_contactCount));
}

sas::CollisionPipelineAlloc::CollisionPipelineAlloc(size_t nmaxBodies, size_t nmaxPairs, size_t nmaxContacts) noexcept
    : maxBodies(nmaxBodies), maxPairs(nmaxPairs), maxContacts(nmaxContacts)
{
    cudaMalloc(&d_activeIDs, maxBodies * sizeof(uint32_t));
    cudaMalloc(&d_sparse, maxBodies * sizeof(uint32_t));
    cudaMalloc(&d_bodies, maxBodies * sizeof(sas::Body3D));
    cudaMalloc(&d_flatNodes, maxBodies * sizeof(sas::FlatNode3D));

    cudaMalloc(&d_pairs, (maxPairs * 3) * sizeof(sas::CandidatePair));
    cudaMalloc(&d_counters, 3 * sizeof(int));
    cudaMalloc(&d_outContacts, maxContacts * sizeof(sas::Contact3D));
    cudaMalloc(&d_contactCount, sizeof(int));
}

void sas::CollisionPipelineAlloc::cleanup() noexcept
{

    cudaFree(d_activeIDs);
    cudaFree(d_bodies);
    cudaFree(d_contactCount);
    cudaFree(d_counters);
    cudaFree(d_flatNodes);
    cudaFree(d_outContacts);
    cudaFree(d_pairs);
    cudaFree(d_sparse);
}

void sas::runCollisionPipeline(sas::CollisionPipelineAlloc &buffers, const std::vector<uint32_t> &activeIDs, const std::vector<int> &sparse, const std::vector<sas::Body2D> &bodies,
                               const std::vector<sas::FlatNode<sas::AABB2D>> &flatTree, std::vector<sas::Contact2D> &outContacts)
{
}