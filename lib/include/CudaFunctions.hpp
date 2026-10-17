#pragma once
#include <vector>
#include <cstdint>

#include "AABBTree.hpp"
#include "CudaObjects.hpp"
namespace sas
{
    void initCuda() noexcept;

    struct Body3D;
    struct Contact3D;
    struct Body2D;
    struct Contact2D;

    struct CollisionPipelineAlloc
    {
        uint32_t *d_activeIDs = nullptr;
        uint32_t *d_sparse = nullptr;
        sas::Body3D *d_bodies = nullptr;
        sas::FlatNode3D *d_flatNodes = nullptr;

        sas::CandidatePair *d_pairs = nullptr;
        int *d_counters = nullptr;
        sas::Contact3D *d_outContacts = nullptr;
        int *d_contactCount = nullptr;

        size_t maxBodies;
        size_t maxPairs;
        size_t maxContacts;

        CollisionPipelineAlloc(size_t nmaxBodies, size_t nmaxPairs, size_t nmaxContacts) noexcept;

        void cleanup() noexcept;

        ~CollisionPipelineAlloc() noexcept
        {
            cleanup();
        }
    };
    void runCollisionPipeline(sas::CollisionPipelineAlloc &buffers, const std::vector<uint32_t> &activeIDs, const std::vector<int> &sparse, const std::vector<sas::Body3D> &bodies,
                              const std::vector<sas::FlatNode<sas::AABB3D>> &flatTree, std::vector<sas::Contact3D> &outContacts);

    void runCollisionPipeline(sas::CollisionPipelineAlloc &buffers, const std::vector<uint32_t> &activeIDs,
                              const std::vector<int> &sparse, const std::vector<sas::Body2D> &bodies,
                              const std::vector<sas::FlatNode<sas::AABB2D>> &flatTree, std::vector<sas::Contact2D> &outContacts);

}