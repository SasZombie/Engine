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

    void runCollisionPipeline(const std::vector<uint32_t> &activeIDs, const std::vector<int> &sparse, const std::vector<sas::Body3D> &bodies,
    const std::vector<sas::FlatNode<sas::AABB3D>> &flatTree, std::vector<sas::Contact3D> &outContacts);

    void runCollisionPipeline(const std::vector<uint32_t> &activeIDs,
    const std::vector<int> &sparse, const std::vector<sas::Body2D> &bodies,
    const std::vector<sas::FlatNode<sas::AABB2D>> &flatTree, std::vector<sas::Contact2D> &outContacts);

    
    void launchGpu(const std::vector<CandidatePair> &analyticPairs,
                   const std::vector<CandidatePair> &boxBoxPairs,
                   const std::vector<CandidatePair> &gjkEpaPairs,
                   const std::vector<Body3D> &bodies,
                   std::vector<Contact3D> &outContacts) noexcept;

    void launchGpu(const std::vector<CandidatePair> &analyticPairs,
                   const std::vector<CandidatePair> &boxBoxPairs,
                   const std::vector<CandidatePair> &gjkEpaPairs,
                   const std::vector<Body2D> &bodies,
                   std::vector<Contact2D> &outContacts) noexcept;
}