#pragma once
#include <vector>
#include <cstdint>
namespace sas
{
    void initCuda() noexcept;

    enum class PairCategory : uint8_t
    {
        Analytic = 0,
        Sat = 1,
        GjkEpa = 2,
        Count = 3
    };

    struct alignas(16) CandidatePair
    {
        uint32_t bodyA;
        uint32_t bodyB;
        uint32_t bodyIndexA;
        uint32_t bodyIndexB;
    };

    struct Body3D;
    struct Contact3D;
    struct Body2D;
    struct Contact2D;
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