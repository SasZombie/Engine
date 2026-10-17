#pragma once
#include <cstdint>

namespace sas
{

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

    struct DevicePairBins
    {
        CandidatePair *pairs[3];
        uint32_t *counters;
        uint32_t maxCapacity;
    };

} // namespace sas
