#include <cassert>
#include <numeric>

#include "LaneKeepingSystem/MovingAverageFilter.hpp"

namespace Xycar {
template <typename PREC, FilteringMode Mode>
MovingAverageFilter<PREC, Mode>::MovingAverageFilter(uint32_t sampleSize) : mSampleSize(sampleSize)
{
    mWeights.resize(mSampleSize);
    std::iota(mWeights.begin(), mWeights.end(), 1);
}

template <typename PREC, FilteringMode Mode>
void MovingAverageFilter<PREC, Mode>::addSample(int32_t newSample)
{
    mSamples.emplace_back(newSample);
    if (mSampleSize < static_cast<uint32_t>(mSamples.size()))
        mSamples.pop_front();

    mFilteringResult = update(static_cast<uint32_t>(mSamples.size()));
}

template <typename PREC, FilteringMode Mode>
PREC MovingAverageFilter<PREC, Mode>::update(uint32_t sampleSize)
{
    int32_t sum = 0;
    int32_t denominator = 0;

    if (Mode == FilteringMode::NORMAL)
    {
        for (uint32_t i = 0; i < sampleSize; ++i)
            sum += mSamples[i];
        denominator = static_cast<int32_t>(sampleSize);
    }
    else if (Mode == FilteringMode::WEIGHTED)
    {
        for (uint32_t i = 0; i < sampleSize; ++i)
        {
            sum += mSamples[i] * mWeights[i];
            denominator += mWeights[i];
        }
    }

    assert(denominator != 0);
    return static_cast<PREC>(sum) / denominator;
}
template class MovingAverageFilter<float>;
template class MovingAverageFilter<double>;
template class MovingAverageFilter<float, FilteringMode::NORMAL>;
template class MovingAverageFilter<double, FilteringMode::NORMAL>;
} // namespace Xycar
