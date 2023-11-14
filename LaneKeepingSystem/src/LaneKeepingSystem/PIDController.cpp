#include "LaneKeepingSystem/PIDController.hpp"
namespace Xycar {

template <typename PREC>
PREC PIDController<PREC>::getControlOutput(int32_t errorFromMid)
{
    PREC castError = static_cast<PREC>(errorFromMid);
    mDifferentialGainError = castError - mProportionalGainError;
    mProportionalGainError = castError;
    mIntegralGainError += castError;
    return mProportionalGain * mProportionalGainError + mIntegralGain * mIntegralGainError + mDifferentialGain * mDifferentialGainError;
}

template class PIDController<float>;
template class PIDController<double>;
} // namespace Xycar