// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file LaneDetector.cpp
 * @author Jeongmin Kim
 * @author Jeongbin Yim
 * @brief lane detector class source file
 * @version 2.1
 * @date 2023-10-13
 */

#include <numeric>
#include "LaneKeepingSystem/LaneDetector.hpp"

namespace Xycar {

template <typename PREC>
void LaneDetector<PREC>::setConfiguration(const YAML::Node& config)
{
    mImageWidth = config["IMAGE"]["WIDTH"].as<int32_t>();
    mImageHeight = config["IMAGE"]["HEIGHT"].as<int32_t>();

    /**
     * If you want to add your parameter
     * declare your parameter.
     * */

    mDebugging = config["DEBUG"].as<bool>();
}

/*
Example Function Form
template <typename PREC>
void LaneDector<PREC>::yourOwnFunction()
{
    write your code.
    &
    you must specify your own function to your LaneDetector.hpp file.
}
*/

template class LaneDetector<float>;
template class LaneDetector<double>;
} // namespace Xycar
