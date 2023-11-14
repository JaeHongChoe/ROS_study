#ifndef LANE_DETECTOR_HPP_
#define LANE_DETECTOR_HPP_

#include "opencv2/opencv.hpp"
#include <yaml-cpp/yaml.h>

/// create your lane detecter
/// Class naming.. it's up to you.
namespace Xycar {
template <typename PREC>
class LaneDetector final
{
public:
    using Ptr = LaneDetector*; /// < Pointer type of the class(it's up to u)

    static inline const cv::Scalar kRed = {0, 0, 255}; /// Scalar values of Red
    static inline const cv::Scalar kGreen = {0, 255, 0}; /// Scalar values of Green
    static inline const cv::Scalar kBlue = {255, 0, 0}; /// Scalar values of Blue
    static inline const cv::Scalar kBlack = {0, 0, 0}; /// Scalar values of Blue
    static inline const cv::Scalar kWhite = {255, 255, 255}; /// Scalar values of Blue

    LaneDetector(const YAML::Node& config) {setConfiguration(config);}
    int32_t GetWidth(){return mImageWidth;}
    int32_t GetHeight(){return mImageHeight;}
private:
    int32_t mImageWidth;
    int32_t mImageHeight;

    // Debug Image and flag
    cv::Mat mDebugFrame; /// < The frame for debugging
    void setConfiguration(const YAML::Node& config);
    bool mDebugging;
};
}

#endif // LANE_DETECTOR_HPP_