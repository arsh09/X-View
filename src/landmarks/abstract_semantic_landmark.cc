#include <x_view_core/landmarks/abstract_semantic_landmark.h>

namespace x_view {

AbstractSemanticLandmark::AbstractSemanticLandmark(
    const FrameData& frame_data)
    : semantic_image_(frame_data.getSemanticImage().clone()),
    depth_image_(frame_data.getDepthImage().clone()),
    pose_(frame_data.getPose()) {
}

AbstractSemanticLandmark::~AbstractSemanticLandmark() {}

const cv::Mat& AbstractSemanticLandmark::getSemanticImage() const {
  return semantic_image_;
}

const cv::Mat& AbstractSemanticLandmark::getDepthImage() const {
  return depth_image_;
}

const SE3& AbstractSemanticLandmark::getPose() const {
  return pose_;
}

const ConstDescriptorPtr& AbstractSemanticLandmark::getDescriptor() const {
  return descriptor_;
}

}
