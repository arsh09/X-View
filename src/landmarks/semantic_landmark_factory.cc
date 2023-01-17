#include <x_view_core/landmarks/semantic_landmark_factory.h>

namespace x_view {

SemanticLandmarkFactory::CreateCallBack SemanticLandmarkFactory::callback_;

void SemanticLandmarkFactory::setCreatorFunction(CreateCallBack callback) {
  callback_ = callback;
}

SemanticLandmarkPtr SemanticLandmarkFactory::createSemanticLandmark(
    const FrameData& frame_data) {
  return callback_(frame_data);
}

}
