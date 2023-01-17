#include "test_histogram_landmark.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/features/vector_descriptor.h>
#include <x_view_core/landmarks/histogram_landmark.h>

using namespace x_view;

namespace x_view_test {

typedef std::shared_ptr<HistogramLandmark> HistogramLandmarkPtr;

#define CAST(from, to) std::dynamic_pointer_cast<to>(from)

void performLabelTest(const SemanticLandmarkPtr& landmark_,
                      const std::vector<std::pair<int, real_t>>& expected) {

  HistogramLandmarkPtr landmark = CAST(landmark_, HistogramLandmark);

  cv::Mat hist =
      CAST(landmark->getDescriptor(), const VectorDescriptor)->getDescriptor();

  auto toVec = [](const cv::Mat& mat) -> std::vector<real_t> {
    std::vector<real_t> v(mat.cols, static_cast<real_t>(0.0));
    for (int i = 0; i < mat.cols; ++i) {
      v[i] = mat.at<real_t>(i);
    }
    return v;
  };

  auto vec = toVec(hist);

  for (int condition = 0; condition < expected.size(); ++condition) {
    const int binIndex = expected[condition].first;
    const real_t expectedPercentage = expected[condition].second;
    CHECK_DOUBLE_EQ(vec[binIndex], expectedPercentage);
  }
}

}
