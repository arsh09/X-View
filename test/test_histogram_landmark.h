#ifndef X_VIEW_TEST_HISTOGRAM_LANDMARK_H
#define X_VIEW_TEST_HISTOGRAM_LANDMARK_H

#include <x_view_core/x_view_types.h>

#include <vector>
#include <utility>

using namespace x_view;

namespace x_view_test {

void performLabelTest(const SemanticLandmarkPtr& landmark_,
                      const std::vector<std::pair<int, real_t>>& expected);
}

#endif //X_VIEW_TEST_HISTOGRAM_LANDMARK_H
