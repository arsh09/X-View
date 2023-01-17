#include <gtest/gtest.h>

#include "test_graph_landmark_matcher.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/x_view_locator.h>

using namespace x_view;
using namespace x_view_test;

TEST(XViewSlamTestSuite, test_graph_landmark_matcher) {

  LOG(INFO) << "\n\n====Testing graph landmark matcher====";

  const int num_semantic_classes = 13;
  LOG(INFO) << "Testing graph landmark matcher with " << num_semantic_classes
            << "classes.";

  std::unique_ptr<AbstractDataset> dataset(
      new AbstractDataset(num_semantic_classes));
  Locator::registerDataset(std::move(dataset));

  const uint64_t seed = 0;

  testChainGraph(seed);
  testRandomGraph(seed);

  // Close all windows.
  cv::destroyAllWindows();
}

