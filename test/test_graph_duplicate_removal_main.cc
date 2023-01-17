#include <gtest/gtest.h>

#include "test_graph_duplicate_removal.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/x_view_locator.h>

#include <glog/logging.h>

using namespace x_view_test;

TEST(XViewSlamTestSuite, test_graph_duplicate_removal) {

  LOG(INFO) << "\n\n====Testing graph duplicate removal====";

  // Instantiate a fake dataset.
  const int num_semantic_classes = 12;
  std::unique_ptr<x_view::AbstractDataset> dataset(
      new x_view::AbstractDataset(num_semantic_classes));
  x_view::Locator::registerDataset(std::move(dataset));

  // Perform the tests.
  testDuplicatesChain();

}




