#include <gtest/gtest.h>

#include "test_camera_projection.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/x_view_locator.h>

#include <glog/logging.h>

using namespace x_view_test;

TEST(XViewSlamTestSuite, test_camera_projection) {

  LOG(INFO) << "\n\n====Testing camera projection====";

  // Instantiate a fake dataset containing all the camera parameters used
  // for the tests.
  std::unique_ptr<x_view::AbstractDataset> dataset(
      new x_view::AbstractDataset(0));
  x_view::Locator::registerDataset(std::move(dataset));

  testPixelToCamera();
  testRandomCameraPose();

}


