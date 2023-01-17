/* This test computes the graph representation of different images and tests
 * various properties associated to them.
 * Such properties could be the number of pixels associated to each semantic
 * label or the uniqueness of pixel in the blob datastructure in the
 * graphLandmark objects
 *
 * The implementation of the tests is located in 'test_graph_landmark.cc'
 */

#include "test_graph_landmark.h"

#include <x_view_core/x_view_locator.h>

#include <gtest/gtest.h>

using namespace x_view;
using namespace x_view_test;

TEST(XViewSlamTestSuite, test_graph_landmark) {

  LOG(INFO) << "\n\n====Testing graph landmark====";

  const auto& parameters = Locator::getParameters();
  const auto& landmark_parameters =
      parameters->getChildPropertyList("landmark");

  // Set the minimum blob size to zero because for testing we don't want to
  // ignore any generated blob.
  landmark_parameters->setInteger("min_blob_size", 0);
  landmark_parameters->setBoolean("dilate_and_erode", false);

  // test different images
  testCustomImage();
  testDiscImage();

  // Close all windows.
  cv::destroyAllWindows();
}

