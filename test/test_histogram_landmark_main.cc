/* This test computes the histogram representation of three different images,
 * namely a black image, a white image and a half-black half-white image.
 * The test verifies that the HistogramLandmark class correctly extracts a
 * histogram representation of the image, i.e. it checks that the resulting
 * histograms contain the correct percentage of label '0' (black) and label
 * '1' (white).
 */

#include <gtest/gtest.h>
#include "test_histogram_landmark.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/landmarks/histogram_landmark.h>
#include <x_view_core/x_view_locator.h>
#include <x_view_core/x_view_types.h>

using namespace x_view;
using namespace x_view_test;

#define CAST(from, to) std::dynamic_pointer_cast<to>(from)

TEST(XViewSlamTestSuite, test_histogram_landmark) {

  LOG(INFO) << "\n\n====Testing histogram landmark====";

  // Initialize a fake dataset having num_semantic_classes classes.
  const int num_semantic_classes = 2;
  std::unique_ptr<AbstractDataset> dataset(
      new AbstractDataset(num_semantic_classes));
  Locator::registerDataset(std::move(dataset));

  // Create various images with following parameters.
  const uint64_t ROWS = 50;
  const uint64_t COLS = 30;
  const uint64_t frame_index = 0;

  // Black image.
  cv::Mat black(ROWS, COLS, CV_8UC3, cv::Scalar(0, 0, 0));
  FrameData frame_data_black(black, cv::Mat(), PoseId(), frame_index);
  SemanticLandmarkPtr black_land = HistogramLandmark::create(frame_data_black);
  // Expect to have 100% votes for label 0.
  std::vector<std::pair<int, real_t>> bExpected = {
      {0, 1.0}, {1, 0.0}
  };
  performLabelTest(black_land, bExpected);

  // White image.
  cv::Mat white(ROWS, COLS, CV_8UC3, cv::Scalar(1, 0, 0));
  FrameData frame_data_white(white, cv::Mat(), PoseId(), frame_index);
  SemanticLandmarkPtr white_land = HistogramLandmark::create(frame_data_white);
  // Expect to have 100% votes for label 1.
  std::vector<std::pair<int, real_t>> wExpected = {
      {0, 0.0}, {1, 1.0}
  };
  performLabelTest(white_land, wExpected);

  // Half white half black image.
  cv::Mat half(ROWS, COLS, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int i = 0; i < ROWS / 2; ++i) {
    for (int j = 0; j < COLS; ++j) {
      half.at<cv::Vec3b>(i, j) = cv::Vec3b(1, 0, 0);
    }
  }
  FrameData frame_data_half(half, cv::Mat(), PoseId(), frame_index);
  SemanticLandmarkPtr half_land = HistogramLandmark::create(frame_data_half);
  // Expect to have 50% votes for label 0 and 50% for label 1.
  std::vector<std::pair<int, real_t>> hExpected = {
      {0, 0.5}, {1, 0.5}, {2, 0.0}
  };
  performLabelTest(half_land, hExpected);

}

