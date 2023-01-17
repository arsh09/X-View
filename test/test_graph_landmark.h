#ifndef X_VIEW_TEST_GRAPH_LANDMARK_H
#define X_VIEW_TEST_GRAPH_LANDMARK_H

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/features/graph_descriptor.h>
#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/x_view_types.h>

#include <opencv2/core/core.hpp>

using namespace x_view;

namespace x_view_test {

typedef std::shared_ptr<GraphLandmark> GraphLandmarkPtr;

/// \brief Test the custom image.
void testCustomImage();
/// \brief Test the disc image.
void testDiscImage();

/**
 * \brief Counts how many pixels with each label are present in the image.
 * \param image cv::Mat to be analyzed.
 * \param pixel_count vector filled up with the pixel count such that
 * 'pixelCount[i]' contains the number of pixels in 'image' having label 'i'.
 */
void countPixelLabelsInImage(const cv::Mat& image,
                             std::vector<uint64_t>& pixel_count);

/**
 * \brief Checks if the number of pixels for each class 'i' in 'image' is the
 * same between counting them explicitly and the one contained in the
 * graphLandmarkPointer.
 * \param graph_landmark_ptr pointer to the graphLandmark.
 * \param imageName logging image name.
 */
void testPixelCount(const GraphLandmarkPtr& graph_landmark_ptr,
                    const std::string& imageName);

/**
 * \brief Checks if the number of blobs per class found by the
 * graphLandmarkPrt object is the same as the expected one.
 * \param graph_landmark_ptr pointer to the graphLandmark.
 * \param expected_blob_count expected number of blobs per class, such
 * that 'expected_blob_count[i]' contains the number of expected blobs
 * for class 'i'.
 * \param image_name logging image name.
 */
void testBlobsCount(const GraphLandmarkPtr& graph_landmark_ptr,
                    const std::vector<int>& expected_blob_count,
                    const std::string& image_name);

/**
 * \brief Creates a custom image of size 'desired_rows' x 'desired_cols'.
 * \param desired_rows desired number of rows.
 * \param desired_cols desired number of cols.
 * \param image pointer to image to be filled up.
 */
void createCustomImage(const int desired_rows, const int desired_cols,
                       cv::Mat* image);

/**
 * \brief Creates an image containing a set of discs.
 * \param desired_rows desired number of rows.
 * \param desired_cols desired number of cols.
 * \param centers vector of disc centers (cv::Point2i).
 * \param radii vector of disc radii.
 * \param labels vector of label to associate to each disc.
 * \param image pointer to image to be filled up.
 */
void createDiscImage(const int desired_rows, const int desired_cols,
                     const std::vector<cv::Point2i>& centers,
                     const std::vector<int> radii,
                     const std::vector<int> labels, cv::Mat* image);

}

#endif //X_VIEW_TEST_GRAPH_LANDMARK_H
