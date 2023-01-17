#include "test_graph_landmark.h"

#include <x_view_core/x_view_locator.h>

namespace x_view_test {

#define CV_IMAGE_TYPE  CV_8UC3

#define CAST(from, to) std::dynamic_pointer_cast<to>(from)

void testCustomImage() {

  const std::string image_name = "custom image";

  // Initialize a fake dataset having num_semantic_classes classes.
  int num_semantic_classes = 4;
  std::unique_ptr<AbstractDataset> dataset(
      new AbstractDataset(num_semantic_classes));
  Locator::registerDataset(std::move(dataset));

#ifdef X_VIEW_DEBUG
  std::vector<int> sizes = {100, 200, 333};
#else
  std::vector<int> sizes = {100, 200, 333, 800, 1500};
#endif // X_VIEW_DEBUG

  for (auto size : sizes) {
    const int rows = size;
    const int cols = static_cast<int>(size * 1.5);
    LOG(INFO) << "Testing " << image_name << " for image size: ["
              << rows << " x " << cols << "]";

    cv::Mat custom_image;
    createCustomImage(rows, cols, &custom_image);
    cv::Mat depth_image =
        cv::Mat(custom_image.size(), CV_16U, cv::Scalar::all(0));
    PoseId pose_id;
    uint64_t frame_index = 0;
    FrameData frame_data(custom_image, depth_image, pose_id, frame_index);
    GraphLandmarkPtr graph_landmark_ptr =
        CAST(GraphLandmark::create(frame_data), GraphLandmark);

    CHECK_NOTNULL(graph_landmark_ptr.get());

    // Display the generated image.
#ifdef X_VIEW_DEBUG
    cv::Mat graph_image = GraphDrawer::createImageWithLabels
        (graph_landmark_ptr->getBlobs(),
         CAST(graph_landmark_ptr->getDescriptor(), const GraphDescriptor)
             ->getDescriptor(), custom_image.size());
    cv::imshow("Current graph landmark", graph_image);
    cv::waitKey();
#endif

    // Tests.
    testPixelCount(graph_landmark_ptr, image_name);
    testBlobsCount(graph_landmark_ptr, {1, 1, 1, 1}, image_name);

    LOG(INFO) << "Test passed.";
  }
}

void testDiscImage() {

  const std::string image_name = "disc image";
  // random number generator to generate discs
  cv::RNG rng(2);

  std::vector<int> classes = {2, 3, 5, 15};
  std::vector<int> num_discs = {2, 15};

  for (const int num_classes : classes) {
    std::unique_ptr<AbstractDataset> dataset(
        new AbstractDataset(num_classes));
    Locator::registerDataset(std::move(dataset));

    const int rows = 500;
    const int cols = static_cast<int>(500 * 1.5);
    const int diag = static_cast<int>(std::sqrt(rows * rows + cols * cols));
    for (const int num_disc : num_discs) {
      LOG(INFO) << "Testing " << image_name << " for image size: ["
                << rows << " x " << cols << "] and " << num_disc << " discs.";

      cv::Mat disc_image;
      std::vector<cv::Point2i> centers;
      std::vector<int> radii, labels;

      for (int i = 0; i < num_disc; ++i) {
        centers.push_back(cv::Point2i(rng.uniform(0, cols),
                                    rng.uniform(0, rows)));
        radii.push_back(std::max(15, rng.uniform(diag / 40, diag / 10)));
        labels.push_back(rng.uniform(1, num_classes));
      }

      createDiscImage(rows, cols, centers, radii, labels, &disc_image);
      cv::Mat depth_image =
          cv::Mat(disc_image.size(), CV_16U, cv::Scalar::all(0));
      PoseId pose_id;
      uint64_t frame_index = 0;
      FrameData frame_data(disc_image, depth_image, pose_id, frame_index);
      GraphLandmarkPtr graph_landmark_ptr =
          CAST(GraphLandmark::create(frame_data), GraphLandmark);

      CHECK_NOTNULL(graph_landmark_ptr.get());

      // Display the generated image.
#ifdef X_VIEW_DEBUG
      cv::Mat graph_image = GraphDrawer::createImageWithLabels
          (graph_landmark_ptr->getBlobs(),
           CAST(graph_landmark_ptr->getDescriptor(), const GraphDescriptor)
               ->getDescriptor(), disc_image.size());
      cv::imshow("Current graph landmark", graph_image);
      cv::waitKey();
#endif

      // Tests.
      testPixelCount(graph_landmark_ptr, image_name);

      LOG(INFO) << "Test passed.";
    }

  }

}

void countPixelLabelsInImage(const cv::Mat& image,
                             std::vector<uint64_t>& pixel_count) {
  const auto& dataset = Locator::getDataset();
  pixel_count.clear();
  pixel_count.resize(dataset->numSemanticClasses());
  for (int i = 0; i < image.rows; ++i) {
    for (int j = 0; j < image.cols; ++j) {
      int label = static_cast<int>(image.at<cv::Vec3b>(i, j)[0]);
      pixel_count[label]++;
    }
  }
}

void testPixelCount(const GraphLandmarkPtr& graph_landmark_ptr,
                    const std::string& imageName) {

  // vector counting explicitly the number of pixels
  std::vector<uint64_t> expected_pixel_count;
  countPixelLabelsInImage(graph_landmark_ptr->getSemanticImage(),
                          expected_pixel_count);

  for (int i = 0; i < expected_pixel_count.size(); ++i) {
    uint64_t semantic_class_pixel_count = 0;
    const auto& semantic_label_blobs = graph_landmark_ptr->getBlobs()[i];
    for (int j = 0; j < semantic_label_blobs.size(); ++j)
      semantic_class_pixel_count += semantic_label_blobs[j].num_pixels;
    CHECK_EQ(expected_pixel_count[i], semantic_class_pixel_count)
      << "In image " << imageName << ", class instance " << i
      << " should have " << expected_pixel_count[i] << " pixels, but has "
      << semantic_class_pixel_count;
  }
}

void testBlobsCount(const GraphLandmarkPtr& graph_landmark_ptr,
                    const std::vector<int>& expected_blob_count,
                    const std::string& image_name) {
  const auto& blobs = graph_landmark_ptr->getBlobs();
  for (int i = 0; i < expected_blob_count.size(); ++i) {
    CHECK_EQ(expected_blob_count[i], blobs[i].size())
      << "In image " << image_name << ", class " << i << " should have "
      << expected_blob_count[i] << " blobs, but has " << blobs[i].size();
  }
}

void createCustomImage(const int desired_rows, const int desired_cols,
                       cv::Mat* image) {

  CHECK_NOTNULL(image);

  image->create(desired_rows, desired_cols, CV_IMAGE_TYPE);
  for (int i = 0; i < desired_rows; ++i) {
    for (int j = 0; j < desired_cols; ++j) {
      if (i < desired_rows / 2) {
        if (j < desired_cols / 2) {
          image->at<cv::Vec3b>(i, j)[0] = static_cast<unsigned char>(0);
          image->at<cv::Vec3b>(i, j)[1] = static_cast<unsigned char>(50);
        } else {
          image->at<cv::Vec3b>(i, j)[0] = static_cast<unsigned char>(1);
          image->at<cv::Vec3b>(i, j)[1] = static_cast<unsigned char>(100);
        }
      } else {
        if (j < desired_cols / 2) {
          image->at<cv::Vec3b>(i, j)[0] = static_cast<unsigned char>(2);
          image->at<cv::Vec3b>(i, j)[1] = static_cast<unsigned char>(150);
        } else {
          image->at<cv::Vec3b>(i, j)[0] = static_cast<unsigned char>(3);
          image->at<cv::Vec3b>(i, j)[1] = static_cast<unsigned char>(200);
        }
      }
    }
  }
}

void createDiscImage(const int desired_rows, const int desired_cols,
                     const std::vector<cv::Point2i>& centers,
                     const std::vector<int> radii,
                     const std::vector<int> labels, cv::Mat* image) {

  CHECK_NOTNULL(image);

  image->create(desired_rows, desired_cols, CV_IMAGE_TYPE);
  *image = cv::Scalar::all(0);

  for (int c = 0; c < centers.size(); ++c) {
    const int label = labels[c];
    const int instance_id = c + 1;
    cv::circle(*image, centers[c], radii[c], cv::Scalar(label, instance_id, 0),
               -1, 8, 0);
  }

}

}
