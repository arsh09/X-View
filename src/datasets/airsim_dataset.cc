#include <x_view_core/datasets/airsim_dataset.h>
#include <x_view_core/x_view_tools.h>

namespace x_view {

#define AIRSIM_NUM_SEMANTIC_CLASSES 13

AirsimDataset::AirsimDataset()
: AbstractDataset(AIRSIM_NUM_SEMANTIC_CLASSES) {
  // See https://github.com/ethz-asl/x-view/tree/master/dataset_tools/airsim_to_rosbag
  // for the available class labels in our Airsim datasets.

  // Initialize the entities as:
  // {name, id, is_to_include_in_graph, is_static, is_to_render}
  semantic_entities_ = {
      {"misc", 0, false, true, false},
      {"street", 1, true, true, true},
      {"building", 2, true, true, true},
      {"car", 3, true, true, true},
      {"sign", 4, true, true, true},
      {"fence", 5, true, true, true},
      {"hedge", 6, true, true, true},
      {"tree", 7, true, true, true},
      {"wall", 8, true, true, true},
      {"bench", 9, true, true, true},
      {"powerline", 10, false, true, true},
      {"rock", 11, true, true, true},
      {"pool", 12, true, true, true}
  };

  CHECK(semantic_entities_.size() == AIRSIM_NUM_SEMANTIC_CLASSES)
  << "Number of defined semantic entities differs from the one "
  << "specified in the header file:\n\tAIRSIM_NUM_SEMANTIC_CLASSES = "
  << AIRSIM_NUM_SEMANTIC_CLASSES << "\n\tdefined entities = "
  << semantic_entities_.size();

  // Set the intrinsic parameters for the Airsim dataset.
  const real_t airsim_focal_length = 512;
  const int airsim_px = 512;
  const int airsim_py = 288;
  camera_intrinsics_ = CameraIntrinsics(airsim_focal_length,
                                        airsim_px, airsim_py);

  // Set up camera-to-image rotation.
  Matrix3r rotation;
  rotation <<
      0.0, -1.0,  0.0,
      0.0,  0.0, -1.0,
      1.0,  0.0,  0.0;


  camera_to_image_rotation_ = rotation;
}

cv::Mat AirsimDataset::convertSemanticImage(
    const sensor_msgs::ImageConstPtr& msg) const {

  const int msg_size = static_cast<int>(msg->data.size());
  const int step_size = msg->step;

  CHECK(!bool(msg->is_bigendian))
  << "Message passed to AirsimDataset must be little endian";

  const int cols = msg->width;
  const int rows = msg->height;

  // New image used as container for semantic labels and instances.
  // the first channel of this new image contains the semantic label
  // associated to each pixel
  // the second channel contains a unique ID associated to dynamic objects
  // the third channel is not used.
  cv::Mat labelImage(rows, cols, CV_8UC3, cv::Scalar::all(0));

  // Loop over the rows of the image implicitly stored into msg.
  for (int i = 0; i < rows; ++i) {
    // Loop over the cols of the image implicitly stored into msg.
    for (int j = 0; j < cols; ++j) {
      // Index of the pixel, need to have "3*j" because each pixel value is
      // stored into a single byte and there are three channels.
      int idx = step_size * i + 3 * j;
      CHECK(idx < msg_size)
      << "Computed index is larger or equal to message size";

      cv::Vec3b values;
      values[2] = static_cast<uchar>(0);
      values[1] = static_cast<uchar>(twoBytesToInt(&(msg->data[idx + 1])) - 1);
      values[0] = static_cast<uchar>(msg->data[idx + 2]);

      labelImage.at<cv::Vec3b>(cv::Point2i(j, i)) = values;
    }

  }
  return labelImage;
}

const x_view::real_t AirsimDataset::getDepth(const cv::Point2i& pixel,
                                             const cv::Mat& depth_image) const {
  const uint8_t point_depth = depth_image.at<uint8_t>(pixel);
  return point_depth * 100.0 / 256.0;
}

#undef AIRSIM_NUM_SEMANTIC_CLASSES

}
