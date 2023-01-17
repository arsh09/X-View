#include <x_view_core/datasets/synthia_dataset.h>

#include <x_view_core/x_view_tools.h>

namespace x_view {

#define SYNTHIA_NUM_SEMANTIC_CLASSES 13

SynthiaDataset::SynthiaDataset()
    : AbstractDataset(SYNTHIA_NUM_SEMANTIC_CLASSES) {
  // see http://synthia-dataset.net/table-classes/ for class labels

  // Initialize the entities as:
  // {name, id, is_to_include_in_graph, is_static, is_to_render}
  semantic_entities_ = {
      {"misc", 0, false, true, false},
      {"sky", 1, false, true, false},
      {"building", 2, true, true, true},
      {"road", 3, true, true, true},
      {"sidewalk", 4, true, true, true},
      {"fence", 5, true, true, true},
      {"vegetation", 6, true, true, true},
      {"pole", 7, true, true, true},
      {"car", 8, true, false, true},
      {"sign", 9, true, true, true},
      {"pedestrian", 10, false, false, true},
      {"cyclist", 11, false, false, true},
      {"lanemarking", 12, false, true, true}
  };

  CHECK(semantic_entities_.size() == SYNTHIA_NUM_SEMANTIC_CLASSES)
  << "Number of defined semantic entities differs from the one "
  << "specified in the header file:\n\tSYNTHIA_NUM_SEMANTIC_CLASSES = "
  << SYNTHIA_NUM_SEMANTIC_CLASSES << "\n\tdefined entities = "
  << semantic_entities_.size();

  // Set the intrinsic parameters for the Synthia dataset.
  const real_t synthia_focal_length = 532.740352;
  const int synthia_px = 640;
  const int synthia_py = 380;
  camera_intrinsics_ = CameraIntrinsics(synthia_focal_length,
                                        synthia_px, synthia_py);

  // Set up camera-to-image rotation.
  // Flipped z-axis and flipped y-axis
  camera_to_image_rotation_ << 1.0,  0.0,  0.0,
                               0.0, -1.0,  0.0,
                               0.0,  0.0, -1.0;
}

cv::Mat SynthiaDataset::convertSemanticImage(
    const sensor_msgs::ImageConstPtr& msg) const {

  const int msg_size = static_cast<int>(msg->data.size());
  const int step_size = msg->step;

  CHECK(!bool(msg->is_bigendian))
  << "Message passed to SynthiaDataset must be little endian";

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
      // Index of the pixel, need to have "6*j" because each pixel value is
      // stored into two consecutive bytes and there are three channels.
      int idx = step_size * i + 6 * j;
      CHECK(idx < msg_size)
      << "Computed index is larger or equal to message size";

      cv::Vec3b values;
      values[2] = static_cast<uchar>(0);
      values[1] = static_cast<uchar>(twoBytesToInt(&(msg->data[idx + 2])));
      values[0] = static_cast<uchar>(
          std::max(
              0,
              std::min(
                  twoBytesToInt(&(msg->data[idx + 2 * 2])),
                  numSemanticClasses() - 1
              )
          )
      );

      labelImage.at<cv::Vec3b>(cv::Point2i(j, i)) = values;
    }

  }
  return labelImage;
}

const x_view::real_t SynthiaDataset::getDepth(const cv::Point2i& pixel,
                                              const cv::Mat& depth_image) const {
  const unsigned short point_depth_cm = depth_image.at<unsigned short>(pixel);
  return point_depth_cm * static_cast<real_t>(0.01);
}

#undef SYNTHIA_NUM_SEMANTIC_CLASSES

}
