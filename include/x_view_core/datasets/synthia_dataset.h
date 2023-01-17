#ifndef X_VIEW_SYNTHIA_DATASET_H
#define X_VIEW_SYNTHIA_DATASET_H

#include <x_view_core/datasets/abstract_dataset.h>

namespace x_view {

/**
 * \brief This class carries information about the Synthia dataset
 * \note Synthia dataset available at http://synthia-dataset.net/
 */
class SynthiaDataset : public AbstractDataset {

 public:
  SynthiaDataset();
  virtual ~SynthiaDataset() {}

  virtual const std::string datasetName() const override {
    return std::string("Synthia Dataset");
  }

  virtual cv::Mat convertSemanticImage(
      const sensor_msgs::ImageConstPtr& msg) const override;

  virtual const x_view::real_t getDepth(const cv::Point2i& pixel,
                                        const cv::Mat& depth_image) const override;

};

}

#endif //X_VIEW_SYNTHIA_DATASET_H
