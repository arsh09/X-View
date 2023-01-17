#ifndef X_VIEW_AIRSIM_DATASET_H
#define X_VIEW_AIRSIM_DATASET_H

#include <x_view_core/datasets/abstract_dataset.h>

#include <glog/logging.h>

namespace x_view {

/**
 * \brief This class carries information about the Airsim dataset.
 * \note Airsim dataset available at https://github.com/Microsoft/AirSim.
 */
class AirsimDataset : public AbstractDataset {

 public:
  AirsimDataset();

  virtual ~AirsimDataset() {}

  virtual cv::Mat convertSemanticImage(
      const sensor_msgs::ImageConstPtr& msg) const override;

  virtual const x_view::real_t getDepth(const cv::Point2i& pixel,
                                        const cv::Mat& depth_image) const
                                            override;

  virtual const std::string datasetName() const override {
    return std::string("Airsim Dataset");
  }
};

}

#endif //X_VIEW_AIRSIM_DATASET_H
