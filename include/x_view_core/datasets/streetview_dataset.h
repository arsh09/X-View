#ifndef X_VIEW_STREETVIEW_DATASET_H
#define X_VIEW_STREETVIEW_DATASET_H

#include <x_view_core/datasets/abstract_dataset.h>

#include <glog/logging.h>

namespace x_view {

/**
 * \brief This class carries information about the Streetview dataset.
 */
class StreetviewDataset : public AbstractDataset {

 public:
  StreetviewDataset();

  virtual ~StreetviewDataset() {}

  virtual cv::Mat convertSemanticImage(
      const sensor_msgs::ImageConstPtr& msg) const override;

  virtual const x_view::real_t getDepth(const cv::Point2i& pixel,
                                        const cv::Mat& depth_image) const
                                            override;

  virtual const std::string datasetName() const override {
    return std::string("Streetview Dataset");
  }
};

}

#endif //X_VIEW_STREETVIEW_DATASET_H
