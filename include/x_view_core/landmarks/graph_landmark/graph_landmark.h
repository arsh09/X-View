#ifndef X_VIEW_GRAPH_LANDMARK_H_
#define  X_VIEW_GRAPH_LANDMARK_H_

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/features/graph_descriptor.h>
#include <x_view_core/features/graph.h>
#include <x_view_core/landmarks/abstract_semantic_landmark.h>
#include <x_view_core/landmarks/graph_landmark/graph_landmark_types.h>
#include <x_view_core/x_view_types.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace x_view {

/**
 * \brief A GraphLandmark represents a landmark by though a graph.
 */
class GraphLandmark : public AbstractSemanticLandmark {

 public:
  static SemanticLandmarkPtr create(const FrameData& frame_data) {
    return std::make_shared<GraphLandmark>(GraphLandmark(frame_data));
  }

  /// \brief Returns a const reference to the blob datastructure.
  const ImageBlobs& getBlobs() const {
    return image_blobs_;
  }

 protected:
  /**
   * \brief Performs a segmentation on the first channel the image passed as
   * argument to determine how semantic entities are represented in the scene.
   * A graph is built upon this segmentation defining its vertices as the
   * 'center of mass' of each blob of the image.
   * \param frame_data Data associated to the constructed landmark.
   */
  explicit GraphLandmark(const FrameData& frame_data);

 private:
  /**
   * \brief Structure containing the semantic segmentation of the
   * semantic_image_.
   * \details The dimensions of the structure are the following:
   * 'image_blobs_.size()' = dataset size (num semantic classes)
   * 'image_blobs_[i].size()' = number of disconnected instances of class 'i'
   * 'image_blobs_[i][j]size_' = number of pixels in
   * contour of 'j'-th instance of class 'i'.
   */
  ImageBlobs image_blobs_;

};

}

#endif // X_VIEW_GRAPH_LANDMARK_H_
