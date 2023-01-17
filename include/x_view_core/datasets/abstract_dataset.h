#ifndef X_VIEW_ABSTRACT_DATASET_H
#define X_VIEW_ABSTRACT_DATASET_H

#include <x_view_core/x_view_types.h>

#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <vector>
#include <string>

namespace x_view {

class AbstractDataset {
 public:

  /**
   * \brief A semantic entity is represented by a human readable name and by
   * an ID ranging from 0 to num_semantic_classes_ - 1.
   */
  struct SemanticEntity {
    SemanticEntity() {}
    SemanticEntity(const std::string& name, const int id,
                   const bool is_to_include_in_graph = true,
                   const bool is_static = true,
                   const bool is_to_render = true)
        : semantic_entity_name(name),
          semantic_entity_id(id),
          is_to_include_in_graph(is_to_include_in_graph),
          is_static(is_static),
          is_to_render(is_to_render) {}

    /// \brief Name associated to the semantic entity.
    std::string semantic_entity_name;
    /// \brief Integer key (label) associated to the semantic entity.
    int semantic_entity_id;
    /// \brief Flag indicating if this entity has to be included in the
    /// semantic graph construction or not.
    bool is_to_include_in_graph;
    /// \brief Flag indicating if this entity is of static type or not.
    bool is_static;
    /// \brief Flag indicating if blobs associated with this entity are to be
    /// rendered when displaying semantic images or not.
    bool is_to_render;
  };

  AbstractDataset(const int num_semantic_classes);
  virtual ~AbstractDataset() {}

  /**
   * \brief Dataset name.
   * \return Human readable name description of dataset.
   */
  virtual const std::string datasetName() const {
    return std::string("Abstract Dataset");
  };

  ///\brief Returns the number of semantic classes contained in the dataset.
  int numSemanticClasses() const { return num_semantic_classes_; }

  /// \brief Returns a reference to the camera intrinsic parameters.
  const CameraIntrinsics& getCameraIntrinsics() const  {
    return camera_intrinsics_;
  }

  /// \brief Returns a reference to the rotation matrix between camera and
  /// image frame.
  const Matrix3r& getCameraToImageRotation() const {
    return camera_to_image_rotation_;
  }

  ///\brief Returns the semantic entities associated to this dataset.
  const std::vector<SemanticEntity>& semanticEntities() const {
    return semantic_entities_;
  }

  ///\brief Returns the label (string) associated to a given index.
  const std::string& label(const int index) const {
    CHECK(index >= 0 && index < num_semantic_classes_);
    return semantic_entities_[index].semantic_entity_name;
  }

  /**
   * \brief returns depth value of pixel, depending on the dataset as depth
   * may be encoded differently depending on the dataset.
   * \param pixel The pixel location.
   * \param depth_image The corresponding depth image.
   * \return Depth value of the pixel in [m].
   */
  virtual const x_view::real_t getDepth(const cv::Point2i& pixel,
                                        const cv::Mat& depth_image) const;

  /**
   * \brief Function called by ROS each time a new semantic image is available.
   * \details This function is called by the x_view worker before passing the
   * image to the x_view.
   */
  virtual cv::Mat convertSemanticImage(const sensor_msgs::ImageConstPtr&
  msg) const;

  /// \brief Since some class labels are too general (e.g. SYNTHIA::MISC),
  /// this function returns a vector of labels that are not, thus labels that
  /// one might want to render.
  virtual const std::vector<int> getLabelsToRender() const;

  /// \brief Returns a vector containing the index of the semantic entities
  /// being static.
  virtual const std::vector<int> getStaticLabels() const;

  /// \brief Returns a vector containing the index of the semantic entities
  /// being dynamic.
  virtual const std::vector<int> getDynamicLabels() const;

  /// \brief Returns a vector containing the index of the semantic entities
  /// to be included in the semantic graph construction. Unimportant or
  /// generic labels are excluded from the semantic graph, as they don't add
  /// any information to the scene.
  virtual const std::vector<int> getLabelsToIncludeInGraph() const;

  /// \brief Returns the length of the longest label in the dataset. This
  /// function is used for formatting the output.
  const uint64_t largestLabelSize() const;

 protected:
  const int num_semantic_classes_;
  std::vector<SemanticEntity> semantic_entities_;
  CameraIntrinsics camera_intrinsics_;
  Matrix3r camera_to_image_rotation_;
};

/**
 * \brief Streams the dataset in a human readable way to the passed
 * stream argument.
 * \param out Stream object to be streamed to.
 * \param dataset AbstractDataset object to be streamed.
 * \return Stream filled with AbstaractDataset.
 */
std::ostream& operator<<(std::ostream& out, const AbstractDataset& dataset);

/// \brief Overloaded operator to print using pointer.
std::ostream& operator<<(std::ostream& out,
                         const std::unique_ptr<AbstractDataset>& ptr);

}

#endif //X_VIEW_ABSTRACT_DATASET_H
