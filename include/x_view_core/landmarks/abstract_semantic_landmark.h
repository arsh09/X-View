#ifndef X_VIEW_ABSTRACT_SEMANTIC_LANDMARK_H
#define X_VIEW_ABSTRACT_SEMANTIC_LANDMARK_H

#include <x_view_core/x_view_types.h>

#include <opencv2/core/core.hpp>

namespace x_view {

/**
 * \brief Internal representation of a semantic landmark. Each landmark type
 * used in XView must implement this interface.
 * \details A SemanticLandmark is an object created whenever new semantic
 * data is given to XView. In particular a semanticLandmark might contain
 * data about the robot's pose and an internal semantic representation of
 * what the robot experiences in that moment.
 * \note The AbstractSemanticLandmark copies the data from the arguments
 * passed to the constructor and holds a copy as member variable.
 */
class AbstractSemanticLandmark {

 public:
  /**
   * \brief When a landmark is initialized, it must directly compute its
   * internal representation.
   * \param frame_data Data associated to the landmark being constructed.
   */
  AbstractSemanticLandmark(const FrameData& frame_data);
  virtual ~AbstractSemanticLandmark();

  /**
   * \brief Returns a const reference to the semantic image associated with
   * this landmark.
   * \note Since cv::Mats are implemented as pointers to the actual image
   * data, if you modify the image returned by this method, you will also
   * modify the image stored by the landmark instance! In order to safely
   * modify the returned image without modifying the one stored in this
   * landmark instance proceed as follows:
   * \code{.cpp}
   * // This modifies the semantic image contained inside the landmark!
   * // It is only a shallow copy of the image.
   * cv::Mat unsafe_copy = landmark.getSemanticImage();
   * unsafe_copy.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
   *
   * // This is a safe deep copy of the semantic image.
   * cv::Mat safe_copy = landmark.getSemanticImage().clone();
   * // This change only affects the safe_copy cv::Mat and not the one stored
   * // inside the landmark.
   * safe_copy.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
   * \endcode
   */
  const cv::Mat& getSemanticImage() const;

  /**
   * \brief Returns a const reference to the depth-image associated with this
   * landmark.
   * \note Since cv::Mats are implemented as pointers to the actual image
   * data, if you modify the image returned by this method, you will also
   * modify the image stored by the landmark instance! In order to safely
   * modify the returned image without modifying the one stored in this
   * landmark instance proceed as follows:
   * \code{.cpp}
   * // This modifies the depth image contained inside the landmark!
   * // It is only a shallow copy of the image.
   * cv::Mat unsafe_copy = landmark.getDepthImage();
   * unsafe_copy.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
   *
   * // This is a safe deep copy of the depth image.
   * cv::Mat safe_copy = landmark.getDepthImage().clone();
   * // This change only affects the safe_copy cv::Mat and not the one stored
   * // inside the landmark.
   * safe_copy.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
   * \endcode
   */
  const cv::Mat& getDepthImage() const;

  /// \brief Returns a const reference to the robot's pose associated with
  /// this landmark.
  const SE3& getPose() const;

  /// \brief Returns a const reference to the stored descriptor representation.
  const ConstDescriptorPtr& getDescriptor() const;

 protected:
  /// \brief Semantic image given as input for the landmark.
  const cv::Mat semantic_image_;

  /// \brief Depth image given as input for the landmark.
  const cv::Mat depth_image_;

  /// \brief Robot's pose associated to this semantic landmark.
  const SE3 pose_;

  /// \brief internal representation of descriptor extracted in this landmark.
  ConstDescriptorPtr descriptor_;

}; // AbstractSemanticLandmark

}
#endif //X_VIEW_ABSTRACT_SEMANTIC_LANDMARK_H
