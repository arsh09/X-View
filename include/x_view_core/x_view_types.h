#ifndef X_VIEW_X_VIEW_TYPES_H
#define X_VIEW_X_VIEW_TYPES_H

#include <Eigen/Core>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <kindr/minimal/quat-transformation.h>
#include <opencv2/core/core.hpp>

#include <iomanip>
#include <memory>

namespace x_view {

// Single or double floating point precision.
// Change this MACRO to 0 to use single precision where possible.
#ifndef X_VIEW_USE_DOUBLE_PRECISION
#define X_VIEW_USE_DOUBLE_PRECISION 1
#endif

#if X_VIEW_USE_DOUBLE_PRECISION
typedef double real_t;
#else
typedef float real_t;
#endif

constexpr real_t real_eps = std::numeric_limits<real_t>::epsilon();

// Matrix types used in X-View.
typedef Eigen::Matrix<real_t, 3, 1> Vector3r;
typedef Eigen::Matrix<real_t, 1, 3> RowVector3r;
typedef Eigen::Matrix<real_t, 3, 3> Matrix3r;
typedef Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXr;
typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;
typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> MatrixXb;
typedef Eigen::Matrix<uchar, Eigen::Dynamic, Eigen::Dynamic> MatrixXuc;

// Forward declaration of classes.
class AbstractDataset;
class AbstractDescriptor;
class AbstractSemanticLandmark;
class AbstractMatcher;

/// \brief Factor graph used for graph optimization.
typedef gtsam::NonlinearFactorGraph FactorGraph;

/// \brief 3D pose (position + orientation).
typedef kindr::minimal::QuatTransformationTemplate<double> SE3;
typedef kindr::minimal::RotationQuaternionTemplate<double> SO3;

/// \brief Pair of 3D poses used in localization.
struct LocalizationPair {

  LocalizationPair() {}
  LocalizationPair(const x_view::SE3& true_pose,
                   const x_view::SE3& estimated_pose)
      : true_pose(true_pose),
        estimated_pose(estimated_pose) {}

  x_view::SE3 true_pose;
  x_view::SE3 estimated_pose;
};

/// \brief Pointer to feature.
typedef std::shared_ptr<const AbstractDescriptor> ConstDescriptorPtr;

/// \brief Pointer to semantic landmark.
typedef std::shared_ptr<AbstractSemanticLandmark> SemanticLandmarkPtr;

/// \brief Pointer to landmark matchers.
typedef std::shared_ptr<AbstractMatcher> LandmarksMatcherPtr;

struct PoseId {
  /// \brief Pose of the robot.
  SE3 pose;
  /// \brief Unique identifier for pose.
  uint64_t id;
};

/// \brief In each frame XView recieves an instance of FrameData containing
/// information about semantic segmentation, depth image and robot pose.
class FrameData {
 public:
  FrameData(const cv::Mat& semantic_image, const cv::Mat& depth_image,
            const PoseId& pose, const uint64_t id)
      : semantic_image_(semantic_image),
        depth_image_(depth_image),
        pose_(pose),
        id_(id) {
  }

  const cv::Mat& getSemanticImage() const {
    return semantic_image_;
  }

  const cv::Mat& getDepthImage() const {
    return depth_image_;
  }

  const SE3& getPose() const {
    return pose_.pose;
  }

  const PoseId& getPoseId() const {
    return pose_;
  }

  const uint64_t getID() const {
    return id_;
  }

 private:
  /// \brief Semantic image representing the semantic segmentation of the
  /// scene being observed. The first channel of this image contains label
  /// information ranging from 0 to num_semantic_classes -1.
  /// The second channel contains instance information of the single
  /// segmented entities.
  const cv::Mat& semantic_image_;

  /// \brief Depth image representing the depth (in cm) of the scene being
  /// observed. Each pixel's value is represented as a single unsigned short
  /// (16 bits) having maximum value of 65535.
  const cv::Mat& depth_image_;

  /// \brief Pose of the robot associated with this frame expressed in world
  /// coordinates.
  const PoseId& pose_;

  /// \brief Unique identifier associated with the index of the frame
  /// represented by this instance.
  const uint64_t id_;
};

/// \brief Camera intrinsic parameters used to compute the 3D location of a
/// pixel given robot pose.
class CameraIntrinsics {
 public:
  CameraIntrinsics(const real_t focal_length, const int px, const int py) {
    intrinsics_ = Matrix3r::Identity();
    intrinsics_(0, 0) = intrinsics_(1, 1) = focal_length;
    intrinsics_(0, 2) = px;
    intrinsics_(1, 2) = py;
  }

  CameraIntrinsics(const real_t focal_length, const Eigen::Vector2i& p)
      : CameraIntrinsics(focal_length, p[0], p[1]) {}

  const Matrix3r& getCameraMatrix() const {
    return intrinsics_;
  }

 private:
  Matrix3r intrinsics_;
};

}

#endif //X_VIEW_X_VIEW_TYPES_H
