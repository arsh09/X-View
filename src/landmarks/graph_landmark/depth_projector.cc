#include <x_view_core/landmarks/graph_landmark/depth_projector.h>

namespace x_view {

DepthProjector::DepthProjector(const x_view::SE3& pose,
                     const CameraIntrinsics& intrinsics,
                     const Matrix3r& camera_to_image_rotation)
    : pose_(pose),
      intrinsic_matrix_(intrinsics.getCameraMatrix()),
      inverse_intrinsic_matrix_(intrinsics.getCameraMatrix().inverse()),
      camera_to_image_rotation_(camera_to_image_rotation),
      image_to_camera_rotation_(camera_to_image_rotation_.inverse()) {
}

Vector3r DepthProjector::getWorldCoordinates(const cv::Point2i& pixel,
                                         const real_t depth) const {
  const Vector3r new_camera_coord =
      pixelToCamera(Eigen::Vector2i(pixel.x, pixel.y), depth);

  const Vector3r new_world_coord = cameraToWorld(new_camera_coord);

  return new_world_coord;
}

cv::Point2i DepthProjector::getPixelCoordinates(const Vector3r& coordinate) const {
  const Vector3r camera_coords =  worldToCamera(coordinate);

  const Eigen::Vector2i pixel_coords = cameraToPixel(camera_coords);

  return cv::Point2i(pixel_coords[0], pixel_coords[1]);
}

Vector3r DepthProjector::worldToCamera(const Vector3r& world_coordinate) const {
    return pose_.inverseTransform(world_coordinate.cast<double>()).cast<real_t>();
}

Eigen::Vector2i DepthProjector::cameraToPixel(
    const Vector3r& camera_coordinate) const {

  Vector3r image_coordinate = camera_to_image_rotation_ * camera_coordinate;
  Vector3r pixel_homo = intrinsic_matrix_ * image_coordinate;

  return Eigen::Vector2i(pixel_homo[0]/pixel_homo[2],
                         pixel_homo[1] / pixel_homo[2]);
}

Vector3r DepthProjector::pixelToCamera( const Eigen::Vector2i& pixel_coordinate,
                                    const real_t depth) const {
  Vector3r pixel_homo;
  pixel_homo << pixel_coordinate[0], pixel_coordinate[1], 1.0;

  Vector3r camera_direction = (inverse_intrinsic_matrix_ * pixel_homo).normalized();

  return image_to_camera_rotation_ * camera_direction * depth;
}

Vector3r DepthProjector::cameraToWorld(const Vector3r& camera_coordinate) const {
  return pose_.transform(camera_coordinate.cast<double>()).cast<real_t>();
}

}

