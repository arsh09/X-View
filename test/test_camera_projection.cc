#include "test_camera_projection.h"

#include <x_view_core/landmarks/graph_landmark/depth_projector.h>
#include <x_view_core/x_view_tools.h>

#include <glog/logging.h>

#include <random>

namespace x_view_test {

void transformsExample() {

  // Position of robot expressed in world coordinates.
  Eigen::Vector3d robot_position_in_world_frame;
  robot_position_in_world_frame << 10.0, 50.0, 200.0;

  Eigen::Matrix3d rotation;
  rotation << 0.0, 0.0, 1.0,
              1.0, 0.0, 0.0,
              0.0, 1.0, 0.0;

  Eigen::Quaternion<double> q(rotation);
  x_view::SE3 pose(q, robot_position_in_world_frame);

  std::cout << "Pose:\n" << pose << std::endl;
  std::vector<x_view::Vector3r> positions {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0}
  };
  for(const auto& v : positions) {
    std::cout << x_view::RowVector3r(v) << " in robot frame expressed in "
        "world's frame: "
        << x_view::RowVector3r(
            pose.transform(v.cast<double>()).cast<x_view::real_t>())
        << std::endl;
    std::cout << x_view::RowVector3r(v) << " in world frame expressed in "
        << "robot's frame: "
        << x_view::RowVector3r(
            pose.inverseTransform(v.cast<double>()).cast<x_view::real_t>())
        << std::endl << std::endl;
  }
}

void testRandomCameraPose() {
  const x_view::real_t tol = 0.1;

  // Random number generator
  const uint64_t seed = 0;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<x_view::real_t> pos_rand(-5, 5);
  std::uniform_real_distribution<x_view::real_t> angle_rand(0.0, 1.0);

  // Static world object coordinates.
  x_view::Vector3r object_in_world_coordinates(0.0, 0.0, 15.0);

  const int image_width = 1240;
  const int image_height = 760;
  Eigen::Vector2i principal_point;
  principal_point << image_width / 2, image_height / 2;
  const x_view::real_t focal_length = 400.0;
  x_view::CameraIntrinsics intrinsics(focal_length, principal_point);

  const int num_reps = 1000;
  for (int i = 0; i < num_reps; ++i) {
    // Random robot position.
    x_view::Vector3r robot_position;
    robot_position << pos_rand(rng), pos_rand(rng), pos_rand(rng);

    // Random robot orientation with pure random rotation around Z axis
    // (depth direction of camera), and small rotation around other axis to
    // prevent too large distortions (e.g. when object is at 90deg on the
    // side, then the projection on the image frame is very unstable).
    x_view::Matrix3r robot_in_world_frame;
    robot_in_world_frame =
        Eigen::AngleAxis<x_view::real_t>(0.5 * angle_rand(rng),
                                       x_view::Vector3r::UnitX())*
        Eigen::AngleAxis<x_view::real_t>(0.5 * angle_rand(rng),
                                       x_view::Vector3r::UnitY())*
        Eigen::AngleAxis<x_view::real_t>(2 * M_PI * angle_rand(rng),
                                  x_view::Vector3r::UnitZ());

    // Pose construction.
    Eigen::Quaternion<x_view::real_t> q(robot_in_world_frame);
    x_view::SE3 pose(q.cast<double>(), robot_position.cast<double>());

    const x_view::real_t depth =
        x_view::dist(object_in_world_coordinates, robot_position);

    x_view::DepthProjector projector(pose, intrinsics);

    const cv::Point2i pixel_coord =
        projector.getPixelCoordinates(object_in_world_coordinates);

    const x_view::Vector3r new_world_coord =
        projector.getWorldCoordinates(pixel_coord, depth);

    for (int k = 0; k < 3; ++k)
      CHECK_NEAR(object_in_world_coordinates[k], new_world_coord[k], tol);

  }
}

void testPixelToCamera() {
  // Random number generator
  const uint64_t seed = 0;
  const int image_width = 1240;
  const int image_height = 760;
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> rand_x(0, image_width-1);
  std::uniform_int_distribution<int> rand_y(0, image_height-1);

  const int focal_length = 1000;
  Eigen::Vector2i principal_point;
  principal_point << image_width / 2, image_height / 2;
  const x_view::real_t depth = 1.0;
  x_view::CameraIntrinsics intrinsics(focal_length, principal_point);

  const int num_reps = 1000;
  for (int i = 0; i < num_reps; ++i) {
    Eigen::Vector2i pixel;
    pixel << rand_x(rng), rand_y(rng);

    const Eigen::Vector2i diff = pixel - principal_point;
    const x_view::real_t l_x =
        std::sqrt(static_cast<x_view::real_t>(
                      diff.x() * diff.x() + focal_length * focal_length));
    const x_view::real_t l_y =
        std::sqrt(static_cast<x_view::real_t>(
                      diff.y() * diff.y() + focal_length * focal_length));

    const x_view::real_t expected_x = diff.x() / l_x * depth;
    const x_view::real_t expected_y = diff.y() / l_y * depth;

    x_view::DepthProjector newProjector(x_view::SE3(), intrinsics);
    x_view::Vector3r proj = newProjector.pixelToCamera(pixel, depth);

    CHECK_NEAR(proj[0], expected_x, 0.05);
    CHECK_NEAR(proj[1], expected_y, 0.05);
  }
}

}

