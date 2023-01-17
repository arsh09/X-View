#ifndef X_VIEW_TEST_CAMERA_PROJECTION_H
#define X_VIEW_TEST_CAMERA_PROJECTION_H


namespace x_view_test {

void transformsExample();

/// \brief Tests the process of projecting an object given in world
/// coordinates to a pixel, and reprojecting that pixel into world
/// coordinates. The test checks is the reprojection into world space is
/// close to the original world coordinates of the object.
void testRandomCameraPose();

/// \brief Tests the process of computing the 3D position expressed in camera
/// frame given a pixel coordinate.
void testPixelToCamera();

}

#endif //X_VIEW_TEST_CAMERA_PROJECTION_H
