#include <x_view_core/landmarks/graph_landmark/blob.h>
#include <x_view_core/x_view_types.h>

#include <glog/logging.h>
#include <nabo/nabo.h>

namespace x_view {

Blob::Blob() : semantic_label(-1), c_blob_() {
}

Blob::Blob(const int semantic_label, const int instance, const CBlob& c_blob)
    : semantic_label(semantic_label),
      instance(instance),
      c_blob_(c_blob) {
  computeContours();
  computeArea();
  computeBoundingBox();
  computeBlobCenter();
}

bool Blob::areNeighbors(const Blob& bi, const Blob& bj, const int distance,
                        const bool use_kd_tree) {

  // Compute squared pixel distance. Increase the distance by one, as if the
  // input parameter is distance=0 we want to allows neighborhood only
  // between touching blobs, i.e. blobs with an actual distance of 1.
  const int distance_squared = (distance + 1) * (distance + 1);

  // Increase the box dimension to avoid rejecting matches between blobs in
  // case they are close but their original bounding_box's don't touch.
  const cv::Point2i box_shift(distance, distance);
  const cv::Size2i box_expansion(2 * distance, 2 * distance);

  const cv::Rect& bounding_box_i =
      (bi.bounding_box - box_shift) + box_expansion;
  const cv::Rect& bounding_box_j =
      (bj.bounding_box - box_shift) + box_expansion;

  if ((bounding_box_i & bounding_box_j).area() == 0)
    return false;

  // External contours.
  const std::vector<cv::Point2i>& external_contour_i =
      bi.external_contour_pixels;
  const std::vector<cv::Point2i>& external_contour_j =
      bj.external_contour_pixels;

  // Internal contours.
  const std::vector<std::vector<cv::Point2i>>& internal_contours_i =
      bi.internal_contour_pixels;
  const std::vector<std::vector<cv::Point2i>>& internal_contours_j =
      bj.internal_contour_pixels;

  if(use_kd_tree) {
    // Compute closest distance between pixels in blob i and pixels in blob
    // by using a KD-tree.

    // Count the number of pixels belonging to blob i and blob j.

    // Number of external pixels.
    const uint64_t num_external_contour_pixels_i = external_contour_i.size();
    const uint64_t num_external_contour_pixels_j = external_contour_j.size();

    // Number of internal pixels.
    auto countNumInternalPixels =
        [](const std::vector<std::vector<cv::Point2i>>& internal_contours) {
          uint64_t num_internal_pixels = 0;
          for (const std::vector<cv::Point2i>
                & internal_contour : internal_contours) {
            num_internal_pixels += internal_contour.size();
          }
          return num_internal_pixels;
        };

    const uint64_t num_internal_contour_pixels_i =
        countNumInternalPixels(internal_contours_i);
    uint64_t num_internal_contour_pixels_j =
        countNumInternalPixels(internal_contours_j);

    // Total number of pixels.
    const uint64_t num_contour_pixels_i =
        num_external_contour_pixels_i + num_internal_contour_pixels_i;
    const uint64_t num_contour_pixels_j =
        num_external_contour_pixels_j + num_internal_contour_pixels_j;

    // Create matrices containing the contour pixels used in nearest neighbor
    // search.
    MatrixXr contour_i_matrix(2, num_contour_pixels_i);
    MatrixXr contour_j_matrix(2, num_contour_pixels_j);

    // Fill up matrices with pixel coordinates.
    auto fillPixelMatrix =
        [](const std::vector<cv::Point2i>& external_contour,
           const std::vector<std::vector<cv::Point2i>>& internal_contours,
           MatrixXr* matrix) {
          uint64_t j = 0;

          for (const cv::Point2i& external_pixel : external_contour) {
            matrix->operator()(0, j) = static_cast<real_t>(external_pixel.x);
            matrix->operator()(1, j) = static_cast<real_t>(external_pixel.y);
            ++j;
          }
          for (const std::vector<cv::Point2i>
                & internal_contour : internal_contours) {
            for (const cv::Point2i& internal_pixel : internal_contour) {
              matrix->operator()(0, j) = static_cast<real_t>(internal_pixel.x);
              matrix->operator()(1, j) = static_cast<real_t>(internal_pixel.y);
              ++j;
            }
          }
        };

    fillPixelMatrix(external_contour_i, internal_contours_i, &contour_i_matrix);
    fillPixelMatrix(external_contour_j, internal_contours_j, &contour_j_matrix);


    // Neighbor search via libnabo library.
    typedef Nabo::NearestNeighbourSearch<real_t> NNSearch;
    // Only search for the nearest neighbor for each pixel of one contour with
    // respect to the other contour.
    const int K = 1;
    // No approximations in neighbor computation.
    const real_t epsilon = 0.0;
    // Nearest neighbor search options.
    const uint64_t
        options = 0;

    // Build a KD-tree for the pixels in blob i.
    auto* kd_tree_pixels_blob_i =
        NNSearch::createKDTreeLinearHeap(contour_i_matrix);

    // For each pixel in blob j there is a corresponding column in the
    // following matrices, containing the index i of the closest pixel in
    // blob i and its associated squared distance.
    Eigen::MatrixXi indices(K, contour_j_matrix.cols());
    MatrixXr distances_squared(K, contour_j_matrix.cols());

    kd_tree_pixels_blob_i->knn(contour_j_matrix, indices, distances_squared, K,
                               epsilon, options);

    for (int j = 0; j < contour_j_matrix.cols(); ++j) {
      // If closest neighbor in blob i for this query pixel belonging to blob j
      // is closer than threshold distance, then these two blobs are neighbors.
      if (distances_squared(0, j) <= distance_squared)
        return true;
    }
    return false;
  } else {

    // Compute closest distance between pixels in blob i and pixels in blob
    // by using a brute force strategy, i.e. by testing each pixel of blob i
    // against each pixel of blob j.

    for (const cv::Point2i& pi : external_contour_i)
      for (const cv::Point2i& pj : external_contour_j) {
        // Compute pixel distance.
        cv::Point2i diff = pi - pj;
        const int dist2 = diff.dot(diff);
        if (dist2 <= distance_squared)
          return true;
      }

    for (const auto& internal_contour_i : internal_contours_i)
      for (const cv::Point2i& pi : internal_contour_i)
        for (const cv::Point2i& pj : external_contour_j) {
          // Compute pixel distance.
          cv::Point2i diff = pi - pj;
          const int dist2 = diff.dot(diff);
          if (dist2 <= distance_squared)
            return true;
        }

    for (const auto& internal_contour_j : internal_contours_j)
      for (const cv::Point2i& pj : internal_contour_j)
        for (const cv::Point2i& pi : external_contour_i) {
          // Compute pixel distance.
          cv::Point2i diff = pi - pj;
          const int dist2 = diff.dot(diff);
          if (dist2 <= distance_squared)
            return true;
        }

    return false;
  }
}

void Blob::computeContours() {
  external_contour_pixels =
      c_blob_.GetExternalContour()->GetContourPoints();
  internal_contour_pixels.clear();
  for (auto& internal_contour : c_blob_.GetInternalContours()) {
    internal_contour_pixels.push_back(internal_contour->GetContourPoints());
  }
}

void Blob::computeBoundingBox() {
  if (external_contour_pixels.size() == 0)
    computeContours();

  bounding_box = cv::boundingRect(external_contour_pixels);
}

void Blob::computeBlobCenter() {
  if (c_blob_.Area(AreaMode::PIXELWISE) >= 5) {
    ellipse = cv::fitEllipse(external_contour_pixels);
  } else {
    float r;
    cv::minEnclosingCircle(external_contour_pixels, ellipse.center, r);
    ellipse.size = cv::Size(r, r);
  }

  // Make sure the center of the fitted ellipse/circle is contained in the
  // blob, as this pixel coordinate will be used to compute the 3D position
  // of the associated semantic entity.
  // pointPolygonTest returns positive (inside), negative (outside), or zero (on
  // an edge) value.
  if(cv::pointPolygonTest(external_contour_pixels, ellipse.center, true) < 0) {
    LOG(WARNING) << "Computed blob center is not inside blob. Shifting center"
        " to closest contour point.";
    int closest_index = -1;
    int min_distance_squared = std::numeric_limits<int>::max();
    const cv::Point2i old_center(ellipse.center.x, ellipse.center.y);
    for(int i = 0; i < external_contour_pixels.size(); ++i) {
      const cv::Point2i& pixel = external_contour_pixels[i];
      const cv::Point2i diff = pixel - old_center;
      const int dist_squared = diff.dot(diff);
      if(dist_squared < min_distance_squared) {
        min_distance_squared = dist_squared;
        closest_index = i;
      }
    }

    CHECK(closest_index != -1);
    // Assign the pixel associated to the closest index to the pixel center.
    pixel_center = ellipse.center = external_contour_pixels[closest_index];
  }

  // Scale down the ellipse by a factor of two.
  ellipse.size.height *= 0.5;
  ellipse.size.width *= 0.5;

  pixel_center = ellipse.center;
}

std::ostream& operator<<(std::ostream& out, const Blob& blob) {
  out << "label: " << blob.semantic_label
      << ", instance: " << blob.instance
      << ", num pixels: " << blob.num_pixels
      << ", center: " << blob.pixel_center
      << ", bounding box: " << blob.bounding_box;
  return out;
}

}

