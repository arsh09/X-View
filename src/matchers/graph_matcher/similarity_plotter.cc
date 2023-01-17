#include <x_view_core/matchers/graph_matcher/similarity_plotter.h>

#include <opencv2/core/eigen.hpp>

namespace x_view {
int SimilarityPlotter::desired_image_size_ = 500;

int SimilarityPlotter::colormap_ = cv::COLORMAP_OCEAN;

cv::Mat SimilarityPlotter::getImageFromSimilarityMatrix(
    const MatrixXr& similarity_matrix, bool auto_size) {

  MatrixXuc uchar_similarity =
      (similarity_matrix * 255.f).cast <uchar>();

  cv::Mat cv_scores, color_scores;
  cv::eigen2cv(uchar_similarity, cv_scores);
  if (auto_size)
    cv::resize(cv_scores, cv_scores,
               SimilarityPlotter::computeSize(cv_scores.size()), 0,
               0, cv::INTER_NEAREST);

  cv::applyColorMap(cv_scores, color_scores, SimilarityPlotter::colormap_);

  return color_scores;

}

cv::Mat SimilarityPlotter::getImageFromSimilarityMatrix(
    const MatrixXb& max_similarity_matrix, bool auto_size) {

  cv::Mat cv_scores(max_similarity_matrix.rows(),
                    max_similarity_matrix.cols(), CV_8UC1);
  for(int i = 0; i < cv_scores.rows; i++)
  {
    uchar* row_ptr = cv_scores.ptr<uchar>(i);
    for(int j = 0; j < cv_scores.cols; j++)
      row_ptr[j] = static_cast<uchar>(max_similarity_matrix(i,j) ? 255 : 0);
  }
  if (auto_size)
    cv::resize(cv_scores, cv_scores,
               SimilarityPlotter::computeSize(cv_scores.size()), 0,
               0, cv::INTER_NEAREST);

  return cv_scores;
}

const cv::Size SimilarityPlotter::computeSize(const cv::Size& original_size) {
  const int original_height = original_size.height;
  const int original_width = original_size.width;

  auto roundToClosestMultiple = [](const int number, const int multiple) {
    return ((number + multiple / 2) / multiple) * multiple;
  };

  const int resulting_height = roundToClosestMultiple
      (SimilarityPlotter::desired_image_size_, original_height);
  const int resulting_width = roundToClosestMultiple
      (SimilarityPlotter::desired_image_size_, original_width);

  if (resulting_height != 0 && resulting_width != 0)
    return cv::Size(resulting_width, resulting_height);
  else
    return original_size;
}

}
