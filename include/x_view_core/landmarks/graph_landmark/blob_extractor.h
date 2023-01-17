#ifndef X_VIEW_BLOB_EXTRACTOR_H
#define X_VIEW_BLOB_EXTRACTOR_H

#include <x_view_core/landmarks/graph_landmark/blob.h>
#include <x_view_core/landmarks/graph_landmark/graph_landmark_types.h>
#include <x_view_core/x_view_types.h>

#include <opencv2/core/core.hpp>
#include <opencvblobslib/blob.h>

#include <vector>
#include <unordered_set>

#ifdef X_VIEW_DEBUG
#define X_VIEW_NUM_THREADS_BLOB_EXTRACTION 1
#else
#define X_VIEW_NUM_THREADS_BLOB_EXTRACTION 4
#endif

namespace x_view {

/**
 * \brief Parameters used by the blob extractor to extract blobs from the
 * passed image.
 */
struct BlobExtractorParams {

  /**
   * \brief The minimum blob size can be either given as an absolute integer
   * value representing the number of pixels, or by a value relative to the
   * number of pixels in the image.
   */
  enum class MIN_BLOB_SIZE_TYPE {
    RELATIVE,
    ABSOLUTE
  };

  /**
   * \brief This struct computes the minimum blob size based on the
   * parameters set in the BlobExtractorParams.
   */
  struct BlobSizeFiltering {

    /**
     * \brief Default constructor will filter out all blobs with a number of
     * pixels smaller than 1% of of the total number of pixels in the image.
     */
    BlobSizeFiltering()
        : type(MIN_BLOB_SIZE_TYPE::RELATIVE),
          fraction_min_pixels(0.01) {}

    MIN_BLOB_SIZE_TYPE type;
    /// \brief If type is RELATIVE, then the fraction_min_pixels part of the
    /// union is active, otherwise if type is ABSOLUTE, the num_min_pixels is
    /// active. The first represents the fraction of pixels relative to the
    /// total number of pixels in the image, the latter represents the minimum
    /// number of pixels a blob must have in order to be extracted.
    union {
      real_t fraction_min_pixels;
      int num_min_pixels;
    };

    /**
     * \brief Computes the minimum number of pixels a blob must have in order
     * to be extracted from the image.
     * \param image_size Size of the image being analyzed for blob extraction.
     * \return Minimum number of pixels a blob must have in order to be
     * extracted from the image.
     */
    int minimumBlobSize(const cv::Size& image_size) const;
  };

  /**
   * \brief Default constructor which does not perform any dilation and
   * erosion operation on the image, works with X_VIEW_NUM_THREADS_BLOB_EXTRACTION
   * threads and uses a default blob filtering struct.
   */
  BlobExtractorParams()
      : dilate_and_erode(false),
        num_dilate_reps(1),
        num_erode_reps(1),
        blob_size_filtering(BlobSizeFiltering()),
        num_threads(X_VIEW_NUM_THREADS_BLOB_EXTRACTION) {
  }

  /// \brief Boolean indicating if a dilation and erosion procedure must be
  /// applied to the input image before blob extraction.
  bool dilate_and_erode;
  /// \brief Number of dilation repetitions to be applied to the image.
  int num_dilate_reps;
  /// \brief Number of erosion repetitions to be applied to the image.
  int num_erode_reps;
  /// \brief Filtering parameter determining which blobs are extracted and
  /// which not.
  BlobSizeFiltering blob_size_filtering;
  /// \brief Number of threads to be used for blob extraction.
  int num_threads;

  /**
   * \brief Computes the minimum number of pixels a blob must have in order
   * to be extracted from the image.
   * \param image_size Size of the image being analyzed for blob extraction.
   * \return Minimum number of pixels a blob must have in order to be
   * extracted from the image.
   */
  int minimumBlobSize(const cv::Size& image_size) const {
    return blob_size_filtering.minimumBlobSize(image_size);
  }
};

/**
* \brief This class provides methods to extract blobs from an image.
* \details Given an image with three channels, this class provides methods to
 * extract blobs considering the semantic class associated to each pixel
 * (first channel), and in case instance information is also provided (second
 * channel) the extracted blobs distinguish between object of the same
 * semantic class having different instance id.
*/
class BlobExtractor {

 public:

  /**
   * \brief Computes the blobs from the image passed as argument and
   * generates a ImageBlobs datastructure containing all the detected blobs.
   * \param image Image to be analyzed for blob detection.
   * \param params Parameters to be used for blob extraction.
   * \return ImageBlobs datastructure containing all detected blobs.
   */
  static ImageBlobs extractBlobs(const cv::Mat& image,
                                 const BlobExtractorParams& params =
                                 BlobExtractorParams());

 private:

  /**
   * \brief Extracts the blobs from a binary single channel mask/image. This
   * function if used to extract blobs for semantic classes which have no
   * instance dinstiction.
   * \param current_class_layer Single channel binary image where a pixel is
   * one only if its label is identical to current_semantic_class.
   * \param class_blobs Pointer to ClassBlobs to be filled up with the
   * detected blobs. The pointer usually points to
   * ImageBlobs[current_semantic_class].
   * \param current_semantic_class Semantic class associated to the extracted
   * blobs.
   * \param params Const reference to parameters used during blobs extraction.
   */
  static void extractBlobsWithoutInstances(cv::Mat& current_class_layer,
                                           ClassBlobs* class_blobs,
                                           const int current_semantic_class,
                                           const BlobExtractorParams& params);

  /**
   * \brief Extracts the blobs from a binary single channel mask/image.
   * \param instance_layer Single channel binary image where a pixel is
   * one only if its label is identical to current_semantic_class. All pixels
   * marked as one in the binary image belong to the same instance of the
   * semantic class.
   * \param class_blobs Pointer to ClassBlobs to be filled up with the
   * detected blobs. The pointer usually points to
   * ImageBlobs[current_semantic_class].
   * \param current_semantic_class Semantic class associated to the extracted
   * blobs.
   * \param params Const reference to parameters used during blobs extraction.
   * \details This function is used to extract blobs for semantic classes which
   * have differentiate instances with an unique instance id, therefore after
   * determining how many instances of class current_semantic_class exist in
   * the image, a loop is performed over the instances and blobs are
   * extracted for each of them.
   */
  static void extractBlobsWithInstances(cv::Mat& instance_layer,
                                        ClassBlobs* class_blobs,
                                        const int current_semantic_class,
                                        const BlobExtractorParams& params);

  /**
   * \brief Given a binary single channel image, this function extracts the
   * blobs associated to the the current_semantic_class and to the specific
   * instance instance_value.
   * \param image Binary single channel image where a pixel is one only if
   * its corresponding semantic label corresponds to current_semantic_class
   * and its instance id is identical to instance_value.
   * \param class_blobs Pointer to ClassBlobs object filled up with the
   * extracted blobs.
   * \param instance_value Instance value of the blobs to be extracted.
   * \param current_semantic_class Semantic class index of blobs to be
   * extracted.
   * \param params Const reference to parameters used during blobs extraction.
   */
  static void extractBlobsAndAddToContainer(cv::Mat& image,
                                            ClassBlobs* class_blobs,
                                            const int instance_value,
                                            const int current_semantic_class,
                                            const BlobExtractorParams& params);

  /**
   * \brief Dilates and erodes the binary single channel image pointed by the
   * passed parameter.
   * \param image Pointer to single channel binary image to be dilated and
   * eroded.
   * \param params Const reference to parameters used for dilation and erosion.
   */
  static void dilateAndErode(cv::Mat* image, const BlobExtractorParams& params);

  /**
   * \brief Given a single channel image containing instance information for
   * each pixel, this function collects the individual instances and stores
   * them into the instance_set parameter passed as argument.
   * \param image Single channel image containing information about the
   * instance associated to each pixel.
   * \param instance_set Pointer to an unordered set. After calling the
   * function the unordered set pointed by the parameter contains all
   * different instances contained in the image passed as parameter.
   */
  static void collectInstancesFromImage(
      const cv::Mat& image, std::unordered_set<unsigned char>* instance_set);

};

}

#undef X_VIEW_NUM_THREADS_BLOB_EXTRACTION

#endif //X_VIEW_BLOB_EXTRACTOR_H
