#ifndef X_VIEW_X_VIEW_TOOLS_H
#define X_VIEW_X_VIEW_TOOLS_H

#include <x_view_core/features/graph.h>
#include <x_view_core/x_view_types.h>

#include <opencv2/core/core.hpp>

#include <random>

namespace x_view {


// ************************* Image manipulation ******************************//
/**
 * \brief Interpret two consecutive bytes as an integer.
 * \param b1 First byte.
 * \param b2 Second byte.
 * \return Integer corresponding to the two bytes passed as argument.
 */
int twoBytesToInt(const unsigned char b1, const unsigned char b2);

/**
 * \brief Interprets two consecutive bytes pointed by the passed argument as
 * an integer.
 * \param b Pointer to the bytes.
 * \return Integer corresponding to the two consecutive bytes pointed by the
 * passed argument.
 */
int twoBytesToInt(const unsigned char* b);

/**
 * \brief Extract a single channel from an image.
 * \param image Input image composed of multiple channels.
 * \param channel Desired channel to be extracted.
 * \return New image containing only the channel passed as argument.
 */
cv::Mat extractChannelFromImage(const cv::Mat& image, const int channel);

// ******************************* Utility ***********************************//
class PaddedInt {
 public:
  PaddedInt(const int64_t value, const int pad = 0, const char fill = '0');

  const std::string& str() const;

 private:
  const int64_t value_;
  const int pad_;
  const char fill_;
  std::string str_;
};

std::string operator+(const std::string& l, const PaddedInt& r);
std::string operator+(const PaddedInt& l, const std::string& r);

const std::string formatSE3(const SE3& se3, const std::string& indent = "",
                            const int precision = Eigen::StreamPrecision);

/// \brief Generates different colors for different semantic labels.
const cv::Scalar getColorFromSemanticLabel(const int semantic_label);

/// \brief Generates a random rotation matrix given three uniformly sampled
/// numbers between zero and one.
const Matrix3r createRotationMatrix(real_t r1, real_t r2, real_t r3);
const Matrix3r randomRotationMatrix(std::mt19937& rng);

/// \brief Computes the squared distance between two points in 3D space.
const real_t distSquared(const Vector3r& v1, const Vector3r& v2);

/// \brief Computes the squared distance between two graph vertices in 3D space.
const real_t distSquared(const VertexProperty& v_p1,
                         const VertexProperty& v_p2);

/// \brief Computes the distance between two points in 3D space.
const real_t dist(const Vector3r& v1, const Vector3r& v2);

/// \brief Computes the distance between two graph vertices in 3D space.
const real_t dist(const VertexProperty& v_p1, const VertexProperty& v_p2);

/// \brief Computes the angle between two poses.
/// \note See here: http://www.continuummechanics.org/transformmatrix.html
const real_t angle(const SE3& p1, const SE3& p2);

/// \brief Computes the argsort of any Eigen type matrix.
/// \return Sorted indices in increasing order such that x[indices[i]] <
/// x[indices[i+1]]
template<typename Derived>
Eigen::VectorXi argsort(const Eigen::MatrixBase<Derived>& x) {

  typedef std::pair<int, double> argsort_pair;

  auto argsortComp = [](const argsort_pair& left, const argsort_pair& right) {
    return left.second < right.second;
  };

  Eigen::VectorXi indices(x.size());
  std::vector<argsort_pair> data(x.size());
  for (int i = 0; i < x.size(); i++) {
    data[i].first = i;
    data[i].second = static_cast<double>(x(i));
  }
  std::sort(data.begin(), data.end(), argsortComp);
  for (int i = 0; i < data.size(); i++) {
    indices(i) = data[i].first;
  }
  return indices;
}

// ******************************* Logging ***********************************//
/**
 * \brief Stringification macros used to transform preprocessor strings into
 * c++ strings.
 */
#define X_VIEW_XSTR(s) X_VIEW_STR(s)
#define X_VIEW_STR(s)  #s

/**
 * @brief Returns a string containing the absolute path to the X_View root
 * directory specified in the CMakeLists.txt file as "-DX_VIEW_ROOT_DIR=..."
 */
const std::string& getRootDirectory();

/**
 * @brief Returns a string containing the absolute path to the X_View output
 * directory specified in the CMakeLists.txt file as "-DX_VIEW_OUT_DIR=..."
 */
const std::string& getOutputDirectory();

/**
 * @brief Returns a string containing the absolute path to the X_View log
 * directory specified in the CMakeLists.txt file as "-DX_VIEW_LOG_DIR=..."
 */
const std::string& getLogDirectory();

/**
 * \brief Sets up parameters for logging such that log files are written to
 * the directory specified by "-DX_VIEW_LOG_DIR=..." in the CMakeLists.txt file.
 * \param argv Arguments received in the main() function.
 */
void setupLogging(char** argv);

/**
 * \brief Makes sure all log are written to the corresponding files.
 */
void finalizeLogging();

// **************************** Graph modifiers ******************************//

/**
 * \brief Adds a new generated VertexProperty to the graph pointed by the
 * passed argument. The newly generated vertex is linked towards
 * link_to_n_vertices existing vertices of the graph.
 * \param graph Pointer to the graph to be modified.
 * \param rng Instance of mersenne twister random number generator.
 * \param index Index to associate to the newly added vertex.
 * \param link_to_n_vertices The added vertex is linked to link_to_n_vertices
 * randomly chosen vertices of the graph. This ensure that the new graph
 * consists of a single connected component.
 */
void addRandomVertexToGraph(Graph* graph, std::mt19937& rng,
                            const int index, const int link_to_n_vertices);

/**
 * \brief Adds a new generated EdgeProperty to the graph pointed by the
 * passed argument. The edge is defined by randomly selecting two different
 * vertices of the graph passed as argument, and is added to the graph only
 * if the resulting edge does not already exist.
 * \param graph Pointer to the graph to be modified.
 * \param rng Instance of mersenne twister random number generator.
 */
void addRandomEdgeToGraph(Graph* graph, std::mt19937& rng);

/**
 * \brief Removes a random vertex from the graph pointed by the passed argument.
 * This function makes sure that removing the vertex from the graph
 * does not create two disconnected components.
 * \param graph Pointer to the graph to be modified.
 * \param rng Instance of mersenne twister random number generator
 */
void removeRandomVertexFromGraph(Graph* graph, std::mt19937& rng);

/**
 * \brief Removes a random edge from the graph pointed by the passed argument.
 * This function makes sure that removing the edge from the graph
 * does not create two disconnected components.
 * \param graph Pointer to the graph to be modified.
 * \param rng Instance of mersenne twister random number generator
 */
void removeRandomEdgeFromGraph(Graph* graph, std::mt19937& rng);

class KeyGenerator {
 public:
  static size_t getNextKey();
};

}

#endif //X_VIEW_X_VIEW_TOOLS_H
