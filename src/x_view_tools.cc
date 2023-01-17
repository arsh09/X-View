#include <x_view_core/x_view_tools.h>

#include <x_view_core/x_view_locator.h>
#include <x_view_core/datasets/abstract_dataset.h>

#include <boost/graph/connected_components.hpp>
#include <boost/graph/random.hpp>
#include <glog/logging.h>

#include <atomic>
#include <iomanip>
#include <sstream>

namespace x_view {

// ************************* Image manipulation ******************************//

int twoBytesToInt(const unsigned char b1, const unsigned char b2) {
  return static_cast<int>((b2 << 8) | b1);
};

int twoBytesToInt(const unsigned char* b) {
  return twoBytesToInt(b[0], b[1]);
}

cv::Mat extractChannelFromImage(const cv::Mat& image, const int channel) {
  if (image.channels() == 1)
    return image;

  CHECK_LT(channel, image.channels()) << "Image has less channels than the "
        "requested one. Make sure the input image has multiple channels";

  std::vector<cv::Mat> image_channels(image.channels());
  cv::split(image, image_channels);
  return image_channels[channel];
}

// ******************************* Utility ***********************************//
PaddedInt::PaddedInt(const int64_t value, const int pad, const char fill)
    : value_(value), pad_(pad), fill_(fill) {
  const std::string value_string = std::to_string(value_);
  const int num_digits = value_string.length();
  CHECK(num_digits <= pad)
  << "You are trying to pad a number with " << num_digits
  << " digits with a pad of " << pad;

  std::stringstream ss;
  ss << std::setfill(fill_);
  ss << std::setw(pad) << value_string;
  str_ = ss.str();
}

const std::string& PaddedInt::str() const {
  return str_;
}

std::string operator+(const std::string& l, const PaddedInt& r) {
  return l + r.str();
}
std::string operator+(const PaddedInt& l, const std::string& r) {
  return l.str() + l;
}

const std::string formatSE3(const SE3& se3, const std::string& indent,
                            const int precision) {
  const Eigen::IOFormat format(precision, 0, ", ",
                               std::string("\n") + indent, "[", "]");
  std::string s = "";
  s += std::string("origin:\n") + indent;

  std::stringstream ss;
  ss << std::setfill(' ')
      << RowVector3r(se3.getPosition().cast<real_t>()).format(format);
  s += ss.str();
  ss.str(std::string());

  s += std::string("\nrotation:\n") + indent;
  ss << se3.getRotationMatrix().format(format);
  s += ss.str();
  return s;
}

const cv::Scalar getColorFromSemanticLabel(const int semantic_label) {
  const unsigned char intensity =
      static_cast<unsigned char>(255. / (semantic_label / 8 + 1));

  cv::Scalar color(0, 0, 0);
  std::bitset<3> bits(semantic_label);

  color[0] = bits[0] ? intensity : static_cast<uchar>(0);
  color[1] = bits[1] ? intensity : static_cast<uchar>(0);
  color[2] = bits[2] ? intensity : static_cast<uchar>(0);

  return color;
}

const Matrix3r createRotationMatrix(real_t r1, real_t r2, real_t r3) {
  r1 = r1 * 2.0 * M_PI;
  r2 = r2 * 2.0 * M_PI;
  r3 = r3 * 2.0;

  real_t r = std::sqrt(r3);
  real_t vx = std::sin(r2) * r;
  real_t vy = std::cos(r2) * r;
  real_t vz = std::sqrt(2.0 - r3);

  real_t st = std::sin(r1);
  real_t ct = std::cos(r1);

  Matrix3r R;
  R << ct, st, 0, -st, ct, 0, 0, 0, 1;

  Matrix3r M =  (Vector3r(vx, vy, vz) * RowVector3r(vx, vy, vz) - Matrix3r::Identity()) * R;
  return M;
}

const Matrix3r randomRotationMatrix(std::mt19937& rng) {
  std::uniform_real_distribution<real_t> dist(0, 1);
  return createRotationMatrix(dist(rng), dist(rng), dist(rng));
}

const real_t distSquared(const Vector3r& v1, const Vector3r& v2) {
  return (v1 - v2).squaredNorm();
}

const real_t distSquared(const VertexProperty& v_p1,
                         const VertexProperty& v_p2) {
  return distSquared(v_p1.location_3d, v_p2.location_3d);
}

const real_t dist(const Vector3r& v1, const Vector3r& v2) {
  return (v1 - v2).norm();
}

const real_t dist(const VertexProperty& v_p1, const VertexProperty& v_p2) {
  return dist(v_p1.location_3d, v_p2.location_3d);
}

const real_t angle(const SE3& p1, const SE3& p2) {
  const Eigen::Matrix3d relative_rot = (p1.inverse() * p2).getRotationMatrix();
  return std::acos((relative_rot.trace() - 1) * 0.5);
}

// ******************************* Logging ***********************************//

const std::string& getRootDirectory() {
  static std::string x_view_root = std::string(X_VIEW_XSTR(X_VIEW_ROOT_DIR));
  return x_view_root;
}

const std::string& getOutputDirectory() {
  static std::string x_view_out = std::string(X_VIEW_XSTR(X_VIEW_OUT_DIR));
  return x_view_out;
}

const std::string& getLogDirectory() {
  static std::string x_view_log = std::string(X_VIEW_XSTR(X_VIEW_LOG_DIR));
  return x_view_log;
}

void setupLogging(char** argv) {

  const std::string& log_dir_name = x_view::getLogDirectory();

  std::vector<std::pair<const int, std::string> > log_file_names =
      {{google::INFO, "log_INFO"},
       {google::WARNING, "log_WARN"},
       {google::ERROR, "log_ERR"},
       {google::FATAL, "log_FATAL"}};

  for (const auto& level : log_file_names) {
    google::SetLogDestination(level.first,
                              (log_dir_name + level.second).c_str());

    google::SetLogSymlink(level.first, "__LAST");
  }

  // Print logs also to the console if their level is greater than
  // min_console_level;
  const int min_console_level = google::ERROR;
  FLAGS_colorlogtostderr = true;
  google::SetStderrLogging(min_console_level);

#ifdef X_VIEW_DEBUG
  FLAGS_alsologtostderr = true;
#endif

  google::InitGoogleLogging(argv[0]);

  std::cout << "X-View is logging to <" << log_dir_name << ">" << std::endl;
}

void finalizeLogging() {
  google::FlushLogFiles(google::INFO);
  google::FlushLogFiles(google::WARNING);
  google::FlushLogFiles(google::ERROR);
  google::FlushLogFiles(google::FATAL);
}

// **************************** Graph modifiers ******************************//

void addRandomVertexToGraph(Graph* graph, std::mt19937& rng,
                            const int index, const int link_to_n_vertices) {
  CHECK_NOTNULL(graph);

  const auto& dataset = Locator::getDataset();

  // Create the new vertex.
  VertexProperty new_vertex;
  if (index == -1)
    new_vertex.index = static_cast<int>(boost::num_vertices(*graph));
  else
    new_vertex.index = index;
  // Random semantic label.
  new_vertex.semantic_label =
      static_cast<int>(rng() % dataset->numSemanticClasses());
  new_vertex.semantic_entity_name =
      "Random vertex " + std::to_string(new_vertex.index);

  // Define random vertices to be linked with the new vertex.
  std::vector<VertexDescriptor> vertices_to_link;
  while (vertices_to_link.size() < link_to_n_vertices) {
    const VertexDescriptor link_v_d = boost::random_vertex(*graph, rng);
    if (std::find(vertices_to_link.begin(), vertices_to_link.end(), link_v_d)
        == vertices_to_link.end())
      vertices_to_link.push_back(link_v_d);
  }

  const VertexDescriptor& new_vertex_d = boost::add_vertex(new_vertex, *graph);
  LOG(INFO) << "Added vertex " << (*graph)[new_vertex_d];
  for (const VertexDescriptor& v_d : vertices_to_link) {
    const VertexProperty& v_p = (*graph)[v_d];
    LOG(INFO) << "--> linked to existing vertex " << v_p.index << std::endl;
    const uint64_t num_times_seen = 1;
    boost::add_edge(v_d, new_vertex_d,
                    {v_p.index, new_vertex.index, num_times_seen}, *graph);
  }
}

void addRandomEdgeToGraph(Graph* graph, std::mt19937& rng) {
  CHECK_NOTNULL(graph);

  bool edge_added = false;
  while (!edge_added) {
    // Select two random vertices from the graph.
    const VertexDescriptor& v1_d = boost::random_vertex(*graph, rng);
    const VertexDescriptor& v2_d = boost::random_vertex(*graph, rng);

    if (addEdgeBetweenVertices(v1_d, v2_d, graph)) {
      const VertexProperty& v1_p = (*graph)[v1_d];
      const VertexProperty& v2_p = (*graph)[v2_d];

      LOG(INFO) << "Added edge between vertices " << v1_p.index
                << ", " << v2_p.index << std::endl;
      edge_added = true;
    }
  }
}

void removeRandomVertexFromGraph(Graph* graph, std::mt19937& rng) {
  CHECK_NOTNULL(graph);

  if (boost::num_vertices(*graph) <= 1) {
    LOG(WARNING) << "Cannot remove a vertex from a graph with "
                 << boost::num_vertices(*graph) << " vertices.";
    return;
  }

  bool single_connected_component = false;
  while (!single_connected_component) {

    // Try to remove the vertex on a test graph.
    Graph test_graph = *graph;

    // Select a random vertex from the graph.
    const VertexDescriptor v_d = boost::random_vertex(test_graph, rng);
    // Create a copy of the vertex property to be deleted as the vertex
    // descriptor will be invalidated after the remove_vertex call.
    const VertexProperty v_p = test_graph[v_d];

    boost::clear_vertex(v_d, test_graph);
    boost::remove_vertex(v_d, test_graph);

    // Compute the connected components of the new graph.
    std::vector<int> component(boost::num_vertices(test_graph));
    int num_connected_components =
        boost::connected_components(test_graph, &component[0]);

    if (num_connected_components == 1) {
      LOG(INFO) << "Removed vertex " << v_p << ".";
      single_connected_component = true;
      *graph = test_graph;
    } else {
      LOG(WARNING)
          << "Could not remove vertex " << v_d
          << " as it would create two disconnected components. "
          << "Choosing a new vertex.";
    }
  }
}

void removeRandomEdgeFromGraph(Graph* graph, std::mt19937& rng) {
  CHECK_NOTNULL(graph);

  if (boost::num_edges(*graph) <= 1) {
    LOG(WARNING) << "Cannot remove an edge from a graph with "
                 << boost::num_edges(*graph) << " edges.";
    return;
  }

  bool single_connected_component = false;
  while (!single_connected_component) {

    // Try to remove the vertex on a test graph.
    Graph test_graph = *graph;

    // Select a random edge to be removed.
    const EdgeDescriptor& e_d = boost::random_edge(test_graph, rng);

    // Compute the connected components of the new graph.
    std::vector<int> component(boost::num_vertices(test_graph));
    int num_connected_components =
        boost::connected_components(test_graph, &component[0]);

    if (num_connected_components == 1) {
      const EdgeProperty& e_p = test_graph[e_d];
      LOG(INFO) << "Removed edge " << e_p << ".";
      single_connected_component = true;
      *graph = test_graph;
    } else {
      const EdgeProperty& e_p = test_graph[e_d];
      LOG(WARNING)
          << "Could not remove edge " << e_p
          << " as it would create two disconnected components. "
          << "Choosing a new edge.";

    }
  }
}

size_t KeyGenerator::getNextKey() {
  static std::atomic<size_t> key(0u);
  size_t new_key = key++;
  return new_key;
}

};
