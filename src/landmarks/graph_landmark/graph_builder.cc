#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/landmarks/graph_landmark/blob.h>
#include <x_view_core/landmarks/graph_landmark/depth_projector.h>
#include <x_view_core/landmarks/graph_landmark/graph_builder.h>
#include <x_view_core/x_view_tools.h>

#include <boost/graph/connected_components.hpp>

namespace x_view {

std::vector<const Blob*> GraphBuilder::DEFAULT_BLOB_VECTOR;

const uint64_t GraphBuilder::INVALID_VERTEX_DESCRIPTOR =
    std::numeric_limits<uint64_t>::max();

Graph GraphBuilder::extractSemanticGraph(const FrameData& frame_data,
                                         const ImageBlobs& blobs,
                                         const GraphBuilderParams& params) {

  if (params.extraction_type ==
      GraphBuilderParams::EXTRACTION_TYPE::EDGES_DEFINED_ON_BLOB_NEIGHBORS) {
    LOG(INFO) << "Building semantic graph by defining edges between neighbor "
        "blobs in semantic segmentation.";
    return extractSemanticGraphOnSemanticImage(frame_data, blobs, params);
  } else if (params.extraction_type ==
      GraphBuilderParams::EXTRACTION_TYPE::EDGES_DEFINED_ON_3D_SPACE) {
    LOG(INFO) << "Building semantic graph by defining edges between neighbor "
        "blobs in 3D space.";
    return extractSemanticGraphOn3DSpace(frame_data, blobs, params);
  } else {
    LOG(ERROR) << "Unrecognized graph extraction type.";
  }
}

Graph GraphBuilder::extractSemanticGraphOnSemanticImage(
    const FrameData& frame_data, const ImageBlobs& blobs,
    const GraphBuilderParams& params) {

  Graph graph;

  // Vector containing references to the created graph vertices.
  std::vector<VertexDescriptor> vertex_descriptors;
  // Vector keeping track of which Blob is associated to which graph node.
  std::vector<const Blob*> blob_vector;

  // Add all blobs to the graph creating semantic vertices.
  GraphBuilder::addBlobsToGraph(frame_data, blobs, &graph, &vertex_descriptors,
                                &blob_vector);

  // Create the edges between nodes sharing an edge.
  for (uint64_t i = 0; i < blob_vector.size(); ++i) {
    for (uint64_t j = i + 1; j < blob_vector.size(); ++j) {

      const Blob* bi = blob_vector[i];
      const Blob* bj = blob_vector[j];

      // Since we don't know yet how many times this edge has been seen in
      // the past, we set its property as if it has been seen only once. This
      // property is updated in the global semantic graph during graph merging.
      const uint64_t num_times_seen = 1;

      // Only create an edge between the two blobs if they are neighbors.
      if (Blob::areNeighbors(*bi, *bj, params.max_distance_for_neighborhood))
        if (vertex_descriptors[i] != INVALID_VERTEX_DESCRIPTOR &&
            vertex_descriptors[j] != INVALID_VERTEX_DESCRIPTOR)
          boost::add_edge(vertex_descriptors[i], vertex_descriptors[j],
                          {i, j, num_times_seen}, graph);
    }
  }

  // Check that the generated graph is a single connected component.
  std::vector<int> component(boost::num_vertices(graph));
  int num_components = boost::connected_components(graph, &component[0]);

  if (num_components == 1)
    return graph;
  while(num_components > 1) {
    LOG(WARNING) << "Graph built upon semantic image presents "
                 << num_components << " disconnected components.";
    connectComponentsInImage(component, &graph);
    num_components = boost::connected_components(graph, &component[0]);
  }
  CHECK_EQ(num_components, 1)
    << "The graph resulting from connectComponentsInImage "
    << "has " << num_components << " components.";
  return graph;

}

Graph GraphBuilder::extractSemanticGraphOn3DSpace(
    const FrameData& frame_data, const ImageBlobs& blobs,
    const GraphBuilderParams& params) {

  Graph graph;

  // Vector containing references to the created graph vertices.
  std::vector<VertexDescriptor> vertex_descriptors;

  // Add all blobs to the graph creating semantic vertices.
  GraphBuilder::addBlobsToGraph(frame_data, blobs, &graph, &vertex_descriptors);

  // Create the edges between nodes sharing an edge.
  const uint64_t num_vertices = boost::num_vertices(graph);
  for (uint64_t i = 0; i < num_vertices; ++i) {
    for (uint64_t j = i + 1; j < num_vertices; ++j) {

      const VertexProperty& vi = graph[i];
      const VertexProperty& vj = graph[j];

      // Since we don't know yet how many times this edge has been seen in
      // the past, we set its property as if it has been seen only once. This
      // property is updated in the global semantic graph during graph merging.
      const uint64_t num_times_seen = 1;

      if (dist(vi, vj) <= params.max_euclidean_distance)
        boost::add_edge(i, j, {i, j, num_times_seen}, graph);
    }
  }

  // Check that the generated graph is a single connected component.
  std::vector<int> component(boost::num_vertices(graph));
  int num_components = boost::connected_components(graph, &component[0]);
  if (num_components == 1)
    return graph;
  while(num_components > 1) {
    LOG(WARNING) << "Graph built upon semantic image presents "
                 << num_components << " disconnected components over "
                 << boost::num_vertices(graph) << " vertices.";
    connectComponentsInSpace(component, &graph);
    num_components = boost::connected_components(graph, &component[0]);
  }

  CHECK_EQ(num_components, 1)
    << "The graph resulting from connectComponentsInSpace "
    << "has " << num_components << " components.";
  return graph;
}

void GraphBuilder::addBlobsToGraph(const FrameData& frame_data,
                                   const ImageBlobs& blobs,
                                   Graph* graph,
                                   std::vector<VertexDescriptor>* vertex_descriptors,
                                   std::vector<const Blob*>* blob_vector) {

  CHECK_NOTNULL(graph);
  CHECK_NOTNULL(vertex_descriptors);
  CHECK_NOTNULL(blob_vector);

  const cv::Mat& depth_image = frame_data.getDepthImage();
  const SE3& pose = frame_data.getPose();

  const real_t max_depth_m =
      Locator::getParameters()->getChildPropertyList("landmark")->getFloat(
          "depth_clip", 10000.f
      );

  // Create a projector object, which projects pixels back to 3D coordinates
  // expressed in world frame.
  DepthProjector projector(pose, Locator::getDataset()->getCameraIntrinsics());

  vertex_descriptors->clear();
  blob_vector->clear();

  // Each blob is a graph node
  uint64_t blob_count = 0;
  for (int c = 0; c < blobs.size(); ++c) {
    for (const Blob& blob : blobs[c]) {
      blob_vector->push_back(&blob);
      size_t key = KeyGenerator::getNextKey();
      VertexProperty vertex =
          GraphBuilder::blobToGraphVertex(key, blob);

      // Extract the depth associated to the vertex.
      const real_t depth_m = Locator::getDataset()->getDepth(vertex.center,
                                                             depth_image);

      // If the projected center of the blob is too distant, set it as invalid.
      if (depth_m >= max_depth_m) {
        vertex_descriptors->push_back(INVALID_VERTEX_DESCRIPTOR);
        continue;
      }
      // Compute the 3D location of the current vertex by projecting the
      // pixel associated to its center into the world.
      vertex.location_3d =
          projector.getWorldCoordinates(vertex.center, depth_m);

      // Set the last_time_seen_ property of the newly created vertex to be
      // the current frame.
      vertex.last_time_seen_ = frame_data.getID();

      // Store the location of the observer.
      vertex.observers.push_back(frame_data.getPoseId());

      // Add the newly generated vertex to the graph.
      vertex_descriptors->push_back(boost::add_vertex(vertex, *graph));
    }
  }
}

VertexProperty GraphBuilder::blobToGraphVertex(const uint64_t index,
                                               const Blob& blob) {
  const auto& dataset = Locator::getDataset();

  const int semantic_label = blob.semantic_label;
  const std::string label = dataset->label(semantic_label);
  const uint64_t size = blob.num_pixels;
  const cv::Point2i center = blob.pixel_center;
  return VertexProperty{index, semantic_label, label, size, center};
}

void GraphBuilder::connectComponentsInImage(const std::vector<int>& component,
                                           Graph* graph) {

  std::vector<int> unique_components(component);
  std::sort(unique_components.begin(), unique_components.end());
  unique_components.erase(std::unique(unique_components.begin(),
                                      unique_components.end()),
                          unique_components.end());

  const uint64_t num_vertices = boost::num_vertices(*graph);

  auto distSquared = [](const cv::Point2i& p1, const cv::Point2i& p2) {
    cv::Point2i d(p1 - p2);
    return d.x * d.x + d.y * d.y;
  };

  // Iterate over each pair of components and determine the closest pair of
  // vertices to be connected.
  int min_distance_square = std::numeric_limits<int>::max();
  EdgeProperty closest_v_d_pair = {0, 0};
  for (auto first_component = unique_components.begin();
       first_component != unique_components.end(); ++first_component) {
    const int first_component_id = *first_component;
    for (auto second_component = std::next(first_component);
         second_component != unique_components.end(); ++second_component) {
      const int second_component_id = *second_component;
      for (uint64_t i = 0; i < num_vertices; ++i) {
        if (component[i] == first_component_id) {
          const cv::Point2i& center_i = (*graph)[i].center;
          for (uint64_t j = 0; j < num_vertices; ++j) {
            if (component[j] == second_component_id) {
              const cv::Point2i& center_j = (*graph)[j].center;
              int dist2 = distSquared(center_i, center_j);
              if (dist2 < min_distance_square) {
                min_distance_square = dist2;
                closest_v_d_pair.from = i;
                closest_v_d_pair.to = j;
                closest_v_d_pair.num_times_seen = 1;
              }
            }
          }
        }
      }
    }
  }
  CHECK(closest_v_d_pair.from != 0 || closest_v_d_pair.to != 0)
  << "Function " << __FUNCTION__ << " could not determine which "
  << "vertices are the closest pair in the disconnected graph "
  << "between component.";
  // The closest vertices between first_component and second_component
  // are the ones contained in closest_v_d_pair.
  boost::add_edge(closest_v_d_pair.from, closest_v_d_pair.to,
                  closest_v_d_pair, *graph);
}


void GraphBuilder::connectComponentsInSpace(const std::vector<int>& component,
                                           Graph* graph) {

  std::vector<int> unique_components(component);
  std::sort(unique_components.begin(), unique_components.end());
  unique_components.erase(std::unique(unique_components.begin(),
                                      unique_components.end()),
                          unique_components.end());

  const uint64_t num_vertices = boost::num_vertices(*graph);

  // Iterate over each pair of components and determine the closest pair of
  // vertices to be connected.
  real_t min_distance_square = std::numeric_limits<real_t>::max();
  EdgeProperty closest_v_d_pair = {0, 0};
  for (auto first_component = unique_components.begin();
       first_component != unique_components.end(); ++first_component) {
    const int first_component_id = *first_component;
    for (auto second_component = std::next(first_component);
         second_component != unique_components.end(); ++second_component) {
      const int second_component_id = *second_component;
      for (uint64_t i = 0; i < num_vertices; ++i) {
        if (component[i] == first_component_id) {
          const VertexProperty& v_p_i = (*graph)[i];
          for (uint64_t j = 0; j < num_vertices; ++j) {
            if (component[j] == second_component_id) {
              const VertexProperty& v_p_j = (*graph)[j];
              real_t dist2 = distSquared(v_p_i, v_p_j);
              if (dist2 < min_distance_square) {
                min_distance_square = dist2;
                closest_v_d_pair.from = i;
                closest_v_d_pair.to = j;
                closest_v_d_pair.num_times_seen = 1;
              }
            }
          }
        }
      }
    }
  }
  CHECK(closest_v_d_pair.from != 0 || closest_v_d_pair.to != 0)
  << "Function " << __FUNCTION__ << " could not determine which "
  << "vertices are the closest pair in the disconnected graph "
  << "between component.";
  // The closest vertices between first_component and second_component
  // are the ones contained in closest_v_d_pair.
  boost::add_edge(closest_v_d_pair.from, closest_v_d_pair.to,
                  closest_v_d_pair, *graph);
}

}
