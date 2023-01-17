#include "test_common.h"

#include <x_view_core/x_view_tools.h>
#include <x_view_core/x_view_types.h>

#include <boost/graph/connected_components.hpp>
#include <boost/graph/erdos_renyi_generator.hpp>
#include <boost/graph/random.hpp>
#include <boost/random/linear_congruential.hpp>
#include <x_view_core/parameters/parameters.h>
#include <x_view_core/x_view_locator.h>

namespace x_view_test {

void createParameters() {
  std::unique_ptr<x_view::Parameters> parameters(
      new x_view::Parameters("Test parameters"));
  std::unique_ptr<x_view::Parameters> dataset_parameters(
      new x_view::Parameters("Test Dataset parameters"));
  dataset_parameters->setString("name", "SYNTHIA");

  std::unique_ptr<x_view::Parameters> landmark_parameters(
      new x_view::Parameters("Test Landmark parameters"));

  std::unique_ptr<x_view::Parameters> matcher_parameters(
      new x_view::Parameters("Test Matcher parameters"));

  std::unique_ptr<x_view::Parameters> localizer_parameters(
      new x_view::Parameters("Test Localizer parameters"));
  localizer_parameters->setString("type", "OPTIMIZATION");

  parameters->addChildPropertyList("dataset", std::move(dataset_parameters));
  parameters->addChildPropertyList("landmark", std::move(landmark_parameters));
  parameters->addChildPropertyList("matcher", std::move(matcher_parameters));
  parameters->addChildPropertyList("localizer",
                                   std::move(localizer_parameters));

  x_view::Locator::registerParameters(std::move(parameters));
}

x_view::Graph generateRandomGraph(const GraphConstructionParams& params) {

  typedef boost::erdos_renyi_iterator<boost::minstd_rand,
                                      x_view::Graph> ERGen;

  boost::minstd_rand gen(params.seed);
  std::mt19937 rng(params.seed);
  std::uniform_int_distribution<int> dist(0, params.num_semantic_classes - 1);
  std::uniform_real_distribution<x_view::real_t> step(-1, 1);

  // Create random graph.
  x_view::Graph graph(ERGen(gen, params.num_vertices, params.edge_probability),
                      ERGen(), params.num_vertices);

  // Add properties to the vertices.
  auto vertex_iter = boost::vertices(graph);
  int vertex_index = 0;
  // Iterate over all vertices.
  for (; vertex_iter.first != vertex_iter.second; ++vertex_iter.first) {
    auto& vertex = graph[*vertex_iter.first];
    vertex.index = vertex_index++;
    vertex.num_pixels = 1;
    vertex.center = cv::Point2i(0, 0);
    vertex.location_3d << step(rng), step(rng), 0.0;
    vertex.last_time_seen_ = 0;
    // Set a random semantic label to the vertex.
    vertex.semantic_label = dist(rng);
    vertex.semantic_entity_name = std::to_string(vertex.semantic_label);
  }


  // Add edge properties.
  auto edges_iter = boost::edges(graph);
  for (; edges_iter.first != edges_iter.second; ++edges_iter.first) {
    // Get the vertex descriptors defining the current edge.
    const auto& from_v = graph[boost::source(*edges_iter.first, graph)];
    const auto& to_v = graph[boost::target(*edges_iter.first, graph)];
    // Set the edge properties.
    graph[*edges_iter.first].from = from_v.index;
    graph[*edges_iter.first].to = to_v.index;
    graph[*edges_iter.first].num_times_seen = 1;
  }

  std::vector<int> component(boost::num_vertices(graph));
  int num_connected_components =
      boost::connected_components(graph, &component[0]);

  if (num_connected_components == 1) {
    LOG(INFO) << "Random graph generated with parameters:"
              << "\n\tnum_vertices         : " << params.num_vertices
              << "\n\tedge_probability     : " << params.edge_probability
              << "\n\tnum_semantic_classes : " << params.num_semantic_classes
              << "\n\tseed                 : " << params.seed;
    return graph;
  } else {
    LOG(WARNING) << "Random graph generated with parameters:"
                 << "\n\tnum_vertices         : " << params.num_vertices
                 << "\n\tedge_probability     : " << params.edge_probability
                 << "\n\tnum_semantic_classes : "
                 << params.num_semantic_classes
                 << "\n\tseed                 : " << params.seed
                 << "\nis not a single connected component."
                 << "\nGenerating new graph with modified parameters.";

    // Generate new parameters for graph generation.
    GraphConstructionParams new_params;
    new_params.num_vertices = params.num_vertices;
    new_params.edge_probability =
        std::min(static_cast<x_view::real_t>(1.1),
                 static_cast<x_view::real_t>(params.edge_probability * 1.1));
    new_params.num_semantic_classes = params.num_semantic_classes;
    new_params.seed = params.seed;
    return generateRandomGraph(new_params);
  }
}

x_view::Graph generateChainGraph(const GraphConstructionParams& params) {

  std::mt19937 rng(params.seed);
  std::uniform_int_distribution<int> dist(0, params.num_semantic_classes - 1);

  x_view::Graph graph;
  std::vector<x_view::VertexDescriptor> vertex_descriptors;
  x_view::real_t theta = 0.0;
  const x_view::real_t d_theta = 2 * M_PI / params.num_vertices;
  // Create the vertices of the graph in sequence.
  for (int i = 0; i < params.num_vertices; ++i, theta += d_theta) {
    x_view::VertexProperty vertex;
    vertex.index = i;
    vertex.num_pixels = 1;
    vertex.center = cv::Point2i(0, 0);
    vertex.location_3d << std::cos(theta), std::sin(theta), 0.0;
    vertex.last_time_seen_ = 0;
    // Set a random semantic label to the vertex.
    vertex.semantic_label = dist(rng);
    vertex.semantic_entity_name = std::to_string(vertex.semantic_label);

    vertex_descriptors.push_back(boost::add_vertex(vertex, graph));
  }
  // Add edges between each pair of consequent vertices.
  const uint64_t num_times_seen = 1;
  for (uint64_t i = 0; i < params.num_vertices - 1; ++i) {
    boost::add_edge(vertex_descriptors[i], vertex_descriptors[i + 1],
                    {i, i + 1, num_times_seen}, graph);
  }
  // Close the loop.
  boost::add_edge(vertex_descriptors.back(), vertex_descriptors.front(),
                  {uint64_t(params.num_vertices - 1), 0, num_times_seen},
                  graph);

  return graph;
}

x_view::Graph extractSubgraphAroundVertex(const x_view::Graph& original,
                                          const x_view::VertexDescriptor& source,
                                          const int radius) {

  // An object mapping each VertexDescriptor to the corresponding integer
  // distance to the source VertexDescriptor. This object is filled up by the
  // KNNBSVisitor class.
  KhopVisitor::DistanceMap dist;

  // Perform BFS in order to compute the integer distance from each vertex of
  // the original graph from the source VertexDescriptor.
  KhopVisitor vis(dist);
  boost::breadth_first_search(original, source, boost::visitor(vis));

  // Map which assigns two VertexDescriptors to each other. This is
  // used to keep track of which vertex is assigned to which vertex between
  // the original and the newly generated graph.
  typedef std::map<const x_view::VertexDescriptor,
                   const x_view::VertexDescriptor> VertexToVertexMap;
  VertexToVertexMap old_vertex_to_new;

  // Extracted graph.
  x_view::Graph extracted_graph;

  // Add the selected vertices to the new graph.
  for (const auto& p : dist) {
    const int distance = p.second;
    // Only add the vertex if its distance to the source is smaller or equal
    // to the radius passed as argument.
    if (distance <= radius) {
      const x_view::VertexDescriptor& v_old_d = p.first;
      // Deep copy of the VertexProperty.
      const x_view::VertexProperty v_p = original[v_old_d];
      const x_view::VertexDescriptor& v_new_d =
          boost::add_vertex(v_p, extracted_graph);
      // Keep track of the newly added vertices by mapping the old ones to
      // the newly inserted.
      old_vertex_to_new.insert({v_old_d, v_new_d});
    }
  }

  // Add the edges between the selected vertices if an edge was present in
  // the original graph.
  for (auto it = old_vertex_to_new.begin(); it != old_vertex_to_new.end(); ++it)
    for (auto jt = std::next(it); jt != old_vertex_to_new.end(); ++jt) {
      const x_view::VertexDescriptor v_old_i_d = it->first;
      const x_view::VertexDescriptor v_old_j_d = jt->first;
      const x_view::VertexDescriptor v_new_i_d = it->second;
      const x_view::VertexDescriptor v_new_j_d = jt->second;

      const auto edge = boost::edge(v_old_i_d, v_old_j_d, original);
      if (edge.second) { // If the edge was present in the original graph.
        const x_view::EdgeProperty e_p = original[edge.first];
        boost::add_edge(v_new_i_d, v_new_j_d, e_p, extracted_graph);
      }
    }

  LOG(INFO) << "Created subgraph centered on vertex " << original[source]
            << " with 'radius' of " << radius << ".\nIt has "
            << boost::num_vertices(extracted_graph) << " vertices and "
            << boost::num_edges(extracted_graph) << " edges.";

  return extracted_graph;
}

void modifyGraph(x_view::Graph* graph, const GraphModifierParams& params,
                 std::mt19937& rng) {
  CHECK_NOTNULL(graph);

  LOG_IF(WARNING, params.start_vertex_index == -1)
  << "The modifier function is used with 'params.start_vertex_index_' "
      "= -1, this means that all vertices added to the graph being "
      "modified have 'VertexProperty::index_' = -1. To avoid this "
      "behaviour please set the start vertex index accordingly. "
      "(Usually the start vertex index corresponds to the max vertex "
      "index of the graph being modified, or to the max vertex index "
      "of the base graph associated from which the modified graph is "
      "extracted)";

  std::vector<int> component(boost::num_vertices(*graph));
  int num_connected_components =
      boost::connected_components(*graph, &component[0]);

  CHECK(num_connected_components == 1)
  << "Input graph to " << __FUNCTION__ << " has " << num_connected_components
  << " connected components. Function " << __FUNCTION__
  << " only works with single components.";

  for (int i = 0; i < params.num_vertices_to_add; ++i) {
    int new_vertex_index = (params.start_vertex_index == -1) ?
                           -1 : params.start_vertex_index + i;
    if(params.num_links_for_new_vertices == 0)
      LOG(WARNING) << "You are trying to add a new vertex to a graph without "
          "linking it to any existing vertex. This is going to create two "
          "disconnected components.";
    addRandomVertexToGraph(graph, rng, new_vertex_index,
                           params.num_links_for_new_vertices);
  }
  for (int i = 0; i < params.num_edges_to_add; ++i)
    addRandomEdgeToGraph(graph, rng);

  for (int i = 0; i < params.num_vertices_to_remove; ++i)
    removeRandomVertexFromGraph(graph, rng);

  for (int i = 0; i < params.num_edges_to_remove; ++i) {
    removeRandomEdgeFromGraph(graph, rng);
  }

  component.resize(boost::num_vertices(*graph));
  num_connected_components =
      boost::connected_components(*graph, &component[0]);

  CHECK(num_connected_components == 1)
  << "After removing and adding vertices/edges to the graph, there are "
  << "disconnected  components.";

}

void setLastTimeSeen(x_view::Graph* graph, const uint64_t last_time_seen) {
  const auto vertices = boost::vertices(*graph);
  for (auto iter = vertices.first; iter != vertices.second; ++iter) {
    (*graph)[*iter].last_time_seen_ = last_time_seen;
  }
}

}

