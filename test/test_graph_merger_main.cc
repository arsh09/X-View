#include <gtest/gtest.h>

#include "test_graph_merger.h"
#include "test_common.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/x_view_locator.h>
#include <x_view_core/x_view_tools.h>

#include <boost/graph/random.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <random>

using namespace x_view;
using namespace x_view_test;

TEST(XViewSlamTestSuite, test_graph_merger) {

  LOG(INFO) << "\n\n======Testing graph merger======";

  const int num_semantic_classes = 13;
  LOG(INFO) << "Testing graph landmark merger with " << num_semantic_classes
            << "classes.";

  std::unique_ptr<AbstractDataset> dataset_(
      new AbstractDataset(num_semantic_classes));
  Locator::registerDataset(std::move(dataset_));

  const auto& dataset = Locator::getDataset();

  const uint64_t seed = 0;

  // Generate a base graph.
  GraphConstructionParams graph_construction_params;
  graph_construction_params.num_semantic_classes = dataset->numSemanticClasses();
  graph_construction_params.seed = seed;
  graph_construction_params.num_vertices = 15;
  graph_construction_params.edge_probability = 0.2;

  const Graph database_graph = generateRandomGraph(graph_construction_params);

  // Extract a subgraph.
  std::mt19937 rng(seed);
  const int radius = 2;
  const VertexDescriptor random_v_d = boost::random_vertex(database_graph, rng);
  const Graph query_graph_orig =
      extractSubgraphAroundVertex(database_graph, random_v_d, radius);

  // Modify the extracted subgraph.
  GraphModifierParams graph_modifier_params;
  graph_modifier_params.num_vertices_to_add = 1;
  graph_modifier_params.num_links_for_new_vertices = 1;
  graph_modifier_params.num_vertices_to_remove = 1;
  graph_modifier_params.num_edges_to_add = 1;
  graph_modifier_params.num_edges_to_remove = 1;

  // Set the index at which the newly inserted vertices start.
  uint64_t max_index = 0;
  const auto vertices = boost::vertices(database_graph);
  for(auto iter = vertices.first; iter != vertices.second; ++iter) {
    max_index = std::max(max_index, database_graph[*iter].index);
  }
  graph_modifier_params.start_vertex_index = int(max_index + 1);

  Graph query_graph = query_graph_orig;
  modifyGraph(&query_graph, graph_modifier_params, rng);
  const uint64_t new_last_time_seen = 1;
  setLastTimeSeen(&query_graph, new_last_time_seen);

  // Perform the matching.
  Graph merged_graph;
  mergeGraphs(database_graph, query_graph, &merged_graph);

  const std::string output_path = getOutputDirectory();
  const std::string g1_path = output_path + "database_graph.dot";
  const std::string g2_path = output_path + "query_graph.dot";
  const std::string merged_path = output_path + "merged.dot";

  writeToFile(database_graph, g1_path);
  writeToFile(query_graph, g2_path);
  writeToFile(merged_graph, merged_path);

  // Close all windows.
  cv::destroyAllWindows();
}

