#include <gtest/gtest.h>

#include "test_random_walk.h"
#include "test_common.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/x_view_locator.h>

#include <chrono>

using namespace x_view;
using namespace x_view_test;

TEST(XViewSlamTestSuite, test_random_walk) {

  LOG(INFO) << "\n\n====Testing random walks generation====";

  const int num_semantic_classes = 13;
  LOG(INFO) << "Testing random walks with " << num_semantic_classes
            << "classes.";

  std::unique_ptr<AbstractDataset> dataset(
      new AbstractDataset(num_semantic_classes));
  Locator::registerDataset(std::move(dataset));

  const uint64_t seed = 0;
  const int walk_length = 3;
  const int num_walks_per_vertex = 20;

  // Create multiple random graphs with different topology and test them.
  std::vector<std::pair<int, real_t>> graph_statistics{
      {10, 0.5},  // Graph statistic has form (num_vertices, edge_probability).
      {10, 1.0},
      {50, 0.2},
      {50, 0.5}
#ifndef X_VIEW_DEBUG
      ,{300, 0.05}
#endif
  };
  std::vector<RandomWalkerParams::SAMPLING_TYPE> sampling_types{
      RandomWalkerParams::SAMPLING_TYPE::UNIFORM,
      RandomWalkerParams::SAMPLING_TYPE::AVOIDING,
      RandomWalkerParams::SAMPLING_TYPE::WEIGHTED,
      RandomWalkerParams::SAMPLING_TYPE::NON_RETURNING
  };

  int num_vertices;
  real_t edge_probability;
  for (auto graph_statistic : graph_statistics) {
    std::tie(num_vertices, edge_probability) = graph_statistic;

    GraphConstructionParams construction_params;
    construction_params.num_vertices = num_vertices;
    construction_params.edge_probability = edge_probability;
    construction_params.num_semantic_classes = num_semantic_classes;
    construction_params.seed = seed;

    const Graph graph = generateRandomGraph(construction_params);

    for (const auto sampling_type : sampling_types) {

      RandomWalkerParams params;
      params.random_sampling_type = sampling_type;
      params.walk_length = walk_length;
      params.num_walks = num_walks_per_vertex;

      LOG(INFO) << "Testing random graph with:\n\t"
                << "Num vertices: " << num_vertices << "\n\t"
                << "Num edges: " << boost::num_edges(graph) << "\n"
                << "Parameters: \n" << params << std::endl;

      // Generate random walks for the graph and measure execution time.
      RandomWalker random_walker(graph, params);
      auto t1 = std::chrono::high_resolution_clock::now();
      random_walker.generateRandomWalks();
      auto t2 = std::chrono::high_resolution_clock::now();
      LOG(INFO) << "Generated " << num_walks_per_vertex
                << " walks for each of " << num_vertices << " vertices "
                << " of length " << walk_length << " in "
                << std::chrono::duration_cast<std::chrono::duration<real_t>>
                    (t2 - t1).count() << " seconds.";

      testRandomWalkSequence(random_walker, graph, params);
      if (params.random_sampling_type ==
          RandomWalkerParams::SAMPLING_TYPE::AVOIDING)
        testAvoidingStrategy(random_walker, graph, params);

      if(params.random_sampling_type ==
          RandomWalkerParams::SAMPLING_TYPE::NON_RETURNING) {
        testNonReturningStrategy(random_walker, graph, params);
      }

    }
  }

  // Test the 'WEIGHTED' random walk sampling strategy by analyzing the
  // statistic properties of its random walks.
  testWeightedStrategyStatistics();

  LOG(INFO) << "Test passed.";

}

