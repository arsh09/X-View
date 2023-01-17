#include "test_common.h"
#include "test_graph_landmark_matcher.h"

#include <x_view_core/features/graph_descriptor.h>
#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/matchers/graph_matcher.h>
#include <x_view_core/x_view_types.h>

#include <boost/graph/random.hpp>

using namespace x_view;
using namespace x_view_test;

typedef std::shared_ptr<GraphMatcher> GraphMatcherPtr;

#define CAST(from, to) std::dynamic_pointer_cast<to>(from)

/**
 * \brief This function generates multiple random graphs with random topology
 * and studies the accuracy of vertex match using random walks.
 * \param seed Seed used for random graph extraction.
 */
void randomGraphAccuracyStudy(const uint64_tseed) {

  const int runs_per_settings = 10;

  std::vector<int> num_labels{3, 5, 13, 20};

  std::vector<struct{int num_vertices_;real_t edge_prob_;}> global_graph_params{
      {50, 0.1}, {50, 0.2}, {500, 0.005}, {500, 0.01}};

  std::vector<int> extraction_radius{1, 2, 3, 4};

  std::vector<int> num_walks = {10, 50, 250};
  std::vector<int> walk_lengths = {1, 2, 3, 4};

  std::mt19937 rng(seed);

  std::cout << std::setfill(' ');
  std::cout << std::right << std::setw(12) << "num labels"
            << std::right << std::setw(12) << "num gl. v."
            << std::right << std::setw(12) << "edge prob."
            << std::right << std::setw(12) << "radius"
            << std::right << std::setw(12) << "num. walks"
            << std::right << std::setw(12) << "walk len."
            << std::right << std::setw(12) << "extract. v."
            << std::left << std::setw(12) << "  acc. (mean)"
            << std::left << std::setw(12) << "  acc. (std)" << std::endl;

  for (const int num_label : num_labels) {
    global_dataset_ptr =
        std::make_shared<AbstractDataset>(AbstractDataset(num_label));
    for (const auto& graph_params : global_graph_params) {
      const int global_num_vertices = graph_params.num_vertices_;
      const real_t global_edge_prob = graph_params.edge_prob_;
      GraphConstructionParams graph_construction_params;
      graph_construction_params.num_vertices_ = global_num_vertices;
      graph_construction_params.edge_probability_ = global_edge_prob;
      graph_construction_params.seed_ = seed;
      graph_construction_params.num_semantic_classes_ = num_label;
      const Graph global_graph = generateRandomGraph(graph_construction_params);
      for (const int radius : extraction_radius) {
        for (const int num_walk : num_walks) {
          for (const int walk_length : walk_lengths) {
            RandomWalkerParams random_walker_params;
            random_walker_params.walk_length_ = walk_length;
            random_walker_params.num_walks_ = num_walk;
            random_walker_params.random_sampling_type_ =
                RandomWalkerParams::RANDOM_SAMPLING_TYPE::AVOID_SAME;

            real_t mean_num_extracted_vertices = 0.0;
            real_t cum_accuracy = 0.0;
            real_t cum_accuracy_squared = 0.0;
            for (int i = 0; i < runs_per_settings; ++i) {
              const VertexDescriptor seed_vertex =
                  boost::random_vertex(global_graph, rng);
              Graph extracted_graph =
                  extractSubgraphAroundVertex(global_graph,
                                              seed_vertex,
                                              radius);
              mean_num_extracted_vertices +=
                  boost::num_vertices(extracted_graph);
              GraphMatcherPtr graph_matcher_ptr =
                  CAST(GraphMatcher::create(random_walker_params,
                                            VertexSimilarity::SCORE_TYPE::HARD),
                       GraphMatcher);

              // Add the base graph to the matcher.
              auto ignore_result = graph_matcher_ptr->match(global_graph);

              // Match the subgraph to the entire graph.
              auto matching_result = graph_matcher_ptr->match(extracted_graph);

              // Retrieve the similarity matrix.
              Eigen::MatrixXf random_similarity =
                  CAST(matching_result, GraphMatcher::GraphMatchingResult)
                      ->getSimilarityMatrix();

              const real_t accuracy =
                  similarityAccuracy({global_graph, extracted_graph},
                                     random_similarity);

              cum_accuracy += accuracy;
              cum_accuracy_squared += accuracy * accuracy;
            }

            mean_num_extracted_vertices /= runs_per_settings;
            const real_t mean_accuracy = cum_accuracy / runs_per_settings;
            const real_t std_accuracy = std::sqrt(cum_accuracy_squared /
                runs_per_settings - mean_accuracy * mean_accuracy);
            std::cout << std::right << std::setw(12) << num_label
                      << std::right << std::setw(12) << global_num_vertices
                      << std::right << std::setw(12) << global_edge_prob
                      << std::right << std::setw(12) << radius
                      << std::right << std::setw(12) << num_walk
                      << std::right << std::setw(12) << walk_length
                      << std::right << std::setw(12)
                      << mean_num_extracted_vertices
                      << "  " << std::left << std::setw(12) << mean_accuracy
                      << "  " << std::left << std::setw(12) << std_accuracy
                      << std::endl;
          }
        }
      }
    }
  }
}
