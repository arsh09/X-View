#include "test_graph_landmark_matcher.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/matchers/graph_matcher.h>
#include <x_view_core/x_view_locator.h>
#include <x_view_core/x_view_tools.h>

#include <boost/graph/connected_components.hpp>
#include <boost/graph/random.hpp>

namespace x_view_test {

typedef std::shared_ptr<GraphMatcher> GraphMatcherPtr;

#define CAST(from, to) std::dynamic_pointer_cast<to>(from)

void testChainGraph(const uint64_t seed) {

  const auto& dataset = Locator::getDataset();

  // Define parameters for generating the graphs.
  GraphConstructionParams construction_params;
  construction_params.num_vertices = 50;
  construction_params.num_semantic_classes
      = dataset->numSemanticClasses();
  construction_params.seed = seed;

  // Define parameters for modifying the graphs.
  GraphModifierParams modifier_params;
  modifier_params.num_vertices_to_add = 0;
  modifier_params.num_links_for_new_vertices = 0;
  modifier_params.num_vertices_to_remove = 0;
  modifier_params.num_edges_to_add = 0;
  modifier_params.num_edges_to_remove = 0;

  // Size of the extracted subgraph.
  const int extraction_radius = 6;

  GraphPair graph_pair_chain = generateChainGraphPair(construction_params,
                                                      modifier_params,
                                                      extraction_radius);

  std::string base_dot_file_name = getOutputDirectory() + "/chain_base.dot";
  std::string sub_dot_file_name = getOutputDirectory() + "/chain_sub.dot";
  writeToFile(graph_pair_chain.base_graph, base_dot_file_name);
  writeToFile(graph_pair_chain.sub_graph, sub_dot_file_name);

  // Define parameters for random walk extraction.
  RandomWalkerParams random_walker_params;
  random_walker_params.walk_length = 3;
  random_walker_params.num_walks = 200;
  random_walker_params.random_sampling_type =
      RandomWalkerParams::SAMPLING_TYPE::AVOIDING;

  GraphMatcherPtr graph_matcher_ptr =
      CAST(GraphMatcher::create(random_walker_params,
                                VertexSimilarity::SCORE_TYPE::WEIGHTED),
           GraphMatcher);

  CHECK_NOTNULL(graph_matcher_ptr.get());

  // Add the base graph to the matcher.
  graph_matcher_ptr->addDescriptor(graph_pair_chain.base_graph);

  // Match the subgraph to the entire graph.
  auto matching_result = graph_matcher_ptr->match(graph_pair_chain.sub_graph);

  const real_t accuracy = similarityAccuracy(graph_pair_chain, matching_result);
  std::cout << "Chain matching has accuracy of " << accuracy << std::endl;

#ifdef X_VIEW_DEBUG
  // Compute similarity matrices.
  const GraphMatcher::SimilarityMatrixType& similarity_matrix =
      CAST(matching_result, const GraphMatcher::GraphMatchingResult)->getSimilarityMatrix();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_colwise =
      CAST(matching_result, const GraphMatcher::GraphMatchingResult)->computeMaxSimilarityColwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_rowwise =
      CAST(matching_result, const GraphMatcher::GraphMatchingResult)->computeMaxSimilarityRowwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_agree =
      max_similarity_colwise.cwiseProduct(max_similarity_rowwise);

  // Display the computed similarities.
  const cv::Mat similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(similarity_matrix);
  cv::imshow("Vertex similarity", similarity_image);

  const cv::Mat max_col_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_colwise);
  cv::imshow("Max col similarity", max_col_similarity_image);

  const cv::Mat max_row_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_rowwise);
  cv::imshow("Max row similarity", max_row_similarity_image);

  const cv::Mat max_agree_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_agree);
  cv::imshow("Max agree similarity", max_agree_similarity_image);

  cv::waitKey();
#endif

}

void testRandomGraph(const uint64_t seed) {

  const auto& dataset = Locator::getDataset();

  // Define parameter for generating the graphs.
  GraphConstructionParams construction_params;
  construction_params.num_vertices = 500;
  construction_params.edge_probability = 0.001;
  construction_params.num_semantic_classes = dataset->numSemanticClasses();
  construction_params.seed = seed;

  // Define parameters for modifying the graphs.
  GraphModifierParams modifier_params;
  modifier_params.num_vertices_to_add = 50;
  modifier_params.num_links_for_new_vertices = 2;
  modifier_params.num_vertices_to_remove = 50;
  modifier_params.num_edges_to_add = 20;
  modifier_params.num_edges_to_remove = 20;

  const int extraction_radius = 3;

  GraphPair graph_pair_random = generateRandomGraphPair(construction_params,
                                                        modifier_params,
                                                        extraction_radius);

  std::string base_dot_file_name = getOutputDirectory() + "random_base.dot";
  std::string sub_dot_file_name = getOutputDirectory() + "random_sub.dot";
  writeToFile(graph_pair_random.base_graph, base_dot_file_name);
  writeToFile(graph_pair_random.sub_graph, sub_dot_file_name);

  // Define parameters for random walk extraction.
  RandomWalkerParams random_walker_params;
  random_walker_params.walk_length = 3;
  random_walker_params.num_walks = 200;
  random_walker_params.random_sampling_type =
      RandomWalkerParams::SAMPLING_TYPE::AVOIDING;

  GraphMatcherPtr graph_matcher_ptr =
      CAST(GraphMatcher::create(random_walker_params,
                                VertexSimilarity::SCORE_TYPE::WEIGHTED),
           GraphMatcher);

  CHECK_NOTNULL(graph_matcher_ptr.get());

  // Add the base graph to the matcher.
  graph_matcher_ptr->addDescriptor(graph_pair_random.base_graph);

  // Match the subgraph to the entire graph.
  auto matching_result = graph_matcher_ptr->match(graph_pair_random.sub_graph);

  const real_t accuracy =
      similarityAccuracy(graph_pair_random, matching_result);

  std::cout << "Random matching has accuracy of " << accuracy << std::endl;

#ifdef X_VIEW_DEBUG
  // Compute similarity matrices.
  const GraphMatcher::SimilarityMatrixType& similarity_matrix =
      CAST(matching_result, const GraphMatcher::GraphMatchingResult)->getSimilarityMatrix();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_colwise =
      CAST(matching_result, const GraphMatcher::GraphMatchingResult)->computeMaxSimilarityColwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_rowwise =
      CAST(matching_result, const GraphMatcher::GraphMatchingResult)->computeMaxSimilarityRowwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_agree =
      max_similarity_colwise.cwiseProduct(max_similarity_rowwise);

  // Display the computed similarities.
  const cv::Mat similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(similarity_matrix);
  cv::imshow("Vertex similarity", similarity_image);

  const cv::Mat max_col_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_colwise);
  cv::imshow("Max col similarity", max_col_similarity_image);

  const cv::Mat max_row_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_rowwise);
  cv::imshow("Max row similarity", max_row_similarity_image);

  const cv::Mat max_agree_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_agree);
  cv::imshow("Max agree similarity", max_agree_similarity_image);

  cv::waitKey();
#endif
}

GraphPair generateChainGraphPair(const GraphConstructionParams& construction_params,
                                 const GraphModifierParams& modifier_params,
                                 const int extraction_radius) {

  // Seed randomization.
  std::mt19937 rng(construction_params.seed);

  GraphPair graph_pair;
  // Generate a chain graph with the parameters specified above.
  graph_pair.base_graph = generateChainGraph(construction_params);

  const VertexDescriptor source_vertex =
      boost::random_vertex(graph_pair.base_graph, rng);
  graph_pair.sub_graph =
      extractSubgraphAroundVertex(graph_pair.base_graph, source_vertex,
                                  extraction_radius);

  // Need to set the index at which the newly inserted vertices start to
  // avoid false positive matches between newly inserted vertices and
  // existing ones: newly existing vertices have indices starting at the
  // maximal value of the base graph indices. In this way there are no two
  // vertices with the same index.
  uint64_t max_index = 0;
  const auto vertices = boost::vertices(graph_pair.base_graph);
  for(auto iter = vertices.first; iter != vertices.second; ++iter) {
    max_index = std::max(max_index, graph_pair.base_graph[*iter].index);
  }
  GraphModifierParams updated_modifier_params = modifier_params;

  updated_modifier_params.start_vertex_index = int(max_index + 1);
  // Effectively add and remove vertices and edges from the graph.
  modifyGraph(&graph_pair.sub_graph, updated_modifier_params, rng);
  const uint64_t new_last_time_seen = 1;
  setLastTimeSeen(&graph_pair.sub_graph, new_last_time_seen);


  LOG(INFO) << "Generated chain graph with "
            << boost::num_vertices(graph_pair.base_graph)
            << " vertices and extracted a subgraph with "
            << boost::num_vertices(graph_pair.sub_graph) << " vertices";

  return graph_pair;
}

GraphPair generateRandomGraphPair(const GraphConstructionParams& construction_params,
                                  const GraphModifierParams& modifier_params,
                                  const int extraction_radius) {

  // Seed randomization.
  std::mt19937 rng(construction_params.seed);

  GraphPair graph_pair;
  // Generate a random graph with the parameters specified above.
  graph_pair.base_graph = generateRandomGraph(construction_params);

  const VertexDescriptor source_vertex =
      boost::random_vertex(graph_pair.base_graph, rng);
  graph_pair.sub_graph =
      extractSubgraphAroundVertex(graph_pair.base_graph, source_vertex,
                                  extraction_radius);

  // Need to set the index at which the newly inserted vertices start to
  // avoid false positive matches between newly inserted vertices and
  // existing ones: newly existing vertices have indices starting at the
  // maximal value of the base graph indices. In this way there are no two
  // vertices with the same index.
  uint64_t max_index = 0;
  const auto vertices = boost::vertices(graph_pair.base_graph);
  for(auto iter = vertices.first; iter != vertices.second; ++iter) {
    max_index = std::max(max_index, graph_pair.base_graph[*iter].index);
  }
  GraphModifierParams updated_modifier_params = modifier_params;

  updated_modifier_params.start_vertex_index = int(max_index + 1);

  // Effectively add and remove vertices and edges from the graph.
  modifyGraph(&graph_pair.sub_graph, updated_modifier_params, rng);
  const uint64_t new_last_time_seen = 1;
  setLastTimeSeen(&graph_pair.sub_graph, new_last_time_seen);

  LOG(INFO) << "Generated random graph with "
            << boost::num_vertices(graph_pair.base_graph)
            << " vertices and extracted a subgraph with "
            << boost::num_vertices(graph_pair.sub_graph) << " vertices";

  return graph_pair;
}

real_t similarityAccuracy(const GraphPair& graph_pair,
                         const AbstractMatcher::MatchingResultPtr& matching_result_ptr) {
  const Graph& base_graph = graph_pair.base_graph;
  const Graph& sub_graph = graph_pair.sub_graph;

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_colwise =
      CAST(matching_result_ptr, GraphMatcher::GraphMatchingResult)
          ->computeMaxSimilarityColwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_rowwise =
      CAST(matching_result_ptr, GraphMatcher::GraphMatchingResult)
          ->computeMaxSimilarityRowwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_agree =
      max_similarity_colwise.cwiseProduct(max_similarity_rowwise);

  int correct_matches = 0;
  int num_proposed_matches = 0;
  // Loop over agree similarity matrix and check if the agreed match is a true
  // match.
  for (int i = 0; i < max_similarity_agree.rows(); ++i) {
    for (int j = 0; j < max_similarity_agree.cols(); ++j) {
      if (max_similarity_agree(i, j) == true) {
        // Retrieve the indices of the i-th and j-th vertices of the
        // base_graph and sub_graph respectively.
        const VertexProperty v_i_base = base_graph[i];
        const VertexProperty v_j_sub = sub_graph[j];

        if (v_i_base.index == v_j_sub.index)
          ++correct_matches;
        ++num_proposed_matches;
      }
    }
  }
  return static_cast<real_t>(correct_matches) / num_proposed_matches;
}

}

