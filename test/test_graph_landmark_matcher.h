#ifndef X_VIEW_TEST_GRAPH_LANDMARK_MATCHER_H
#define X_VIEW_TEST_GRAPH_LANDMARK_MATCHER_H
#include "test_common.h"

#include <x_view_core/features/graph.h>
#include <x_view_core/matchers/graph_matcher.h>

#include <Eigen/Core>

#include <random>

using namespace x_view;

namespace x_view_test {

/**
 * \brief Tests a chain-like graph.
 * \param seed Seed used to randomly generate the chain-like graph.
 */
void testChainGraph(const uint64_t seed);

/**
 * \brief Tests a graph with random topology.
 * \param seed Seed used to randomly generate the random graph.
 */
void testRandomGraph(const uint64_t seed);


/**
 * \brief Small container used to combine a base graph with a new subgraph.
 */
struct GraphPair {
  Graph base_graph;
  Graph sub_graph;
};

/**
 * \brief Generates a new graph with a chain topology.
 * \param construction_params Parameters used to construct the graph.
 * \param modifier_params Parameters used to modify the subgraph extracted
 * from the generated base_graph.
 * \param extraction_radius Radius used for graph extraction.
 * \return A GraphPair object, containing the generated graph with
 * corresponding extracted subgraph.
 */
GraphPair generateChainGraphPair(const GraphConstructionParams& construction_params,
                                 const GraphModifierParams& modifier_params,
                                 const int extraction_radius);

/**
 * \brief Generates a new graph with a random topology.
 * \param construction_params Parameters used to construct the graph.
 * \param modifier_params Parameters used to modify the subgraph extracted
 * from the generated base_graph_.
 * \param extraction_radius Radius used for graph extraction.
 * \return A GraphPair object, containing the generated graph with
 * corresponding extracted subgraph.
 */
GraphPair generateRandomGraphPair(const GraphConstructionParams& construction_params,
                                  const GraphModifierParams& modifier_params,
                                  const int extraction_radius);

/**
 * \brief Computes the accuracy between the matched vertices contained in the
 * matching_result_ptr passed as argument related to the graphs contained in
 * the graph_pair parameter.
 * \param graph_pair Struct containing the two graphs being matched.
 * \param matching_result_ptr Pointer pointing to the GraphMatchingResult
 * containing the similarity matrix computed between the graphs contained in
 * the first parameter.
 * \return Accuracy of the matching.
 */
real_t similarityAccuracy(const GraphPair& graph_pair,
                         const AbstractMatcher::MatchingResultPtr&
                         matching_result_ptr);

}

#endif //X_VIEW_TEST_GRAPH_LANDMARK_MATCHER_H
