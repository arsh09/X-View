#include "test_random_walk.h"
#include "test_common.h"

#include <x_view_core/x_view_types.h>

namespace x_view_test {

void testRandomWalkSequence(const x_view::RandomWalker& random_walker,
                            const x_view::Graph& graph,
                            const x_view::RandomWalkerParams& params) {
  const auto& all_random_walks = random_walker.getRandomWalks();
  int start_vertex_index = 0;
  for (const auto& random_walks : all_random_walks) {
    for (const auto& random_walk : random_walks) {
      CHECK(areVerticesConnectedByIndex(start_vertex_index,
                                        random_walk[0]->index, graph) ||
          start_vertex_index == random_walk[0]->index)
      << "Start vertex " << start_vertex_index << " and vertex "
      << random_walk[0]->index << " appear in a random walk "
      << "but there is no edge between them.";
      for (int i = 0; i < random_walk.size() - 1; ++i) {
        const int from_index = random_walk[i]->index;
        const int to_index = random_walk[i + 1]->index;
        CHECK(areVerticesConnectedByIndex(from_index, to_index, graph) ||
            from_index == to_index)
        << "Vertex " << from_index << " and vertex " << to_index
        << " appear in a random walk at position " << i << " and " << i + 1
        << " respectively in the random walk starting at vertex "
        << start_vertex_index
        << " but there is no edge between them in the graph.";
      }
    }
    ++start_vertex_index;
  }
}

void testAvoidingStrategy(const x_view::RandomWalker& random_walker,
                          const x_view::Graph& graph,
                          const x_view::RandomWalkerParams& params) {
  const auto& all_random_walks = random_walker.getRandomWalks();
  int start_vertex_index = 0;
  for (const auto& random_walks : all_random_walks) {
    for (const auto& random_walk : random_walks) {
      for (int i = 0; i < random_walk.size() - 1; ++i) {
        const int from_index = random_walk[i]->index;
        const int from_label = random_walk[i]->semantic_label;
        // Using from_index as key for extracting vertex descriptor from
        // graph because in this test the index member of the VertexProperty
        // corresponds to the VertexDescriptor index associated to the
        // VertexProperty.
        const auto& from_vertex_descriptor = boost::vertex(from_index, graph);
        // Get the neighbors of the current vertex.
        auto from_vertex_neighbors =
            boost::adjacent_vertices(from_vertex_descriptor, graph);
        bool all_neighbors_have_same_label = true;
        // Check whether all neighbors have the same semantic label as the
        // current vertex or not.
        for (; from_vertex_neighbors.first != from_vertex_neighbors.second;
               ++from_vertex_neighbors.first) {
          if (graph[*from_vertex_neighbors.first].semantic_label !=
              from_label) {
            all_neighbors_have_same_label = false;
            break;
          }
        }
        // Only verify avoiding property if there is at least one neighbor
        // with different semantic label.
        if (!all_neighbors_have_same_label) {
          const int to_index = random_walk[i + 1]->index;
          const int to_label = random_walk[i + 1]->semantic_label;
          CHECK(from_label != to_label)
          << "Even though the RandomWalker class is using the avoiding "
          << "strategy, there is a random walk starting from vertex "
          << start_vertex_index << " with an edge between nodes "
          << from_index << " and " << to_index
          << " that have the same semantic label " << from_label;
        }
      }
    }
    ++start_vertex_index;
  }
}

void testNonReturningStrategy(const x_view::RandomWalker& random_walker,
                              const x_view::Graph& graph,
                              const x_view::RandomWalkerParams& params) {
  const auto& all_random_walks = random_walker.getRandomWalks();
  uint64_t start_vertex_index = 0;
  for (const x_view::RandomWalker::RandomWalks
        & random_walks : all_random_walks) {
    x_view::VertexProperty start_vertex_property;
    start_vertex_property.index = start_vertex_index;

    for (x_view::RandomWalker::RandomWalk random_walk : random_walks) {
      // Insert the start_vertex_index into the random walk to facilitate
      // checking the non-returning property in a loop.
      random_walk.insert(random_walk.begin(), &start_vertex_property);
      for (uint64_t i = 1; i < random_walk.size() - 1; ++i) {
        // The non-returning property must only be satisfied for vertex B in
        // a random walk of the form '[...] - B - [...]' if B has a larger
        // degree than 1, i.e. at least two neighbors.
        const x_view::VertexDescriptor center_vertex_index =
            random_walk[i]->index;
        if (boost::degree(center_vertex_index, graph) > 1) {
          CHECK(random_walk[i - 1]->index != random_walk[i + 1]->index)
          << "Random walk presents a returning walk for random walk "
          << "with start_vertex_index " << start_vertex_index << " "
          << "at step " << i - 1 << " and " << i + 1 << " with central "
          << "index being " << center_vertex_index << " which has "
          << boost::degree(center_vertex_index, graph) << " neighbors.";
        }
      }
    }
    ++start_vertex_index;
  }
}

void testWeightedStrategyStatistics() {

  // Create a chain-like graph.
  GraphConstructionParams graph_construction_params;
  graph_construction_params.num_vertices = 10;
  graph_construction_params.num_semantic_classes = 10;
  graph_construction_params.seed = 0;

  x_view::Graph chain_graph = generateChainGraph(graph_construction_params);

  // Test for different multiplication factors between successive vertices.
  std::vector<int> multiplication_factors = {1, 2, 3};
  for (const int multiplication_factor : multiplication_factors) {

    LOG(INFO) << "Testing weighted strategy statistics with multiplication "
              << "factor " << multiplication_factor << ".";

    // A vector whose elements represent the ratio of random walks that start
    // from vertex i and go to vertex i+1 over the ones that go to vertex i-1;
    std::vector<x_view::real_t> expected_next_vertex_ratio(
        graph_construction_params.num_vertices, multiplication_factor);

    // Special case for vertex 0, since it is linked to vertex 1 on one side
    // and on the last vertex of the graph from the other side.
    if (multiplication_factor > 1) {
      // One weight (left) is huge, other (right) is small.
      expected_next_vertex_ratio[0] = 0.f;
    } else if (multiplication_factor < 1) {
      // One weight (left) is small, other (right) is huge.
      expected_next_vertex_ratio[0] = 1.f;
    }

    // Set the edges weights such that the weights are all different and
    // increasing by doubling their value.
    const auto edges = boost::edges(chain_graph);
    uint64_t num_times_seen = 1;
    for (auto iter = edges.first; iter != edges.second; ++iter) {
      x_view::EdgeProperty& e_p = chain_graph[*iter];
      e_p.num_times_seen = num_times_seen;
      num_times_seen *= multiplication_factor;
    }

    LOG(INFO) << "Chain graph for weighted strategy: \n" << chain_graph << ".";

    x_view::RandomWalkerParams random_walker_params;
    // We are interested in testing how a vertex is chosen independently of the
    // walk length.
    random_walker_params.walk_length = 1;
    // Choose a large number of random walks per vertex in order to have better
    // (and smoother) statistics.
    random_walker_params.num_walks = 10000;
    random_walker_params.random_sampling_type =
        x_view::RandomWalkerParams::SAMPLING_TYPE::WEIGHTED;
    x_view::RandomWalker random_walker(chain_graph, random_walker_params);
    random_walker.generateRandomWalks();

    for (int i = 0; i < graph_construction_params.num_vertices; ++i) {
      auto& random_walks = random_walker.getRandomWalksOfVertex(i);
      uint64_t starts_i_min_1 = 0;
      uint64_t starts_i_plus_1 = 0;
      for (auto& random_walk: random_walks) {
        if (random_walk.front()->index ==
            (i + graph_construction_params.num_vertices - 1)
                % graph_construction_params.num_vertices)
          ++starts_i_min_1;
        else
          ++starts_i_plus_1;
      }

      const x_view::real_t ratio =
          static_cast<x_view::real_t>(starts_i_plus_1) / starts_i_min_1;
      CHECK_NEAR(expected_next_vertex_ratio[i], ratio, 0.15);
    }

    LOG(INFO) << "Statistic test passed.";
  }
}

}
