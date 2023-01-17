#include "test_graph_duplicate_removal.h"

#include "test_common.h"

#include <x_view_core/matchers/graph_matcher/graph_merger.h>
#include <x_view_core/x_view_locator.h>
#include <x_view_core/x_view_tools.h>
#include <x_view_core/x_view_types.h>

#include <boost/graph/random.hpp>

#include <random>

namespace x_view_test {

void testDuplicatesChain() {

  const uint64_t seed = 0;
  std::mt19937 rng(seed);

  // Create an instance of a chain graph.
  GraphConstructionParams construction_params;
  construction_params.num_vertices = 20;
  construction_params.num_semantic_classes =
      x_view::Locator::getDataset()->numSemanticClasses();
  construction_params.seed = 0;

  x_view::Graph database_graph = generateChainGraph(construction_params);

  // Manually insert a vertex into the existing chain graph, with very
  // similar coordinates as an existing vertex.
  for(int c = 0; c < 2; ++c) {
    const uint64_t anchor_vertex = boost::random_vertex(database_graph, rng);
    const x_view::VertexProperty& anchor_v_p = database_graph[anchor_vertex];
    const auto anchor_neighbors =
        boost::adjacent_vertices(anchor_vertex, database_graph);
    const x_view::VertexProperty& anchor_neighbor =
        database_graph[*anchor_neighbors.first];

    x_view::VertexProperty mergeable_v_p = anchor_v_p;
    // Modify the 3D location of the new vertex.
    mergeable_v_p.location_3d[0] += 0.1;
    // Increase the index of the newly added vertex.
    mergeable_v_p.index = boost::num_vertices(database_graph);

    const uint64_t new_v_d = boost::add_vertex(mergeable_v_p, database_graph);

    // Add some edges from the mergeable to existing vertices of the graph.
    const uint64_t num_times_seen = 1;
    for (int i = 0; i < 4; ) {
      const uint64_t linked_v_d = boost::random_vertex(database_graph, rng);
      const x_view::VertexProperty& linked_v_p = database_graph[linked_v_d];
      const x_view::EdgeProperty linked_e_p{
          mergeable_v_p.index, linked_v_p.index, num_times_seen
      };
      if(new_v_d != linked_v_d && boost::edge(new_v_d, linked_v_d,
                                              database_graph).second == false) {
        boost::add_edge(new_v_d, linked_v_d, linked_e_p, database_graph);
        ++i;
      }
    }
  }

  const uint64_t num_times_seen = 1;
  boost::add_edge(20, 21, {20, 21, num_times_seen}, database_graph);
  boost::remove_edge(10, 11, database_graph);


  const std::string output_path = x_view::getOutputDirectory();
  x_view::writeToFile(database_graph, output_path + "removal_before.dot");

  const x_view::real_t merge_distance = 0.2;
  x_view::GraphMerger::mergeDuplicates(merge_distance, &database_graph);
  x_view::writeToFile(database_graph, output_path + "removal_after.dot");
}

}
