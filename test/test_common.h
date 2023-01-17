#ifndef X_VIEW_TEST_COMMON_H
#define X_VIEW_TEST_COMMON_H

#include <x_view_core/features/graph.h>
#include <x_view_core/x_view_types.h>

#include <boost/graph/breadth_first_search.hpp>

#include <map>
#include <random>

namespace x_view_test {

void createParameters();

struct GraphConstructionParams {

  GraphConstructionParams()
      : num_vertices(10),
        edge_probability(0.1),
        num_semantic_classes(13),
        seed(0) {}

  /// \brief Number of vertices of the generated graph.
  int num_vertices;
  /// \brief Probability to generate an edge between each pair of vertices.
  x_view::real_t edge_probability;
  /// \brief Number of semantic classes. Each vertex is associated to a
  /// random semantic class (random integer in {0, .., num_semantic_classes-1}
  int num_semantic_classes;
  /// \brief Seed to be used for graph generation.
  uint64_t seed;
};

/**
 * \brief Parameters used to modify the topology of a graph.
 */
struct GraphModifierParams {
  /// \brief Number of new vertices to add to the graph.
  int num_vertices_to_add;
  /// \brief Number of vertices to remove from the graph.
  int num_vertices_to_remove;
  /// \brief Number of new edges to add to the graph.
  int num_edges_to_add;
  /// \brief Number of edges to remove from the graph.
  int num_edges_to_remove;
  /// \brief Number of edges to create between each new vertex and the
  /// existing ones.
  int num_links_for_new_vertices = 2;
  /// \brief Start index to use when adding new vertices.
  int start_vertex_index = -1;
};

/**
 * \brief Generates a random graph with num_vertices vertices, where each
 * pair of vertices is linked by an edge with probability edge_probability. A
 * random semantic class is associated to each vertex of the graph.
 * \param params Parameters used for graph construction.
 */
x_view::Graph generateRandomGraph(const GraphConstructionParams& params);

/**
 * \brief Generates a chain-like graph with num_vertices each of
 * which is assigned to a random semantic class between 0 and
 * num_semantic_classes - 1.
 * \param params Parameters used for graph construction.
 */
x_view::Graph generateChainGraph(const GraphConstructionParams& params);

/**
 * \brief An instance of this class is used to build a subgraph based on K-hops.
 */
class KhopVisitor : public boost::default_bfs_visitor {

 public:
  /// \brief Map which assigns to each VertexDescriptor an integer distance
  /// (number of edges/hops) needed to reach the source vertex in a graph.
  typedef std::map<const x_view::VertexDescriptor, int> DistanceMap;

  KhopVisitor(DistanceMap& dist)
      : dist_(dist) {}

  /**
   * \brief Function called each time an edge is added to the search tree.
   * \details This function is used to compute the distance from the target
   * vertex of the added edge to the source vertex of the BFS algorithm.
   */
  template<typename EdgeT, typename GraphT>
  void tree_edge(EdgeT e, GraphT& g) {
    // Distance to the target it equal to distance to the source (of the
    // edge) increased by one.
    dist_[boost::target(e, g)] = dist_[boost::source(e, g)] + 1;
  }

 private:
  DistanceMap& dist_;
};

/**
 * \brief Extracts a subgraph from a larger graph, defined by a source vertex
 * and all vertices having a maximal distance from the source defined by the
 * passed parameters.
 * \param original Original graph from which the subgraph is extracted.
 * \param source Vertex descriptor indicating which vertex has to be used for
 * graph extraction.
 * \param radius Number of edges to traverse for graph extraction.
 * \return A new graph containing all vertices which lie at most at radius
 * edges of distance from the source vertex passed as argument.
 */
x_view::Graph extractSubgraphAroundVertex(const x_view::Graph& original,
                                          const x_view::VertexDescriptor& source,
                                          const int radius);

/**
 * \brief Modifies the graph pointed by the passed argument following the
 * parameters contained in the second argument.
 * \param graph Pointer pointing to the graph to be modified.
 * \param params Parameters used for modifying the graph.
 * \param rng Random number generator used to randomly create/remove
 * vertices/edges.
 */
void modifyGraph(x_view::Graph* graph, const GraphModifierParams& params,
                 std::mt19937& rng);

/**
 * \brief Changes the 'last_time_seen' property of all vertices of the graph
 * passed as argument.
 * \param graph Pointer to the graph being modified.
 * \param last_time_seen New index to set on all vertices of the graph passed
 * as argument.
 */
void setLastTimeSeen(x_view::Graph* graph, const uint64_t last_time_seen);

}

#endif //X_VIEW_TEST_COMMON_H
