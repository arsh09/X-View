#ifndef X_VIEW_GRAPH_MERGER_H
#define X_VIEW_GRAPH_MERGER_H

#include <x_view_core/features/graph.h>
#include <x_view_core/matchers/graph_matcher/graph_matcher.h>

#include <queue>

namespace x_view {

/**
 * \brief Parameters used by the GraphMerger class.
 */
struct GraphMergerParameters {
  GraphMergerParameters()
    : time_window(std::numeric_limits<uint64_t>::max()),
      similarity_threshold(0.f),
      distance_threshold(std::numeric_limits<real_t>::max())
  {}

  GraphMergerParameters(const uint64_t time_window,
                        const real_t similarity_threshold,
                        const real_t distance_threshold)
      : time_window(time_window),
        similarity_threshold(similarity_threshold),
        distance_threshold(distance_threshold) {}

  /// \brief Time window used to select which candidate matching vertices
  /// should be merged together. In particular, if a vertex is candidate to
  /// be merged with another, the merge only takes place if the time-distance
  /// between the two vertices is smaller or equal to the allowed time window
  /// defined by time_window_.
  uint64_t time_window;

  /// \brief Only vertices whose semantic similarity is greater than this
  /// parameter are considered in the routine merging two graphs together.
  real_t similarity_threshold;

  /// \brief Only allow to match vertices if their euclidean distance is
  /// smaller than this threshold.
  real_t distance_threshold;
};

/**
 * \brief This class implements the routine for merging two semantic graphs
 * based on the semantic similarity between the graph vertices.
 */
class GraphMerger {

  /// \brief Datastructure used to keep track of which vertices of the query
  /// graph still need to be "linked" to the database graph.
  typedef std::queue<VertexDescriptor> DescriptorQueue;

 public:

  /**
   * \brief Constructor of the graph merger instance.
   * \param database_graph Graph structure representing the database graph.
   * \param query_graph Graph structure representing the query graph to be
   * merged into the database graph.
   * \param matching_result Object resulting from the matching between the
   * query and the database graph. The similarity matrix contained in this
   * structure is used as mean for deciding which vertex of the query graph
   * is associated with which vertex of the database graph.
   * \param graph_merger_parameters Parameters used for merging two graphs
   * together.
   */
  GraphMerger(const Graph& database_graph, const Graph& query_graph,
              const GraphMatcher::GraphMatchingResult& matching_result,
              const GraphMergerParameters& graph_merger_parameters);

  /**
   * \brief Performs the merging operation between the query and the database
   * graph.
   * \return A new graph representing the merged version of the query and the
   * database graph.
   * \details The database graph is usually much larger than the query graph.
   * For this reason, the graph resulting from merging is initialized as a
   * copy of the database graph and extended with the vertices of the query
   * graph.
   */
  const Graph computeMergedGraph();

  /**
   * \brief This function cleans the graph passed as argument by merging
   * together all pairs of vertices which fulfill proximity/similarity
   * properties.
   * \param merge_distance Vertices whose Euclidean distance is larger than
   * this parameter are not merged together.
   * \param graph Graph to be cleaned.
   * \details Given two vertices v_1 and v_2, if their euclidean distance is
   * small enough, and their semantic label is identical, this function
   * merges them together. The resulting vertex v_m is a vertex whose edges
   * correspond to the union of the edges of v_1 and v_2.
   */
  static void mergeDuplicates(const real_t merge_distance, Graph* graph);

  /**
   * \brief This function creates an edge between each pair of vertices
   * belonging to the graph passed as argument whose Euclidean distance is
   * smaller than the parameter passed as argument.
   * \param max_link_distance Parameter specifying which vertex pair should
   * be linked together by a new edge. If vertex v_i and vertex v_j have a
   * lie at a distance closer than this parameter, a new edge (if non
   * existing) is added to the graph linking them.
   * \param graph Graph to be modified by adding edges where necessary.
   */
  static void linkCloseVertices(const real_t max_link_distance, Graph* graph);

  /**
   * \brief Function to test if the vertices associated to the vertex
   * descriptors passed as argument should be merged into a unique vertex or
   * not.
   * \param v_d_1 VertexDescriptor associated to the first vertex being
   * queried for merging.
   * \param v_d_2 VertexDescriptor associated to the second vertex being
   * queried for merging.
   * \param graph Const reference to the graph structure containing the
   * vertices passed as argument.
   * \param merge_distance Vertices whose euclidean distance is larger than
   * this parameter are not merged together.
   * \return True if the vertices associated with the passed parameters
   * should be merged together, false otherwise.
   * \details Two vertices should be merged together only if their semantic
   * label is identical, and if their euclidean distance in world space is
   * smaller than a fixed threshold.
   */
  static const bool verticesShouldBeMerged(const VertexDescriptor v_d_1,
                                           const VertexDescriptor v_d_2,
                                           const Graph& graph,
                                           const real_t merge_distance);

 private:

  /// \brief Const reference to the database graph.
  const Graph& database_graph_;

  /// \brief Const reference to the query graph to be merged into the
  /// database graph.
  const Graph& query_graph_;

  /// \brief Graph resulting from the merging operation.
  Graph merged_graph_;


  /// \brief Structure containing information about the similarities between
  /// the vertices in the query and in the database graph.
  const GraphMatcher::GraphMatchingResult& matching_result_;


  /// \brief Queue containing all vertex descriptors of the query graph which
  /// still need to be merged into the database graph.
  DescriptorQueue still_to_process_;

  /// \brief This map maps the vertices of the query graph to the associated
  /// vertices in the database graph.
  std::unordered_map<VertexDescriptor, VertexDescriptor> query_in_db_;

  /// \brief Index assigned to the vertices being added to the merged graph.
  uint64_t current_vertex_index_;

  /// \brief Vector containign references to vertices belonging to the query
  /// graph which have been matched to a vertex of the database graph.
  std::vector<VertexDescriptor> matched_vertices_;


  /// \brief Parameters used during graph merging.
  const GraphMergerParameters& graph_merger_parameters_;

  void addVertexToMergedGraph(const VertexDescriptor& source_in_query_graph);

  /**
   * \brief This function tries to merge the query_graph_ to the
   * merged_graph_ by merging and linking together close vertices.
   * This function is executed whenever no matches are found between the
   * query and the database graph.
   */
  void linkUnmatchedQueryGraph();

  /**
   * \brief Computes the temporal distance between the i-th vertex of the
   * database graph and the j-th vertex of the query graph.
   * \param i Index of the database vertex to be considered.
   * \param j Index of the query vertex to be considered.
   * \return The temporal distance between the two vertices indicated by the
   * indices passed as argument. This value is computed by subtracting the
   * 'last_time_seen' value of the database vertex from the 'last_time_seen'
   * value of the query vertex.
   */
  const uint64_t temporalDistance(const uint64_t i, const uint64_t j) const;

  /**
   * \brief Computes the euclidean distance between the i-th vertex of the
   * database graph and the j-th vertex of the query graph.
   * \param i Index of the database vertex to be considered.
   * \param j Index of the query vertex to be considered.
   * \return The euclidean distance between the two vertices indicated by the
   * indices passed as argument. This value is computed by subtracting the
   * 'location_3d' property of the database vertex from the 'location_3d'
   * value of the query vertex and by taking its norm.
   */
  const real_t spatialDistance(const uint64_t i, const uint64_t j) const;

};

}

#endif //X_VIEW_GRAPH_MERGER_H
