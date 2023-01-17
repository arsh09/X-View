#ifndef X_VIEW_GRAPH_H
#define X_VIEW_GRAPH_H

#include <x_view_core/x_view_types.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <random>

#include <x_view_core/x_view_types.h>

namespace x_view {

/// \brief Property associated to a graph vertex.
struct VertexProperty {

  /// \brief Index of the vertex in the graph.
  /// \note This index should not be used to access the vertex in the graph,
  /// as even if the vertex order is maintained, due to vertex merging
  /// operations it might happen that vertices are deleted from the graph.
  uint64_t index;

  /// \brief Semantic label associated to this graph vertex. This label
  /// corresponds to the same specified in the dataset description.
  int semantic_label;

  /// \brief Name of semantic entity associated to this vertex.
  std::string semantic_entity_name;

  /// \brief Number of pixels contained in this vertex/blob.
  uint64_t num_pixels;

  /// \brief Blob center.
  cv::Point2i center;

  /// \brief 3D location of this vertex expressed in world frame.
  Vector3r location_3d;

  /// \brief Index referring to the last time this vertex has been observed.
  uint64_t last_time_seen_;

  /// \brief Indices of poses from which the vertex was observed.
  std::vector<PoseId> observers;
};

/// \brief Property associated to a graph edge.
struct EdgeProperty {
  /// \brief Index of the first vertex defining this edge.
  /// \note This index should not be used to access the associated vertex, as
  /// it might differ with the storage index of the graph. To access the
  /// source vertex of this edge proceed as follows:
  /// \code{.cpp}
  /// const VertexDescriptor from_v_d = boost::source(edge, graph);
  /// const VertexProperty from_v_p = graph[from_v_d];
  /// \endcode

  uint64_t from;
  /// \brief Index of the second vertex defining this edge.
  /// \note This index should not be used to access the associated vertex, as
  /// it might differ with the storage index of the graph. To access the
  /// target vertex of this edge proceed as follows:
  /// \code{.cpp}
  /// const VertexDescriptor to_v_d = boost::target(edge, graph);
  /// const VertexProperty to_v_p = graph[to_v_d];
  /// \endcode
  uint64_t to;

  /// \brief Integer indicating how many times this edge has been observed.
  uint64_t num_times_seen;
};
/**
 * \brief A graph object represented as an adjacency list.
 * \details
 * First parameter:
 *      What stl container is used to store the edges.
 *      Using setS as edge container to enforce uniqueness of edges, i.e.
 *      edge 1-2 is the same as edge 2-1.
 * Second parameter:
 *      What stl container is used to store the graph vertices.
 * Third parameter:
 *      Directed or undirected graph type.
 * Forth parameter:
 *      Vertex representation.
 * Fifth parameter:
 *      Edge representation.
 * \note The second parameter must be boost::vecS such that each vertex can
 * be accessed directly by the program as follows:
 * \code{.cpp}
 * Graph graph = createSomeGraph();
 * VertexProperty first_vertex = graph[0];
 * VertexProperty tenth_vertex = graph[9];
 * \endcode
 * Otherwise each time one wants to access a vertex it must perform a
 * linear/logarithmic search on the vertex list:
 * \code{.cpp}
 * Graph graph = createSomeGraph();
 * VertexProperty tenth_vertex;
 * auto vertex_iter = boost::vertices(graph);
 * for(int i = 0; vertex_iter.first != vertex_iter.second;
 *     ++i,++vertex_iter.first) {
 *     if(i == 9)
 *         tenth_vertex = graph[*vertex_iter.first];
 * }
 *\endcode
 */
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS,
                              VertexProperty, EdgeProperty> Graph;

/// \brief Access to vertices stored in the graph. Since vertices are stored
/// in a boost::vecS container, their VertexDescriptor corresponds to the
/// index at which they are stored: a VertexDescriptor is an uint64_t type.
typedef boost::graph_traits<Graph>::vertex_descriptor VertexDescriptor;

/// \brief Access to edges stored in the graph.
typedef boost::graph_traits<Graph>::edge_descriptor EdgeDescriptor;

/**
 * \brief Tests if the i-th and the j-th vertex of the graph passed as
 * parameter are linked by an edge.
 * \param v1 Index got by graph[v_1_d].index_ of first vertex descriptor.
 * \param v2 Index got by graph[v_2_d].index_ of second vertex descriptor.
 * \param graph Graph containing the two vertices passed as argument.
 * \return True if an edge exists between vertex with index_ = i
 * and vertex with index_ = j, false otherwise.
 * \note This approach is inefficient as it uses a member of the
 * VertexProperty to check the edge existence, consider using
 * VertexDescriptors whenever possible:
 * \code{.cpp}
 * Graph graph = getSomeGraph();
 * const VertexDescriptor v_1_d = getFirstVertexDescriptor();
 * const VertexDescriptor v_2_d = getSecondVertexDescriptor();
 * // Efficient way:
 * bool vertices_connected_1 = boost.:edge(v_1_d, v_2_d, graph).second;
 * // Inefficient way:
 * const VertexProperty v_1_p = graph[v_1 _d];
 * const VertexProperty v_2_p = graph[v_2 _d];
 * bool vertices_connected_2 =
 *    areVerticesConnectedByIndex(v_1_p.index_, v_2_p.index_, graph);
 *
 * assert(vertices_connected_1  == vertices_connected_2);
 * \endcode
 */
bool areVerticesConnectedByIndex(const int v1, const int v2,
                                 const Graph& graph);

/**
 * \brief Adds an edge between the VertexDescriptors passed as argument if
 * the edge does not exist yet.
 * \param v_1_d VertexDescriptor of first vertex.
 * \param v_2_d VertexDescriptor of second vertex.
 * \param graph Pointer to graph an edge is added to.
 * \return True if the edge has been added, false if the edge was already
 * present.
 */
bool addEdgeBetweenVertices(const VertexDescriptor& v_1_d,
                            const VertexDescriptor& v_2_d, Graph* graph);

/**
 * \brief Removes the edge between the vertex descriptors passed as argument
 * and returns true only if the vertex has been removed correctly.
 * \param v_1_d First vertex descriptor.
 * \param v_2_d Second vertex descriptor.
 * \param graph Pointer to graph to be modified.
 * \return True if the function succeeded in removing the edge, false
 * otherwise.
 */
bool removeEdgeBetweenVertices(const VertexDescriptor& v_1_d,
                               const VertexDescriptor& v_2_d, Graph* graph);


/// \brief Overloaded operator to print a vertex.
std::ostream& operator<<(std::ostream& out, const VertexProperty& v);

/// \brief Overloaded operator to print an edge.
std::ostream& operator<<(std::ostream& out, const EdgeProperty& e);

/// \brief Overloaded operator to print a graph.
std::ostream& operator<<(std::ostream& out, const Graph& graph);

/**
 * \brief Writes the graph passed as argument to the file specified as second
 * argument in the '.dot' file format.
 * \details The file written by this function contains the following
 * information about the graph passed as argument:
 *   - Graph name (same for all graphs: 'semantic_graph')
 *   - An unordered list of vertices with the following properties:
 *     - Vertex identifier encoded through the vertex index property.
 *     - A text label reflecting the semantic entity name of the vertex.
 *     - Fill color: color associated to the semantic entity (see
 *       getColorFromSemanticLabel()).
 *     - Font color: color used for the label (computed such that it is
 *       visible overlayed with the fill color).
 *     - 2D coordinates to be used when rendering the graph (coordinates are
 *       associated to the x and y coordinate of the 3D world coordinate of the
 *       vertex.)
 *     - A comment on the real 3D location of the vertex.
 *   - An unordered list of undirected edges defined by the vertex identifiers
 *     composing the edge (e.g '3 -- 4' refers to edge between vertex with
 *     identifier '3' and vertex with identifier '4').
 *     The 'num_times_seen' property is encoded as the 'penwidth' graphviz
 *     property.
 */
void writeToFile(const Graph& graph, const std::string& filename);

}

#endif //X_VIEW_GRAPH_H
