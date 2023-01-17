#ifndef X_VIEW_TEST_MCGREGOR_MATCHING_H
#define X_VIEW_TEST_MCGREGOR_MATCHING_H

#include <iostream>

#include <gtest/gtest.h>
#include <glog/logging.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/mcgregor_common_subgraphs.hpp>

namespace x_view_test {
//*************************** Graph specifications ***************************//

/// \brief Object representing a vertex of the graph.
struct VertexData {
  enum VertexLabelName {
    NON_DEF_VERTEX_LABEL_NAME = -1, A, B, C
  };

  VertexData() : label(NON_DEF_VERTEX_LABEL_NAME) {}
  VertexData(const VertexLabelName label)
      : label(label) {}
  VertexLabelName label;

  friend std::ostream& operator<<(std::ostream& out, const VertexData& v) {
    return out << v.label;
  }
};

/// \brief Object representing an edge in the graph
struct EdgeData {
  enum EdgeLabelName {
    NON_DEF_EDGE_LABEL_NAME = -1, CLOSE, DISTANT, VISIBLE
  };

  EdgeData() : label(NON_DEF_EDGE_LABEL_NAME) {}
  EdgeData(const EdgeLabelName label)
      : label(label) {}
  EdgeLabelName label;
};

/// \brief a Graph is represented as a boost adjacency list.
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                              VertexData, EdgeData> GraphType;

//**************************** Test specifications ***************************//

/// \brief Interface for graph test objects
class AbstractMaximalSubgraphTest {
 public:
  /// \brief The query_graph_ stored in this object is compared against all
  /// other graphs in the database, a distance measure is computed using the
  /// mcgregor algorithm and this distance is compared against an expected
  /// distance specified by the user.
  virtual void run() const;

 protected:
  /**
   * \brief Base class constructor for graph tests.
   * \param graph_name String describing the concrete implementation of the
   * graph test, it is used for logging purposes only.
   */
  AbstractMaximalSubgraphTest(const std::string& graph_name)
      : graph_name_(graph_name) {};

  /**
   * \brief The graph database is built and the query graph is specified
   */
  virtual void buildGraphDatabase() = 0;

  /**
   * \brief Computes the MCS distance as:|E(q)| +
   * \f$\text{dist}(a,b) = |E(a)| + |E(b)| - 2\times |E(\text{mcs}(a,b))|\f$
   * \param graph_index Index of the database graph to compare against the
   * query graph.
   * \return The MCS distance between the query graph and the database graph
   * associated to the index passed as argument
   */
  int computeMCSDistance(int graph_index) const;

  GraphType query_graph_;
  std::vector<GraphType> graph_database_;
  std::vector<int> expected_distances_;

 private:
  std::string graph_name_;
};

/// \brief test for very basic graphs defined manually
class SimpleGraphsTest : public AbstractMaximalSubgraphTest {
 public:
  SimpleGraphsTest()
      : AbstractMaximalSubgraphTest("SimpleGraph") {
    buildGraphDatabase();
  };

 protected:
  virtual void buildGraphDatabase() override;
};

/// \brief test for the graphs reported in the paper
/// https://openproceedings.org/2012/conf/edbt/ZhuQYC12.pdf
class PaperGraphsTest : public AbstractMaximalSubgraphTest {
 public:
  PaperGraphsTest()
      : AbstractMaximalSubgraphTest("PaperGraph") {
    buildGraphDatabase();
  };

 protected:
  virtual void buildGraphDatabase() override;
};

//************************** Callback specifications *************************//

/// \brief Global variable that counts edges in maximal common subgraph.
/// \details It is ugly to have it global, but it is not possible to store as
/// a member variable of SubgraphCallback as the callback object is
/// continuously destroyed and restored by the mcgregor function.
static int mc_gregor_maximal_num_edges;

/**
 * \brief Each time a subgraph is found by the mcgregor algorithm, the
 * operator () of this object is called.
 */
struct SubgraphCallback {

  SubgraphCallback(const GraphType& graph1, const GraphType& graph2) :
      graph1_(graph1), graph2_(graph2) {}

  // leave this function as templated, as the passed arguments are of very
  // complex and nested type
  template<typename CorrespondenceMapFirstToSecond,
      typename CorrespondenceMapSecondToFirst>
  bool operator()(CorrespondenceMapFirstToSecond correspondence_map_1_to_2,
                  CorrespondenceMapSecondToFirst correspondence_map_2_to_1,
                  typename boost::graph_traits<GraphType>::vertices_size_type
                  subgraph_size) {

    std::ostringstream log_message;
    log_message << "Correspondences found by mcgregor:\n";

    // print out the vertex correspondences found by the mcgregor algorithm
    auto v_begin = boost::vertices(graph1_).first;
    auto v_end = boost::vertices(graph1_).second;
    for (; v_begin != v_end; ++v_begin) {
      auto v = *v_begin;
      // Skip unmapped vertices
      auto correspondence = boost::get(correspondence_map_1_to_2, v);
      if (correspondence != boost::graph_traits<GraphType>::null_vertex()) {
        log_message << "\tvertex: \t" << v << ", " << graph1_[v]
                    << " \t <-> \t "
                    << "vertex: \t" << correspondence << ", "
                    << graph2_[correspondence] << "\n";
      }
    }

    LOG(INFO) << log_message.str();

    // count the number of edges in the found subgraph used as a distance
    // measure
    int current_subgraph_edges = 0;
    for (auto v1_begin = boost::vertices(graph1_).first; v1_begin != v_end;
         ++v1_begin) {
      auto v1 = *v1_begin;
      auto correspondence1 = boost::get(correspondence_map_1_to_2, v1);
      if (correspondence1 != boost::graph_traits<GraphType>::null_vertex())
        for (auto v2_begin = v1_begin + 1; v2_begin != v_end; ++v2_begin) {
          auto v2 = *v2_begin;
          auto correspondence2 = boost::get(correspondence_map_1_to_2, v2);
          if (correspondence2
              != boost::graph_traits<GraphType>::null_vertex()) {
            if (boost::edge(v1, v2, graph1_).second) {
              ++current_subgraph_edges;
            }
          }
        }
    }

    mc_gregor_maximal_num_edges =
        std::max(mc_gregor_maximal_num_edges, current_subgraph_edges);

    // Return true as we don't want to terminate the search of MCS.
    return true;
  }

 private:
  const GraphType& graph1_;
  const GraphType& graph2_;

};

}

#endif //X_VIEW_TEST_MCGREGOR_MATCHING_H
