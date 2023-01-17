#include "test_mcgregor_matching.h"

#include <chrono>

namespace x_view_test {

void AbstractMaximalSubgraphTest::run() const {
  LOG(INFO) << "Testing " << graph_name_;

  for (int i = 0; i < graph_database_.size(); ++i) {
    const int expected_distance = expected_distances_[i];
    auto start = std::chrono::high_resolution_clock::now();
    const int computed_distance = computeMCSDistance(i);
    CHECK_EQ(expected_distance, computed_distance)
      << "Failed test for graph <" << graph_name_
      << "> for database graph number " << i << ".";
    auto end = std::chrono::high_resolution_clock::now();
    if (computed_distance ==
        boost::num_edges(query_graph_) + boost::num_edges(graph_database_[i]))
      LOG(INFO) << "\tNo correspondences found";
    else {
      LOG(INFO) << "Elapsed time for graph " << i << ": "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                    end - start).count() << " milliseconds.";
    }
  }
}

int AbstractMaximalSubgraphTest::computeMCSDistance(int graph_index) const {
  // input sanity check
  if (graph_index < 0 || graph_index >= graph_database_.size())
    return 0;

  const GraphType& g = graph_database_[graph_index];

  // Since we don't want only to consider graph topology but also consider
  // node and edge properties when deciding if two graphs are similar or not,
  // we need to specify what is called 'equivalence predicate', which can be
  // thought as a function which, given two arguments, returns true if they
  // are equal, false otherwise.

  // Edges are equivalent only if their label is the same.
  auto e_property_map_a = boost::get(&EdgeData::label, query_graph_);
  auto e_property_map_b = boost::get(&EdgeData::label, g);

  // Vertices are equivalent only it their label is the same.
  auto v_property_map_a = boost::get(&VertexData::label, query_graph_);
  auto v_property_map_b = boost::get(&VertexData::label, g);


  // Predicate type required by mcgregor function
  auto equivalence_predicates =
      boost::vertices_equivalent(boost::make_property_map_equivalent
                                     (v_property_map_a, v_property_map_b))
          .edges_equivalent(boost::make_property_map_equivalent
                                (e_property_map_a, e_property_map_b));


  // Object called by mcgregor function when the MCS is found
  SubgraphCallback subgraph_callback(query_graph_, g);

  // Are we only interested in connected subgraphs or not?
  bool only_connected_subgraphs = false;

  // reset global edge counter, is filled up by the subgraph_callback object
  mc_gregor_maximal_num_edges = 0;

  // compute maximal common subgraph between the two graphs
  boost::mcgregor_common_subgraphs_maximum_unique(
      query_graph_, g, only_connected_subgraphs, subgraph_callback,
      equivalence_predicates);

  const int num_edges_in_subgraph = mc_gregor_maximal_num_edges;

  const int num_edges_a = static_cast<int>(boost::num_edges(query_graph_));
  const int num_edges_b = static_cast<int>(boost::num_edges(g));

  // Compute distance score given the number of edges in the MCS
  const int distance = num_edges_a + num_edges_b - 2 * num_edges_in_subgraph;

  return distance;
}

void SimpleGraphsTest::buildGraphDatabase() {
  // Build the query graph of the form   A - B - C
  query_graph_.clear();
  auto v0_0 = boost::add_vertex({VertexData::VertexLabelName::A}, query_graph_);
  auto v0_1 = boost::add_vertex({VertexData::VertexLabelName::B}, query_graph_);
  auto v0_2 = boost::add_vertex({VertexData::VertexLabelName::C}, query_graph_);

  boost::add_edge(v0_0, v0_1, {EdgeData::EdgeLabelName::CLOSE}, query_graph_);
  boost::add_edge(v0_1, v0_2, {EdgeData::EdgeLabelName::DISTANT}, query_graph_);

  // Build the graph database
  graph_database_.resize(2);
  expected_distances_.resize(2);

  // Build second graph of the form    A - C
  GraphType& g1 = graph_database_[0];
  expected_distances_[0] = 3;  // dist(q, g1) = 2 + 1 - (2 * 0)
  auto v1_0 = boost::add_vertex({VertexData::VertexLabelName::A}, g1);
  auto v1_1 = boost::add_vertex({VertexData::VertexLabelName::C}, g1);

  boost::add_edge(v1_0, v1_1, {EdgeData::EdgeLabelName::CLOSE}, g1);

  // Build third graph of the form    B - C - C - B - A - B
  // (matching of query graph here            <------->      )
  GraphType& g2 = graph_database_[1];
  expected_distances_[1] = 3; // dist(q, g2) = 2 + 5 - (2 * 2)
  auto v2_0 = boost::add_vertex({VertexData::VertexLabelName::B}, g2);
  auto v2_1 = boost::add_vertex({VertexData::VertexLabelName::C}, g2);
  auto v2_2 = boost::add_vertex({VertexData::VertexLabelName::C}, g2);
  auto v2_3 = boost::add_vertex({VertexData::VertexLabelName::B}, g2);
  auto v2_4 = boost::add_vertex({VertexData::VertexLabelName::A}, g2);
  auto v2_5 = boost::add_vertex({VertexData::VertexLabelName::B}, g2);

  boost::add_edge(v2_0, v2_1, {EdgeData::EdgeLabelName::CLOSE}, g2);
  boost::add_edge(v2_1, v2_2, {EdgeData::EdgeLabelName::VISIBLE}, g2);
  boost::add_edge(v2_2, v2_3, {EdgeData::EdgeLabelName::DISTANT}, g2);
  boost::add_edge(v2_3, v2_4, {EdgeData::EdgeLabelName::CLOSE}, g2);
  boost::add_edge(v2_4, v2_5, {EdgeData::EdgeLabelName::VISIBLE}, g2);
}

void PaperGraphsTest::buildGraphDatabase() {
  // Build up graphs as defined in Fig 3 of
  // https://openproceedings.org/2012/conf/edbt/ZhuQYC12.pdf
  // adding some edge constraints

  // build the query graph
  //
  //                              C
  //                            /   \
  //     C         B           B     C
  //    /  \       |           |     |
  //   /    \      |           |     |
  //  C ---- B --- A --- A --- C --- B

  query_graph_.clear();
  auto v0_0 = boost::add_vertex({VertexData::VertexLabelName::A}, query_graph_);
  auto v0_1 = boost::add_vertex({VertexData::VertexLabelName::C}, query_graph_);
  auto v0_2 = boost::add_vertex({VertexData::VertexLabelName::B}, query_graph_);
  auto v0_3 = boost::add_vertex({VertexData::VertexLabelName::A}, query_graph_);
  auto v0_4 = boost::add_vertex({VertexData::VertexLabelName::B}, query_graph_);
  auto v0_5 = boost::add_vertex({VertexData::VertexLabelName::A}, query_graph_);
  auto v0_6 = boost::add_vertex({VertexData::VertexLabelName::C}, query_graph_);
  auto v0_7 = boost::add_vertex({VertexData::VertexLabelName::B}, query_graph_);
  auto v0_8 = boost::add_vertex({VertexData::VertexLabelName::C}, query_graph_);
  auto v0_9 = boost::add_vertex({VertexData::VertexLabelName::C}, query_graph_);
  auto
      v0_10 = boost::add_vertex({VertexData::VertexLabelName::B}, query_graph_);

  boost::add_edge(v0_0, v0_1, {EdgeData::EdgeLabelName::CLOSE}, query_graph_);
  boost::add_edge(v0_1, v0_2, {EdgeData::EdgeLabelName::CLOSE}, query_graph_);
  boost::add_edge(v0_0, v0_2, {EdgeData::EdgeLabelName::DISTANT}, query_graph_);
  boost::add_edge(v0_2, v0_3, {EdgeData::EdgeLabelName::VISIBLE}, query_graph_);
  boost::add_edge(v0_3, v0_4, {EdgeData::EdgeLabelName::DISTANT}, query_graph_);
  boost::add_edge(v0_3, v0_5, {EdgeData::EdgeLabelName::CLOSE}, query_graph_);
  boost::add_edge(v0_5, v0_6, {EdgeData::EdgeLabelName::VISIBLE}, query_graph_);
  boost::add_edge(v0_6, v0_7, {EdgeData::EdgeLabelName::CLOSE}, query_graph_);
  boost::add_edge(v0_7, v0_8, {EdgeData::EdgeLabelName::VISIBLE}, query_graph_);
  boost::add_edge(v0_8, v0_9, {EdgeData::EdgeLabelName::CLOSE}, query_graph_);
  boost::add_edge(v0_9, v0_10, {EdgeData::EdgeLabelName::DISTANT},
                  query_graph_);
  boost::add_edge(v0_6, v0_10, {EdgeData::EdgeLabelName::DISTANT},
                  query_graph_);

  // build the graph database
  graph_database_.resize(2);
  expected_distances_.resize(2);

  // Build second graph (g1) which is a subgraph of q --> distance should be 6
  //
  //     C         B
  //    /  \       |
  //   /    \      |
  //  C ---- B --- A --- A
  GraphType& g1 = graph_database_[0];
  expected_distances_[0] = 6;   // dist(q, g1) = 12 + 6 - (2 * 6)
  auto v1_0 = boost::add_vertex({VertexData::VertexLabelName::A}, g1);
  auto v1_1 = boost::add_vertex({VertexData::VertexLabelName::C}, g1);
  auto v1_2 = boost::add_vertex({VertexData::VertexLabelName::B}, g1);
  auto v1_3 = boost::add_vertex({VertexData::VertexLabelName::A}, g1);
  auto v1_4 = boost::add_vertex({VertexData::VertexLabelName::B}, g1);
  auto v1_5 = boost::add_vertex({VertexData::VertexLabelName::A}, g1);

  boost::add_edge(v1_0, v1_1, {EdgeData::EdgeLabelName::CLOSE}, g1);
  boost::add_edge(v1_1, v1_2, {EdgeData::EdgeLabelName::CLOSE}, g1);
  boost::add_edge(v1_0, v1_2, {EdgeData::EdgeLabelName::DISTANT}, g1);
  boost::add_edge(v1_2, v1_3, {EdgeData::EdgeLabelName::VISIBLE}, g1);
  boost::add_edge(v1_3, v1_4, {EdgeData::EdgeLabelName::DISTANT}, g1);
  boost::add_edge(v1_3, v1_5, {EdgeData::EdgeLabelName::CLOSE}, g1);

  // Build third graph (g2) which has same structure as q, but a node label
  // is different --> distance should be 4
  //
  //                              C
  //                            /   \
  //     C         B           B     C
  //    /  \       |           |     |
  //   /    \      |           |     |
  //  C ---- B --- A --- C --- C --- B
  GraphType& g2 = graph_database_[1];
  expected_distances_[1] = 4; // dist(q, g2) = 12 + 12 - (2 * 10)
  auto v2_0 = boost::add_vertex({VertexData::VertexLabelName::A}, g2);
  auto v2_1 = boost::add_vertex({VertexData::VertexLabelName::C}, g2);
  auto v2_2 = boost::add_vertex({VertexData::VertexLabelName::B}, g2);
  auto v2_3 = boost::add_vertex({VertexData::VertexLabelName::A}, g2);
  auto v2_4 = boost::add_vertex({VertexData::VertexLabelName::B}, g2);
  auto v2_5 = boost::add_vertex({VertexData::VertexLabelName::C}, g2);
  auto v2_6 = boost::add_vertex({VertexData::VertexLabelName::C}, g2);
  auto v2_7 = boost::add_vertex({VertexData::VertexLabelName::B}, g2);
  auto v2_8 = boost::add_vertex({VertexData::VertexLabelName::C}, g2);
  auto v2_9 = boost::add_vertex({VertexData::VertexLabelName::C}, g2);
  auto v2_10 = boost::add_vertex({VertexData::VertexLabelName::B}, g2);

  boost::add_edge(v2_0, v2_1, {EdgeData::EdgeLabelName::CLOSE}, g2);
  boost::add_edge(v2_1, v2_2, {EdgeData::EdgeLabelName::CLOSE}, g2);
  boost::add_edge(v2_0, v2_2, {EdgeData::EdgeLabelName::DISTANT}, g2);
  boost::add_edge(v2_2, v2_3, {EdgeData::EdgeLabelName::VISIBLE}, g2);
  boost::add_edge(v2_3, v2_4, {EdgeData::EdgeLabelName::DISTANT}, g2);
  boost::add_edge(v2_3, v2_5, {EdgeData::EdgeLabelName::CLOSE}, g2);
  boost::add_edge(v2_5, v2_6, {EdgeData::EdgeLabelName::VISIBLE}, g2);
  boost::add_edge(v2_6, v2_7, {EdgeData::EdgeLabelName::CLOSE}, g2);
  boost::add_edge(v2_7, v2_8, {EdgeData::EdgeLabelName::VISIBLE}, g2);
  boost::add_edge(v2_8, v2_9, {EdgeData::EdgeLabelName::CLOSE}, g2);
  boost::add_edge(v2_9, v2_10, {EdgeData::EdgeLabelName::DISTANT}, g2);
  boost::add_edge(v2_6, v2_10, {EdgeData::EdgeLabelName::DISTANT}, g2);
}

}



