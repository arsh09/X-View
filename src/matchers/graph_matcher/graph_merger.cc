#include <x_view_core/matchers/graph_matcher/graph_merger.h>

#include <x_view_core/matchers/graph_matcher.h>
#include <x_view_core/x_view_tools.h>
#include <x_view_core/x_view_locator.h>

#include <boost/graph/connected_components.hpp>

#include <iterator>

namespace x_view {

GraphMerger::GraphMerger(const Graph& database_graph,
                         const Graph& query_graph,
                         const GraphMatcher::GraphMatchingResult& matching_result,
                         const GraphMergerParameters& graph_merger_parameters)
    : database_graph_(database_graph),
      query_graph_(query_graph),
    // Initialize merged graph as a copy of the database graph.
      merged_graph_(database_graph),
      matching_result_(matching_result),
      graph_merger_parameters_(graph_merger_parameters) {

  LOG(INFO) << "Using graph merger with following parameters:"
            << "\n\tTime window:          "
            << graph_merger_parameters_.time_window
            << "\n\tSimilarity threshold: "
            << graph_merger_parameters_.similarity_threshold
            << "\n\tDistance threshold:   "
            << graph_merger_parameters_.distance_threshold << ".";

  // Define start vertex index of vertices being added to the merged graph.
  current_vertex_index_ = 0;
  const auto db_vertices = boost::vertices(database_graph_);
  for (auto iter = db_vertices.first; iter != db_vertices.second; ++iter)
    current_vertex_index_ =
        std::max(current_vertex_index_, database_graph_[*iter].index);
  ++current_vertex_index_;
}

const Graph GraphMerger::computeMergedGraph() {

  const auto& timer = Locator::getTimer();

  const uint64_t num_query_vertices = boost::num_vertices(query_graph_);
  const uint64_t num_db_vertices = boost::num_vertices(database_graph_);

  const GraphMatcher::SimilarityMatrixType& similarity_matrix =
      matching_result_.getSimilarityMatrix();

  // This map contains the matches between the vertex descriptors in the
  // query graph to the corresponding vertex descriptor in the database_graph.
  query_in_db_.clear();

  timer->registerTimer("VertexMatching", "GraphGrowing");
  timer->start("VertexMatching");
  // Loop over the vertices of the query graph.
  for (uint64_t j = 0; j < num_query_vertices; ++j) {
    // Loop over the vertices of the database graph.
    for (uint64_t i = 0; i < num_db_vertices; ++i) {
      // Check if the two vertices are possible matches.
      if (similarity_matrix(i, j)
          >= graph_merger_parameters_.similarity_threshold &&
          temporalDistance(i, j) <= graph_merger_parameters_.time_window &&
          spatialDistance(i, j)
              <= graph_merger_parameters_.distance_threshold) {
        // There is a match between the j-th vertex of the query graph and
        // the i-th vertex of the database graph.
        query_in_db_.insert({j, i});
        matched_vertices_.push_back(j);
        break;
      }
    }
  }
  timer->stop("VertexMatching");

  if (matched_vertices_.size() == 0) {
    LOG(WARNING) << "The query graph was unmatched to the database graph. "
        "Creating connections between query and database graph based only on "
        "geometric information.";
    linkUnmatchedQueryGraph();
    return merged_graph_;
  }

  // Push all matched vertices into the queue of 'still to process' vertices.
  still_to_process_ = DescriptorQueue();
  for (const VertexDescriptor v_d : matched_vertices_) {
    still_to_process_.push(v_d);
  }

  // Iterate over the still_to_process vertices and add them to the merged
  // graph. Also iterate over their neighbors and, in case they have not been
  // processed yet add them to the still_to_process queue.
  timer->registerTimer("VertexMerging", "GraphGrowing");
  timer->start("VertexMerging");
  while (!still_to_process_.empty()) {
    const VertexDescriptor source_in_query_graph = still_to_process_.front();
    // Remove the element from the queue.
    still_to_process_.pop();
    LOG(INFO) << "Analyzing " << source_in_query_graph
              << "-th vertex of query graph (index: "
              << query_graph_[source_in_query_graph].index << ").";
    addVertexToMergedGraph(source_in_query_graph);
  }
  timer->stop("VertexMerging");

  return merged_graph_;
}


void GraphMerger::mergeDuplicates(const real_t merge_distance, Graph* graph) {

  LOG(INFO) << "Merging all vertices with same semantic label whose euclidean "
            << "distance is smaller than " << merge_distance << ".";

  // Utility class used to store a possible merge between vertices.
  class CandidateMerge {
   public:
    CandidateMerge()
        : anchor_(0), mergeable_(0) {
    }

    CandidateMerge(const VertexDescriptor anchor,
                   const VertexDescriptor mergeable)
        : anchor_(anchor), mergeable_(mergeable) {
      CHECK(anchor < mergeable) << "The anchor vertex descriptor must "
          "have smaller index value than the mergeable!";
    }

    const VertexDescriptor anchor() const { return anchor_; }
    const VertexDescriptor mergeable() const { return mergeable_; }

   private:

    /// \brief Vertex descriptor associated to the 'anchor' vertex, i.e. the
    /// vertex which will absorbe the mergeable_ vertex.
    const VertexDescriptor anchor_;

    /// \brief Vertex descriptor associated to the 'meargeble' vertex, i.e.
    /// the vertex which will be merged into the anchor_ vertex.
    const VertexDescriptor mergeable_;
  };

  // Vector filled up with all possible merges.
  std::vector<CandidateMerge> candidates;

  const uint64_t num_vertices = boost::num_vertices(*graph);

  // This function does not support merges of the type 1-2, 2-3, since after
  // merging 1 with 2, vertex 2 does not exist anymore. For this reason we
  // need to keep track which vertex has already been assigned for a merge.
  std::vector<bool> taken(num_vertices, false);

  LOG(INFO) << "Computing the number of vertex pairs to be merged together.";
  // Traverse the vertices in backward order.
  for (uint64_t i_back = 0; i_back < num_vertices; ++i_back) {
    const uint64_t i = num_vertices - i_back - 1;
    // If the current vertex already wants to be merged to an other, then
    // skip it now.
    if (taken[i] == true)
      continue;

    // Iterate over the remaining elements and create a candidate merge.
    for (uint64_t j_back = 0; j_back < i && taken[i] == false; ++j_back) {
      const uint64_t j = i - j_back - 1;
      // If the current vertex already wants to be merged to an other, then
      // skip it now.
      if (taken[j] == true)
        continue;

      if (GraphMerger::verticesShouldBeMerged(i, j, *graph, merge_distance)) {
        // There is a merge between v_p_i and v_p_j.
        candidates.push_back({j, i});
        taken[i] = taken[j] = true;
      }
    }
  }
  LOG(INFO) << "There are " << candidates.size() << " merging pairs "
            << "for a graph with " << num_vertices << " vertices in total.";

  // Traverse the candidate merges and merge them
  for (const CandidateMerge& candidate : candidates) {

    const uint64_t anchor = candidate.anchor();
    const uint64_t mergeable = candidate.mergeable();

    // Get a list of all edges going into the mergeable vertex.
    const auto adjacent_vertices =
        boost::adjacent_vertices(mergeable, *graph);

    // Attach the adjacent vertices of mergeable to the anchor.
    for (auto adjacent_vertex = adjacent_vertices.first; adjacent_vertex !=
        adjacent_vertices.second; ++adjacent_vertex) {
      LOG(INFO) << "Adding edge between anchor vertex " << anchor
                << " and neighbor vertex " << *adjacent_vertex
                << " of meargeble vertex " << mergeable << ".";

      // Avoid self-edges (case that happens if anchor was a neighbor of
      // mergeable).
      if (anchor != *adjacent_vertex)
        addEdgeBetweenVertices(anchor, *adjacent_vertex, graph);
    }

    LOG(INFO) << "Updating timestamp of anchor vertex from "
              << (*graph)[anchor].last_time_seen_ << " to "
              << (*graph)[mergeable].last_time_seen_ << ".";
    // Transfer timestamp information to anchor.
    (*graph)[anchor].last_time_seen_ =
        (*graph)[mergeable].last_time_seen_;

    LOG(INFO) << "Removing mergeable vertex from graph structure.";
    // Clear the old mergeable vertex.
    boost::clear_vertex(mergeable, *graph);
    boost::remove_vertex(mergeable, *graph);

  }
}

void GraphMerger::linkCloseVertices(const real_t max_link_distance, Graph* graph) {
  const uint64_t num_vertices = boost::num_vertices(*graph);
  const real_t max_link_distance_squared =
      max_link_distance * max_link_distance;

  for(uint64_t i = 0; i < num_vertices; ++i) {
    const VertexProperty& v_p_i = (*graph)[i];
    for(uint64_t j = i + 1; j < num_vertices; ++j) {
      const VertexProperty& v_p_j = (*graph)[j];
      if(distSquared(v_p_i, v_p_j) < max_link_distance_squared)
        boost::add_edge(i, j, {i, j, 1}, *graph);
    }
  }
}

const bool GraphMerger::verticesShouldBeMerged(const VertexDescriptor v_d_1,
                                               const VertexDescriptor v_d_2,
                                               const Graph& graph,
                                               const real_t merge_distance) {

  // Query the vertex properties associated to the vertex descriptors passed
  // as argument.
  const VertexProperty& v_p_1 = graph[v_d_1];
  const VertexProperty& v_p_2 = graph[v_d_2];

  // Label consistency: if the semantic label associated to the vertices is
  // different, then the two vertices cannot be merged.
  if (v_p_1.semantic_label != v_p_2.semantic_label)
    return false;

  // Spatial consistency: only merge vertices if their Euclidean distance is
  // smaller than the merge_distance parameter passed as argument.
  if (distSquared(v_p_1, v_p_2) > merge_distance * merge_distance)
    return false;

  // Since all tests are fulfilled, the two vertices should be merged.
  return true;

}

void GraphMerger::addVertexToMergedGraph(const VertexDescriptor& source_in_query_graph) {

  // Get correspondent vertex in database graph.
  const VertexDescriptor source_in_db_graph =
      query_in_db_[source_in_query_graph];
  // Get the associated VertexProperty. Need to get it by value, as when
  // adding new vertices to the merged graph the internal representation of
  // the graph sometimes changes, changing also the values which would be
  // referred if the property was taken by reference.
  VertexProperty source_v_p = merged_graph_[source_in_db_graph];
  LOG(INFO) << "\tCorresponds to " << source_in_db_graph
            << "-th vertex in database graph:" << source_v_p << ".";

  const uint64_t new_time_stamp =
      query_graph_[source_in_query_graph].last_time_seen_;
  const uint64_t old_time_stamp =
      source_v_p.last_time_seen_;

  LOG(INFO) << "\tSetting last_time_seen_ property of merged vertex from "
            << old_time_stamp << " (db) to " << new_time_stamp << " (query).";

  source_v_p.last_time_seen_ = new_time_stamp;

  // Add observers of vertex to database graph.
  for (size_t i = 0u; i < query_graph_[source_in_query_graph].observers.size();
      ++i) {
    source_v_p.observers.push_back(
        query_graph_[source_in_query_graph].observers[i]);
  }

  LOG(INFO) << "\tIterating over its "
            << boost::degree(source_in_query_graph, query_graph_)
            << " neighbors in query graph.";

  // Get the list of direct neighbors of the source_in_query_graph.
  const auto neighbors_in_query_graph =
      boost::adjacent_vertices(source_in_query_graph, query_graph_);
  // Loop over the neighbors of query_graph and add them to db_graph if they
  // are not matched.
  int num_neighbor = 0;
  for (auto neighbor_in_query_graph = neighbors_in_query_graph.first;
       neighbor_in_query_graph != neighbors_in_query_graph.second;
       ++neighbor_in_query_graph) {
    LOG(INFO) << "\t\t" << num_neighbor++ << "-th neighbor: "
              << query_graph_[*neighbor_in_query_graph] << ":";

    // Check if this neighbor has a corresponding match in the database graph
    // or if it is a "new" observation.
    const auto neighbor_pos =
        std::find(matched_vertices_.begin(), matched_vertices_.end(),
                  *neighbor_in_query_graph);
    if (neighbor_pos == matched_vertices_.end()) {
      LOG(INFO) << "\t\tis unmatched (newly observed semantic entity), "
                << "so it is attached to the merged graph by linking it to the "
                << "'parent' matched vertex  " << source_v_p << ".";

      // The neighbor_in_query_graph was unmatched, so we add it to the
      // merged graph as a new vertex.
      VertexProperty neighbor_v_p = query_graph_[*neighbor_in_query_graph];
      // Set the new index of the newly added vertex.
      neighbor_v_p.index = current_vertex_index_++;
      const VertexDescriptor neighbor_in_db_graph =
          boost::add_vertex(neighbor_v_p, merged_graph_);
      LOG(INFO) << "\t\tWas added to merged graph: " << neighbor_v_p
                << " graph with VD: " << neighbor_in_db_graph << ".";
      // Create an edge between source_in_db_graph and the newly added vertex.
      // Since this edge links an already known vertex to a new entity, we
      // set its 'num_times_seen' property to 1.
      const uint64_t num_times_seen = 1;
      auto edge_d = boost::add_edge(source_in_db_graph, neighbor_in_db_graph,
                                    {source_v_p.index, neighbor_v_p.index,
                                      num_times_seen}, merged_graph_);

      LOG(INFO) << "\t\tsource index: " << source_v_p.index << ", target "
          "index: " << neighbor_v_p.index << ".";
      LOG(INFO) << "\t\tadded corresponding edge "
                << merged_graph_[edge_d.first] << ".";

      CHECK(edge_d.second == true)
      << "Added a vertex associated to a new semantic observation to the "
          "merged graph, but an edge between it and its parent vertex was "
          "already existing. This should not happen";
      // Set the neighbor as matched as it is now fully inserted into the
      // merged graph.
      matched_vertices_.push_back(*neighbor_in_query_graph);
      query_in_db_.insert({*neighbor_in_query_graph, neighbor_in_db_graph});
      still_to_process_.push(*neighbor_in_query_graph);
    } else {
      LOG(INFO) << "\t\tis already matched, checking if an edge already "
          << "exists in the merged graph or not.";
      // The neighbor was already a matched vertex. We need to check if an
      // edge exists in the merged graph or not.
      const VertexDescriptor neighbor_in_db_graph =
          query_in_db_[*neighbor_in_query_graph];

      const auto edge_in_merged_graph =
          boost::edge(source_in_db_graph, neighbor_in_db_graph, merged_graph_);
      if(edge_in_merged_graph.second == true) {
        // An edge between the two vertices was already present, so we need
        // to increase the 'num_times_seen' property of the edge by one.
        // Note, we need to do it only once, and avoid doing it twice, i.e.
        // once for vertex. Thus we introduce this check:
        if(source_in_db_graph < neighbor_in_db_graph) {
          EdgeProperty& e_p = merged_graph_[edge_in_merged_graph.first];
          LOG(INFO) << "\t\tAn edge was already present in the merged graph. "
                    << "Increasing num_times_seen from " << e_p.num_times_seen
                    << " to " << e_p.num_times_seen + 1 << ".";
          ++e_p.num_times_seen;
        } else {
          LOG(INFO) << "\t\tAn edge was already present in the merged graph. "
                    << "num_times_seen is not updated now, but was/will be "
                    << "updated when seen in the opposite direction.";
        }
      } else {
        // There is no edge between the two vertices, so let's add a new one.
        const VertexProperty neighbor_v_p = merged_graph_[neighbor_in_db_graph];
        const uint64_t num_times_seen = 1;
        boost::add_edge(source_in_db_graph, neighbor_in_db_graph,
                        {source_v_p.index, neighbor_v_p.index, num_times_seen},
                        merged_graph_);
        LOG(INFO) << "\t\tCreated a new edge in the merged graph even though "
            "the two vertices where already merged.";
      }
    }
  }
  // Since the source vertex property might have be modified, we need to set
  // it again with the updated values.
  merged_graph_[source_in_db_graph] = source_v_p;
}

void GraphMerger::linkUnmatchedQueryGraph() {

  // Add the vertices of the query graph to the database graph in the
  // merged_graph_ and update their index.
  const auto query_vertices = boost::vertices(query_graph_);
  for(auto iter = query_vertices.first; iter != query_vertices.second; ++iter) {
    const VertexDescriptor vertex_in_query_graph = *iter;
    VertexProperty v_p = query_graph_[*iter];
    v_p.index = current_vertex_index_++;
    const VertexDescriptor vertex_in_merged_graph =
        boost::add_vertex(v_p, merged_graph_);
    query_in_db_.insert({vertex_in_query_graph, vertex_in_merged_graph});
  }

  // Add edges belonging to the query graph inside the merged graph.
  const auto query_edges = boost::edges(query_graph_);
  for(auto iter = query_edges.first; iter != query_edges.second; ++iter) {
    const VertexDescriptor from_in_query = boost::source(*iter, query_graph_);
    const VertexDescriptor to_in_query = boost::target(*iter, query_graph_);

    const VertexDescriptor from_in_merged = query_in_db_[from_in_query];
    const VertexDescriptor to_in_merged = query_in_db_[to_in_query];

    const uint64_t from_index = merged_graph_[from_in_merged].index;
    const uint64_t to_index = merged_graph_[to_in_merged].index;

    // Since this edge comes from the query graph, we set its num_times_seen
    // property to 1.
    const uint64_t num_times_seen = 1;
    boost::add_edge(from_in_merged, to_in_merged,
                    {from_index, to_index, num_times_seen},  merged_graph_);
  }

  // The merged_graph presents two disconnected components.
  std::vector<int> components(boost::num_vertices(merged_graph_));
  int num_components =
      boost::connected_components(merged_graph_, &components[0]);

  CHECK(num_components > 1) << "The merged graph should have at least two "
      "disconnected components now but has " << num_components << " "
      "components.";

  // Merte togheter all vertices whose euclidean distance is smaller than a
  // threshold passed as parameter.
  const auto& parameters = Locator::getParameters();
  const auto& matching_parameters = parameters->getChildPropertyList("matcher");
  const real_t merge_distance =
      matching_parameters->getFloat("merge_distance", 1.0f);

  mergeDuplicates(merge_distance, &merged_graph_);

  // Create new edges between close vertices in 3D space.
  const real_t max_link_distance =
      matching_parameters->getFloat("max_link_distance");
  linkCloseVertices(max_link_distance, &merged_graph_);

  // Check for disconnected components.
  components.resize(boost::num_vertices(merged_graph_));
  num_components = boost::connected_components(merged_graph_, &components[0]);

  // Merging and linking close vertices was enough as we only have a single
  // connected component now.
  if(num_components == 1)
    return;

  // Merging and linking close vertices was not enough, thus we manually link
  // the disconnected components with a single edge between the closest
  // vertices belonging to different disconnected components.
  std::vector<int> unique_components(components);
  std::sort(unique_components.begin(), unique_components.end());
  unique_components.erase(std::unique(unique_components.begin(),
                                      unique_components.end()),
                          unique_components.end());

  const uint64_t num_vertices = boost::num_vertices(merged_graph_);

  // Iterate over each pair of components and determine the closest pair of
  // vertices to be connected.
  real_t min_distance_square = std::numeric_limits<real_t>::max();
  EdgeProperty closest_v_d_pair = {0, 0};
  for (auto first_component = unique_components.begin();
       first_component != unique_components.end(); ++first_component) {
    const int first_component_id = *first_component;
    for (auto second_component = std::next(first_component);
         second_component != unique_components.end(); ++second_component) {
      const int second_component_id = *second_component;
      for (uint64_t i = 0; i < num_vertices; ++i) {
        if (components[i] == first_component_id) {
          const VertexProperty& v_p_i = merged_graph_[i];
          for (uint64_t j = 0; j < num_vertices; ++j) {
            if (components[j] == second_component_id) {
              const VertexProperty& v_p_j = merged_graph_[j];
              real_t dist2 = distSquared(v_p_i, v_p_j);
              if (dist2 < min_distance_square) {
                min_distance_square = dist2;
                closest_v_d_pair.from = i;
                closest_v_d_pair.to = j;
                closest_v_d_pair.num_times_seen = 1;
              }
            }
          }
        }
      }
    }
  }

  CHECK(closest_v_d_pair.from != 0 || closest_v_d_pair.to != 0)
  << "Function " << __FUNCTION__ << " could not determine which "
  << "vertices are the closest pair in the disconnected graph "
  << "between component.";
  // The closest vertices between first_component and second_component
  // are the ones contained in closest_v_d_pair.
  boost::add_edge(closest_v_d_pair.from, closest_v_d_pair.to,
                  closest_v_d_pair, merged_graph_);

  CHECK(boost::connected_components(merged_graph_, &components[0]) == 1)
        << "It should now be a single component!";

  return;
}

const uint64_t GraphMerger::temporalDistance(const uint64_t i,
                                             const uint64_t j) const {
  const VertexProperty& v_i_database = database_graph_[i];
  const VertexProperty& v_j_query = query_graph_[j];

  return v_j_query.last_time_seen_ - v_i_database.last_time_seen_;
}

const real_t GraphMerger::spatialDistance(const uint64_t i, const uint64_t j)
const {
  const VertexProperty& v_i_database = database_graph_[i];
  const VertexProperty& v_j_query = query_graph_[j];

  return dist(v_j_query, v_i_database);
}

}
