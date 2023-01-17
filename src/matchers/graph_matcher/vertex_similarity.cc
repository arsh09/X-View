#include <x_view_core/matchers/graph_matcher/vertex_similarity.h>

namespace x_view {

// Set default score type
VertexSimilarity::SCORE_TYPE VertexSimilarity::current_score_type_ =
    VertexSimilarity::SCORE_TYPE::WEIGHTED;

VertexSimilarity::ScoreFunctionType VertexSimilarity::score =
    VertexSimilarity::getScoreFunction();

void VertexSimilarity::setScoreType(
    const VertexSimilarity::SCORE_TYPE score_type) {
  VertexSimilarity::current_score_type_ = score_type;
  VertexSimilarity::score = VertexSimilarity::getScoreFunction();
}

VertexSimilarity::ScoreFunctionType VertexSimilarity::getScoreFunction() {
  switch (VertexSimilarity::current_score_type_) {
    case VertexSimilarity::SCORE_TYPE::WEIGHTED: {
      return VertexSimilarity::score_weighted;
    }
    case VertexSimilarity::SCORE_TYPE::SURFACE: {
      return VertexSimilarity::score_surface;
    }
    default: {
      CHECK(false) << "Score type passed to " << __FUNCTION__
                   << " is unrecognized.";
    }
  }
}

const real_t VertexSimilarity::score_weighted(const RandomWalker::WalkMap& node1,
                                             const RandomWalker::WalkMap& node2) {
  int score = 0;
  int normalization = 0;
  for (const auto& walk1 : node1) {
    const int walk1_id = walk1.first;
    const int walk1_multiplicity = walk1.second.multiplicity;
    // Check if walk1_id is also present in the node2 WalkMap.
    const auto found_iter = node2.find(walk1_id);
    // walk1_id found in node2
    if (found_iter != node2.end()) {

      const int walk2_multiplicity = found_iter->second.multiplicity;

      const int min_multiplicity =
          std::min(walk1_multiplicity, walk2_multiplicity);

      score += min_multiplicity;
    }
    normalization += walk1_multiplicity;
  }
  return real_t(score) / normalization;
}

const real_t VertexSimilarity::score_surface(const RandomWalker::WalkMap& node1,
                                 const RandomWalker::WalkMap& node2) {

  int normalization = 0;
  for(const auto& walk1 : node1)
    normalization += walk1.second.multiplicity;
  for(const auto& walk2 : node2)
    normalization += walk2.second.multiplicity;

  int score = 0;
  for (const auto& walk1 : node1) {
    const int walk1_id = walk1.first;
    const int walk1_multiplicity = walk1.second.multiplicity;
    // Check if walk1_id is also present in the node2 WalkMap.
    const auto found_iter = node2.find(walk1_id);
    // walk1_id found in node2
    if (found_iter != node2.end()) {

      const int walk2_multiplicity = found_iter->second.multiplicity;

      score += walk1_multiplicity + walk2_multiplicity;
    }
  }
  return real_t(score) / normalization;

}

}
