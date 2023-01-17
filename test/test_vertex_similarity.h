#ifndef X_VIEW_TEST_VERTEX_SIMILARITY_H
#define X_VIEW_TEST_VERTEX_SIMILARITY_H

#include <x_view_core/matchers/graph_matcher/random_walker.h>
#include <x_view_core/matchers/graph_matcher/vertex_similarity.h>

using namespace x_view;

namespace x_view_test {

/**
 * \brief This class implements tests to verify the symmetry and integrity of
 * the VertexSimilarity between two vertex descriptors (random walks)
 */
class VertexSimilarityTest {

 public:

  VertexSimilarityTest()
      : vertices_left_(0),
        vertices_right_(0),
        expected_scores_(0),
        score_type_(VertexSimilarity::SCORE_TYPE::WEIGHTED) {
  }

  /// \brief Resets the internal parameters such that the calling instance
  /// can be reused for new tests.
  void clear() {
    vertices_left_.clear();
    vertices_right_.clear();
    expected_scores_.clear();
  }

  /// \brief Sets the score type to be tested.
  /// \param score_type Score type to be tested.
  void setScoreType(const VertexSimilarity::SCORE_TYPE score_type) {
    VertexSimilarity::setScoreType(score_type);
    score_type_ = score_type;
  }

  /**
   * \brief Adds a new pair of vertices whose similarity score must be
   * compared against the third argument.
   * \param walk_left WalkMap structure representing the first vertex to be
   * compared.
   * \param walk_right WalkMap structure representing the second vertex to be
   * compared.
   * \param expected_score Expected score resulting from the matching.
   */
  void addWalkMap(const RandomWalker::WalkMap& walk_left,
                  const RandomWalker::WalkMap& walk_right,
                  const real_t expected_score);

  /// \brief Runs all tests for the walk maps contained in vertices_left_ and
  /// vertices_right_ respectively.
  void run() const;

  /// \brief Tests if the similarity computed by using the similarity score
  /// defined by score_type_ is equivalent to the one contained in
  /// expected_scores_.
  void checkExpectedSimilarity() const;

  /// \brief Tests if the score computation between two WalkMaps 'a' and 'b'
  /// is the same as the one between 'b' and 'a'.
  void checkSymmetry() const;

  /**
   * \brief Utility function used to generate a WalkMap structure from a
   * vector of vectors representing a list of random walks.
   * \param random_walks Vector of vectors, such that each inner vector has
   * the same size and represents a random walk. Those random walks can be
   * identical to each other.
   * \return A MappedWalk object representing the random walks passed as
   * argument.
   * \note The MappedWalk's elements do not contain a valid reference to the
   * associated RandomWalk object.
   */
  static RandomWalker::WalkMap generateWalkMap(const std::vector<std::vector<int>>& random_walks);

 private:
  /// \brief List of WalkMaps: the i-th element of this vector is compared
  /// agains the i-th element of the vertices_right_ vector.
  std::vector<RandomWalker::WalkMap> vertices_left_;

  /// \brief List of WalkMaps: the i-th element of this vector is compared
  /// agains the i-th element of the vertices_left_ vector.
  std::vector<RandomWalker::WalkMap> vertices_right_;

  /// \brief Vector containing at position i the expected score between
  /// WalkMaps at position i in vertices_left_ and vertices_right respectively.
  std::vector<real_t> expected_scores_;

  /// \brief Similarity score type being used.
  VertexSimilarity::SCORE_TYPE score_type_;
};

/// \brief Runs the symmetry test over a series of randomly generated random
/// walks using different parameters.
void testScoreSymmetry();

/// \brief Runs the entire test (symmetry plus value) over a set of manually
/// designed random walks.
void testScoreValue();

}

#endif //X_VIEW_TEST_VERTEX_SIMILARITY_H
