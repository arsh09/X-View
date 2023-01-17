#ifndef X_VIEW_VERTEX_SIMILARITY_H
#define X_VIEW_VERTEX_SIMILARITY_H

#include <x_view_core/matchers/graph_matcher/random_walker.h>
#include <x_view_core/x_view_types.h>

namespace x_view {

/**
 * \brief Class providing implementation for the computation of vertex
 * similarities based on the random walks defined over each vertex.
 */
class VertexSimilarity {

 public:

  /**
   * \brief Enums specifying which score type to use when comparing two graph
   * vertices.
   */
  enum class SCORE_TYPE {
    /// \brief When the score function is WEIGHTED, then the score_weighted
    /// function is used to compute the score between graph vertices.
        WEIGHTED = 0,

        SURFACE
  };

  /**
   * \brief The score functions must all implement the following signature.
   * This means the score function must accept two WalkMap references as
   * arguments and compute a floating point score value which is returned.
   */
  typedef std::function< const real_t(
  const RandomWalker::WalkMap&,  const RandomWalker::WalkMap&)>
  ScoreFunctionType;

  /**
   * \brief This function sets the score function according to the parameter
   * passed as argument.
   * \param score_type Identifier associated to the score function to be used.
   */
  static void setScoreType(const SCORE_TYPE score_type);

  /**
   * \brief Used to get the current score function.
   * \return The current score function corresponding to the
   * current_score_type_ member of this class.
   */
  static ScoreFunctionType getScoreFunction();

  /**
   * \brief Function exposed to the used to compute the score between two
   * graph vertices.
   */
  static ScoreFunctionType score;

 private:
  /**
   * \brief Current score function type.
   */
  static SCORE_TYPE current_score_type_;

  /**
   * \brief Function implementing the ScoreFunctionType signature which
   * computes the similarity between two graph vertices by counting how many
   * exact correspondences exist between the random walks associated to the
   * graph vertices passed as argument. This function the similarity between
   * two WalkMaps considering the associated multiplicity.
   * \details Whenever a match is found between the two WalkMaps passed as
   * argument, this function considers the respective multiplicity, and
   * increases the score by the minimum multiplicity of the two matches:
   *   _______       _______
   *  | - - - | --> | - - - |
   *  | - - - | --> | - - - |
   *  | x x x | --> | x x x |     Total number of matches: 4
   *  | o o o |     | x x x |     Total number of possible matches: 6
   *  | o o o |     | \ \ \ |     ==> Similarity score: 4/6
   *  | \ \ \ | --> | \ \ \ |
   *   ~~~~~~~       ~~~~~~~
   *
   * In the example depicted above, the computation of the similarity score
   * follows these steps:
   *   1) random walk [---] has multiplicity 2 in left walkmap. It has also
   *      multiplicity 2 in right walkmap, thus increase score by 2/6, where 6
   *      is the total multiplicity of left walkmap.
   *   2) random walk [xxx] has multiplicity 1 in left walkmap. It has
   *      multiplicity 2 in right walkmap, thus only one random walk is matched.
   *      The score increases by 1/6, where 6 is the total multiplicity of left
   *      walkmap.
   *   3) random walk [ooo] has multiplicity 2 in left walkmap, but does not
   *      appear in right walkmap. There is no increase in score.
   *   4) random walk [\\\] has multiplicity 1 in left walkmap. It has
   *      multiplicity 2 in right walkmap, thus only one random walk is matched.
   *      The score increases by 1/6, where 6 is the total multiplicity of
   *      left walkmap.
   *
   * The above score computation results in a total score of (2+1+1)/6 = 4/6.
   *
   * \param node1 Walk map associated to the first node to be compared.
   * \param node2 Walk map associated to the second node to be compared.
   * \return A floating point score in [0,1] representing the similarity
   * between node1 and node2. The higher the most similar.
   * \code{.cpp}
   * RandomWalker random_walker(graph, random_walker_params);
   * const auto& mapped_walks = random_walker.getMappedWalks();
   * real_t similarity_i_j =
   *    VertexSimilarity::score(mapped_walks[i], mapped_walks[j]);
   * \endcode
   */
  static const real_t score_weighted(const RandomWalker::WalkMap& node1,
                                    const RandomWalker::WalkMap& node2);

  /**
   * \brief Function implementing the ScoreFunctionType signature which
   * computes the similarity between two graph vertices by counting how many
   * exact correspondences exist between the random walks associated to the
   * graph vertices passed as argument. This function the similarity between
   * two WalkMaps considering the associated multiplicity.
   * \details Whenever a match is found between the two WalkMaps passed as
   * argument, this function considers the respective multiplicity, and
   * increases the score by the summed multiplicity of the two matches:
   *   _______         _______
   *  | - - - | --->  | - - - |
   *  | - - - | --->  | - - - |
   *  | x x x | --->  | x x x |     Total number of matches (surface): 10
   *  | o o o |  \->  | x x x |     Total surface: 12
   *  | o o o |  /->  | \ \ \ |     ==> Similarity score: 10/12
   *  | \ \ \ | --->  | \ \ \ |
   *   ~~~~~~~         ~~~~~~~
   * In the example depicted above, the computation of the similarity score
   * follows these steps:
   *   1) random walk [---] has multiplicity 2 in left walkmap. It has also
   *      multiplicity 2 in right walkmap, thus increase score by 4/12, where
   *      12 is the total multiplicity of both walkmaps, and 4 is the number
   *      of random walks of that type present in the walkmaps combined.
   *   2) random walk [xxx] has multiplicity 1 in left walkmap. It has
   *      multiplicity 2 in right walkmap, thus increase score by 3/12, where
   *      12 is the total multiplicity of both walkmaps, and 3 is the number
   *      of random walks of that type present in the walkmaps combined.
   *   3) random walk [ooo] has multiplicity 2 in left walkmap, but does not
   *      appear in right walkmap. There is no increase in score since there
   *      is no match.
   *   4) random walk [\\\] has multiplicity 1 in left walkmap. It has
   *      multiplicity 2 in right walkmap, thus increase score by 3/12, where
   *      12 is the total multiplicity of both walkmaps, and 3 is the number
   *      of random walks of that type present in the walkmaps combined.
   *
   * The above score computation results in a total score of (4+3+3)/12 = 10/12.
   *
   * \param node1 Walk map associated to the first node to be compared.
   * \param node2 Walk map associated to the second node to be compared.
   * \return A floating point score in [0, 1] representing the similarity
   * between node1 and node2. The higher the most similar.
   * \code{.cpp}
   * RandomWalker random_walker(graph, random_walker_params);
   * const auto& mapped_walks = random_walker.getMappedWalks();
   * real_t similarity_i_j =
   *    VertexSimilarity::score(mapped_walks[i], mapped_walks[j]);
   * \endcode
   */
  static const real_t score_surface(const RandomWalker::WalkMap& node1,
                                    const RandomWalker::WalkMap& node2);

};

}

#endif //X_VIEW_VERTEX_SIMILARITY_H
