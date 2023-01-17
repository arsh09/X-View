#include "test_vertex_similarity.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/x_view_locator.h>

namespace x_view_test {

void VertexSimilarityTest::addWalkMap(const RandomWalker::WalkMap& walk_left,
                                      const RandomWalker::WalkMap& walk_right,
                                      const real_t expected_score) {
  vertices_left_.push_back(walk_left);
  vertices_right_.push_back(walk_right);
  expected_scores_.push_back(expected_score);
}

void VertexSimilarityTest::run() const {
  checkSymmetry();
  checkExpectedSimilarity();
}

void VertexSimilarityTest::checkExpectedSimilarity() const {
  const uint64_t num_mapped_walks = vertices_left_.size();
  CHECK_EQ(vertices_right_.size(), num_mapped_walks);
  CHECK_EQ(expected_scores_.size(), num_mapped_walks);
  for(int i = 0; i < num_mapped_walks; ++i) {
    const RandomWalker::WalkMap& left = vertices_left_[i];
    const RandomWalker::WalkMap& right = vertices_right_[i];
    const real_t expected_score = expected_scores_[i];

    // Compute score.
    const real_t computed_score = VertexSimilarity::score(left, right);
    CHECK_LE(0.f, computed_score);
    CHECK_GE(1.f, computed_score);

    CHECK_DOUBLE_EQ(expected_score, computed_score);
  }
}

void VertexSimilarityTest::checkSymmetry() const {
  const uint64_t num_mapped_walks = vertices_left_.size();
  CHECK_EQ(vertices_right_.size(), num_mapped_walks);
  CHECK_EQ(expected_scores_.size(), num_mapped_walks);
  for(int i = 0; i < num_mapped_walks; ++i) {
    const RandomWalker::WalkMap& left = vertices_left_[i];
    const RandomWalker::WalkMap& right = vertices_right_[i];

    // Compute score.
    const real_t computed_score_1 = VertexSimilarity::score(left, right);
    const real_t computed_score_2 = VertexSimilarity::score(right, left);

    CHECK_LE(0.f, computed_score_1);
    CHECK_GE(1.f, computed_score_1);

    CHECK_LE(0.f, computed_score_2);
    CHECK_GE(1.f, computed_score_2);

    CHECK_DOUBLE_EQ(computed_score_1, computed_score_2);
  }
}

RandomWalker::WalkMap VertexSimilarityTest::generateWalkMap(
    const std::vector<std::vector<int>>& random_walks) {

  const auto& dataset = x_view::Locator::getDataset();

  //Check that the elements in the random_walks passed as argument are all
  // smaller than the number of semantic classes.
  for(const auto& random_walk : random_walks)
    for(const int val : random_walk)
      CHECK(val < dataset->numSemanticClasses() && val >= 0);

  auto computeKey = [&](const std::vector<int>& random_walk)-> int {
    const static int num_classes = dataset->numSemanticClasses();
    int id = 0;
    int mult = 1;
    for (const int val : random_walk) {
      id += mult * val;
      mult *= num_classes;
    }
    return id;
  };

  RandomWalker::WalkMap walk_map;
  for(const std::vector<int>& random_walk : random_walks) {
    const int key = computeKey(random_walk);
    // Check if the key was already in the walk_map.
    auto found = walk_map.find(key);
    if(found == walk_map.end()) {
      //Create a MappedWalk.
      RandomWalker::RandomWalk fake_random_walk;
      RandomWalker::MappedWalk mapped_walk(fake_random_walk);
      mapped_walk.multiplicity = 1;
      walk_map.insert({key, mapped_walk});
    } else {
      // Increase the multiplicity of the associated random walk.
      ++(found->second);
    }
  }

  return walk_map;
}

void testScoreSymmetry() {


  const uint64_t seed = 0;
  std::mt19937 rng(seed);

  // Test for a different number of semantic classes.
  std::vector<int> nums_semantic_classes = {2, 5, 13};
  // Test for a different number of random walks per node, i.e. the number of
  // random walks starting from each vertex.
  std::vector<int> nums_random_walks = {5, 10, 100, 500};
  // Test different score types.
  std::vector<VertexSimilarity::SCORE_TYPE> score_types = {
      VertexSimilarity::SCORE_TYPE::WEIGHTED,
      VertexSimilarity::SCORE_TYPE::SURFACE
  };
  // Check with different random walk lengths.
  std::vector<int> random_walk_lengths = {2, 3, 5};
  // Set the batch size to be checked.
  const int num_vertices = 50;

  for (int num_semantic_classes : nums_semantic_classes) {
    std::unique_ptr<AbstractDataset> dataset(
        new AbstractDataset(num_semantic_classes));
    Locator::registerDataset(std::move(dataset));
    std::uniform_int_distribution<int> dist(0, num_semantic_classes - 1);
    auto gen = std::bind(dist, rng);
    for (const int num_random_walks : nums_random_walks) {
      for (const auto score_type : score_types) {
        for (const int random_walk_length : random_walk_lengths) {
          VertexSimilarityTest vertex_similarity_test;
          vertex_similarity_test.setScoreType(score_type);
          for (int n = 0; n < num_vertices; ++n) {
            std::vector<std::vector<int>> walk_left(num_random_walks);
            std::vector<std::vector<int>> walk_right(num_random_walks);
            for (int i = 0; i < num_random_walks; ++i) {
              auto& left = walk_left[i];
              auto& right = walk_right[i];
              left.resize(random_walk_length);
              right.resize(random_walk_length);
              std::generate(left.begin(), left.end(), gen);
              std::generate(right.begin(), right.end(), gen);
            }
            const auto walk_map_left =
                VertexSimilarityTest::generateWalkMap(walk_left);
            const auto walk_map_right =
                VertexSimilarityTest::generateWalkMap(walk_right);
            vertex_similarity_test.addWalkMap(walk_map_left,
                                              walk_map_right,
                                              0.f);
          }
          // Only check symmetry property.
          vertex_similarity_test.checkSymmetry();
        }
      }
    }
  }
}

void testScoreValue() {
  const int num_semantic_classes = 10;
  std::unique_ptr<AbstractDataset> dataset(
      new AbstractDataset(num_semantic_classes));
  Locator::registerDataset(std::move(dataset));

  std::vector<std::vector<int>> left = {
      {0,0,0},
      {0,0,0},
      {1,2,3},
      {2,1,3},
      {4,3,1}
  };
  std::vector<std::vector<int>> right = {
      {0,0,0},
      {5,5,5},
      {0,0,0},
      {2,1,3},
      {2,1,3}
  };

  auto left_map = VertexSimilarityTest::generateWalkMap(left);
  auto right_map = VertexSimilarityTest::generateWalkMap(right);

  VertexSimilarityTest vertex_similarity_test;

  vertex_similarity_test.setScoreType(VertexSimilarity::SCORE_TYPE::WEIGHTED);
  const real_t expected_score_weighted = 3.0/5.0;
  vertex_similarity_test.addWalkMap(left_map, right_map,
                                    expected_score_weighted);
  vertex_similarity_test.run();
  vertex_similarity_test.clear();


  vertex_similarity_test.setScoreType(VertexSimilarity::SCORE_TYPE::SURFACE);
  const real_t expected_score_surface = 7.0/10.0;
  vertex_similarity_test.addWalkMap(left_map, right_map,
                                    expected_score_surface);
  vertex_similarity_test.run();
  vertex_similarity_test.clear();

  // Identical matches should have score = 1.
  vertex_similarity_test.addWalkMap(left_map, left_map, 1.f);
  vertex_similarity_test.addWalkMap(right_map, right_map, 1.f);
  vertex_similarity_test.setScoreType(VertexSimilarity::SCORE_TYPE::WEIGHTED);
  vertex_similarity_test.run();
  vertex_similarity_test.setScoreType(VertexSimilarity::SCORE_TYPE::SURFACE);
  vertex_similarity_test.run();

  vertex_similarity_test.clear();
}

}

