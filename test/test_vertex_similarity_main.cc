#include <gtest/gtest.h>

#include "test_vertex_similarity.h"

using namespace x_view_test;

TEST(XViewSlamTestSuite, test_vertex_similarity) {

  LOG(INFO) << "\n\n======Testing vertex similarity======";

  // Test the symmetry of the different scores provided by the
  // VertexSimilarity class.
  testScoreSymmetry();

  // Test the computed score value for manually designed cases.
  testScoreValue();

}



