#include <gtest/gtest.h>

#include "test_mcgregor_matching.h"

using namespace x_view_test;

TEST(XViewSlamTestSuite, test_mcgregor_matching) {

  // Only test mcgregor algorithm in release mode as in debut it takes too long.
#ifdef X_VIEW_DEBUG
  CHECK(true);
#else
  LOG(INFO) << "\n\n====Testing mcgregor graph matching====";

  // Run a bunch of tests for different graphs and compute matches with
  // mcgregor algorithm
  SimpleGraphsTest simple_graph_test;
  simple_graph_test.run();

  PaperGraphsTest paper_graphs_test;
  paper_graphs_test.run();
#endif
}



