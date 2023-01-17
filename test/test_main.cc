#include <gtest/gtest.h>
#include <glog/logging.h>

#include "test_common.h"
#include <x_view_core/x_view_tools.h>
#include <x_view_core/datasets/abstract_dataset.h>

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {

  x_view::setupLogging(argv);
#if X_VIEW_USE_DOUBLE_PRECISION
  LOG(INFO) << "\n============== Running X-View Tests (DP) ===============\n";
#else
  LOG(INFO) << "\n============== Running X-View Tests (SP) ===============\n";
#endif


  testing::InitGoogleTest(&argc, argv);

  // Create dummy parameters used by the tests.
  x_view_test::createParameters();

  int test_succesfull = RUN_ALL_TESTS();

  x_view::finalizeLogging();

  return test_succesfull;
}
