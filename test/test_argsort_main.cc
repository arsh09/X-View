#include <gtest/gtest.h>

#include "test_argsort.h"
#include <x_view_core/x_view_tools.h>

#include <glog/logging.h>

using namespace x_view_test;

TEST(XViewSlamTestSuite, test_argsort) {

  LOG(INFO) << "\n\n====Testing argsort====";

  testArgsortColums();

  testArgsortRows();

  testRandom();

  testRepeating();

}


