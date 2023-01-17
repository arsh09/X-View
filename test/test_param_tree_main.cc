#include <gtest/gtest.h>

#include "test_param_tree.h"

using namespace x_view_test;

TEST(XViewSlamTestSuite, test_param_tree) {

  LOG(INFO) << "\n\n====Testing parameter tree====";

  testParameterValues();

  testParameterChildren();

}

