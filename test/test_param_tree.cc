#include "test_param_tree.h"

#include <x_view_core/parameters/parameters.h>

#include <iostream>
using namespace x_view;

namespace x_view_test {

void testParameterValues() {
  Parameters parameters;
  parameters.setInteger("IntegerValue1", 1);
  parameters.setInteger("IntegerValue42", 42);
  parameters.setInteger("IntegerValue-3", -3);

  const int integer_value_1 = parameters.getInteger("IntegerValue1");
  CHECK_EQ(integer_value_1, 1);
  const int integer_value_42 = parameters.getInteger("IntegerValue42");
  CHECK_EQ(integer_value_42, 42);
  const int integer_value_min3 = parameters.getInteger("IntegerValue-3");
  CHECK_EQ(integer_value_min3, -3);

  parameters.setFloat("FloatValue1", 1.0);
  parameters.setFloat("FloatValue42.5", 42.5);
  parameters.setFloat("FloatValue-3.14", -3.14);

  const real_t float_value_1 = parameters.getFloat("FloatValue1");
  CHECK_NEAR(float_value_1, 1.0, x_view::real_eps);
  const real_t float_value_42_5 = parameters.getFloat("FloatValue42.5");
  CHECK_NEAR(float_value_42_5, 42.5,x_view::real_eps);
  const real_t float_value_min3_14 = parameters.getFloat("FloatValue-3.14");
  CHECK_NEAR(float_value_min3_14, -3.14, x_view::real_eps);

  parameters.setBoolean("False", false);
  parameters.setBoolean("True", true);

  const bool false_ = parameters.getBoolean("False");
  CHECK(!false_);
  const bool true_ = parameters.getBoolean("True");
  CHECK(true_);

  parameters.setString("StringWhat", "What");
  parameters.setString("StringWhere", "Where");

  const std::string what = parameters.getString("StringWhat");
  CHECK(what == "What");
  const std::string where = parameters.getString("StringWhere");
  CHECK(where == "Where");
}

void testParameterChildren() {
  std::unique_ptr<Parameters> parent(new Parameters("Parent"));
  std::unique_ptr<Parameters> child1(new Parameters("Child1"));
  std::unique_ptr<Parameters> child2(new Parameters("Child2"));
  std::unique_ptr<Parameters> grandchild1_1(new Parameters("Child1_1"));
  std::unique_ptr<Parameters> grandchild1_2(new Parameters("Child1_2"));
  std::unique_ptr<Parameters> grandchild2_1(new Parameters("Child2_1"));
  std::unique_ptr<Parameters> grandchild2_2(new Parameters("Child2_2"));

  parent->setInteger("parentInt", 0);
  parent->addChildPropertyList("c1", std::move(child1));
  parent->addChildPropertyList("c2", std::move(child2));
  parent->getChildPropertyList("c1")->setInteger("childInt", 1);
  parent->getChildPropertyList("c2")->setInteger("childInt", 2);

  parent->getChildPropertyList("c1")
      ->addChildPropertyList("c1", std::move(grandchild1_1));
  parent->getChildPropertyList("c1")->getChildPropertyList("c1")->setInteger
      ("grandChildInt", 3);

  parent->getChildPropertyList("c1")
      ->addChildPropertyList("c2", std::move(grandchild1_2));
  parent->getChildPropertyList("c1")->getChildPropertyList("c2")->setInteger
      ("grandChildInt", 4);

  parent->getChildPropertyList("c2")
      ->addChildPropertyList("c1", std::move(grandchild2_1));
  parent->getChildPropertyList("c2")->getChildPropertyList("c1")->setInteger
      ("grandChildInt", 5);

  parent->getChildPropertyList("c2")
      ->addChildPropertyList("c2", std::move(grandchild2_2));
  parent->getChildPropertyList("c2")->getChildPropertyList("c2")->setInteger
      ("grandChildInt", 6);

  CHECK(parent->getInteger("parentInt", 10) == 0);
  CHECK(parent->getChildPropertyList("c1")->getInteger("parentInt", 10) == 10);
  CHECK(parent->getInteger("childInt", 10) == 10);

  auto& child1_ = parent->getChildPropertyList("c1");
  auto& child2_ = parent->getChildPropertyList("c2");

  CHECK(child1_->getInteger("childInt") == 1);
  CHECK(child2_->getInteger("childInt") == 2);

  CHECK(child1_->getChildPropertyList("c1")->getInteger("grandChildInt") == 3);
  CHECK(child1_->getChildPropertyList("c2")->getInteger("grandChildInt") == 4);
  CHECK(child2_->getChildPropertyList("c1")->getInteger("grandChildInt") == 5);
  CHECK(child2_->getChildPropertyList("c2")->getInteger("grandChildInt") == 6);
}

}
