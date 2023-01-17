#include <gtest/gtest.h>
#include "test_timer.h"

#include <x_view_core/timer/null_timer.h>
#include <x_view_core/timer/timer.h>

#include <glog/logging.h>

#include <chrono>

using namespace x_view;
using namespace x_view_test;


TEST(XViewSlamTestSuite, test_timer) {

  LOG(INFO) << "\n\n====Testing timer====";

  // Define a duration to be slept by inside the called functions.
  const std::chrono::duration<x_view::real_t> sleep_duration(
      static_cast<x_view::real_t>(0.05));
  const x_view::real_t tolerance = static_cast<x_view::real_t>(0.001);

  // Check that the real timer implementation works as expected,
  // i.e. by measuring the elapsed time precisely.
  Timer timer;
  bool register_timer_1 = timer.registerTimer("Function1");
  bool register_timer_2 = timer.registerTimer("Function2");
  bool register_duplicate_timer = timer.registerTimer("Function2");
  CHECK(register_timer_1);
  CHECK(register_timer_2);
  CHECK(!register_duplicate_timer);

  timer.start("Function1");
  waitFunction(sleep_duration);
  const auto duration_1 = timer.stop("Function1");
  CHECK_NEAR(duration_1.count(), sleep_duration.count(), tolerance);

  timer.start("Function2");
  waitFunction(sleep_duration * 2);
  const auto duration_2 = timer.stop("Function2");
  CHECK_NEAR(duration_2.count(), sleep_duration.count() * 2, tolerance);


  // Check that the NullTimer does not measure any time duration.
  NullTimer null_timer;
  null_timer.registerTimer("Function1");
  null_timer.registerTimer("Function2");

  null_timer.start("Function1");
  waitFunction(sleep_duration);
  const auto duration_3 = null_timer.stop("Function1");

  CHECK_NEAR(duration_3.count(), static_cast<x_view::real_t>(0.0), tolerance);

}


