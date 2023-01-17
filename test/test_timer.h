#ifndef X_VIEW_TEST_TIMER_H
#define X_VIEW_TEST_TIMER_H

#include <x_view_core/x_view_types.h>

#include <chrono>

namespace x_view_test {

/**
 * \brief A function that waits the time duration passed as argument.
 * \param duration The duration this function has to wait.
 */
void waitFunction(const std::chrono::duration<x_view::real_t>& duration);

}

#endif //X_VIEW_TEST_TIMER_H
