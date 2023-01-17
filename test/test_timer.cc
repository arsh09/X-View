#include "test_timer.h"

#include <thread>

namespace x_view_test {

void waitFunction(const std::chrono::duration<x_view::real_t>& duration) {
	std::this_thread::sleep_for(duration);
	return;
}

}
