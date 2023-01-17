#include <x_view_core/x_view_locator.h>

#include <x_view_core/timer/null_timer.h>

namespace x_view {

std::unique_ptr<Parameters> Locator::parameters_(new Parameters("EMPTY"));

std::unique_ptr<AbstractDataset> Locator::dataset_(new AbstractDataset(-1));

std::unique_ptr<AbstractTimer> Locator::timer_(new NullTimer());

const std::unique_ptr<Parameters>& Locator::getParameters() {
  return Locator::parameters_;
}

const std::unique_ptr<AbstractDataset>& Locator::getDataset() {
  return Locator::dataset_;
}

const std::unique_ptr<AbstractTimer>& Locator::getTimer() {
  return Locator::timer_;
}


void Locator::registerParameters(std::unique_ptr<Parameters> parameters) {

  parameters_ = std::move(parameters);
}

void Locator::registerDataset(std::unique_ptr<AbstractDataset> dataset) {
  dataset_ = std::move(dataset);
}

void Locator::registerTimer(std::unique_ptr<AbstractTimer> timer) {
  timer_ = std::move(timer);
}

AbstractTimer* Locator::removeTimer() {
  return timer_.release();
}


}

