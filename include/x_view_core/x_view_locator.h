#ifndef X_VIEW_LOCATOR_H
#define X_VIEW_LOCATOR_H

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/parameters/parameters.h>
#include <x_view_core/timer/abstract_timer.h>

namespace x_view {

class Locator {
 public:

  /**
   * \brief Access to the current parameters.
   * \return A unique pointer reference to the current parameters.
   * \code{cpp}
   * const auto parameter = Locator::getParameters();
   * parameters->getFloat(...)
   * \endcode
   */
  static const std::unique_ptr<Parameters>& getParameters();

  /**
   * \brief Access to the current dataset.
   * \return A unique pointer reference to the current dataset.
   * \code{cpp}
   * const auto dataset = Locator::getDataset();
   * int num_classes = dataset->numSemanticClasses();
   * \endcode
   */
  static const std::unique_ptr<AbstractDataset>& getDataset();

  /**
   * \brief Access to the current timer.
   * \return A unique pointer reference to the current timer.
   * \code{cpp}
   * const auto timer = Locator::getTimer();
   * timer->registerTimer("newTimerName");
   * timer->start("newTimerName");
   * {...}
   * timer->stop("newTimerName");
   * \endcode
   */
  static const std::unique_ptr<AbstractTimer>& getTimer();

  /**
   * \brief Register a new parameter instance in the locator.
   * \param parameters Moved unique pointer pointing to a Parameter instance.
   * \code{cpp}
   * std::unique_ptr<Parameters> parameters(new Parameters());
   * Locator::registerParameters(std::move(parameters));
   * // parameters is no longer a valid pointer here!!
   * // need to use Locator::getParameters() instead
   * \endcode
   */
  static void registerParameters(std::unique_ptr<Parameters> parameters);

  /**
   * \brief Register a new dataset instance in the locator.
   * \param dataset Moved unique pointer pointing to a AbstractDataset instance.
   * \code{cpp}
   * std::unique_ptr<AbstractDataset> dataset(new SynthiaDataset());
   * Locator::registerDataset(std::move(dataset));
   * // dataset is no longer a valid pointer here!!
   * // need to use Locator::getDataset() instead
   * \endcode
   */
  static void registerDataset(std::unique_ptr<AbstractDataset> dataset);

  /**
   * \brief Register a new timer instance in the locator.
   * \param timer Moved unique pointer pointing to a AbstractTimer instance.
   * \code{cpp}
   * std::unique_ptr<AbstractTimer> timer(new Timer());
   * Locator::registerTimer(std::move(timer));
   * // timer is no longer a valid pointer here!!
   * // need to use Locator::getTimer() instead
   * \endcode
   */
  static void registerTimer(std::unique_ptr<AbstractTimer> timer);

  /**
   * \brief Removes the current timer from the locator and returns a pointer
   * to it. The caller is responsible for deleting the pointer.
   * \return A pointer to the current timer.
   * \note The Locator no longer has a registered timer instance.
   */
  static AbstractTimer* removeTimer();

 private:
  static std::unique_ptr<Parameters> parameters_;
  static std::unique_ptr<AbstractDataset> dataset_;
  static std::unique_ptr<AbstractTimer> timer_;
};

}

#endif //X_VIEW_LOCATOR_H
