#ifndef X_VIEW_ABSTRACT_TIMER_H
#define X_VIEW_ABSTRACT_TIMER_H

#include <x_view_core/x_view_types.h>

#include <chrono>
#include <string>
#include <unordered_map>

namespace x_view {

/**
 * \brief This class defines an interface for all timer classes used in X-View.
 */
class AbstractTimer {

 public:

  typedef std::chrono::duration<x_view::real_t, std::ratio<1, 1>>
      ElapsedTimeType;

  /**
   * \brief Whenever the user wants to time a new piece of code, it has to
   * register that timing with a name.
   * \param timer_name Name used to refer to a specific timer measurement.
   * \param parent_timer_name Name of the parent timer. If left blank, then
   * timer_name is assumed to be a top-level timer.
   * \return True if the registration was successful, false if a timer with
   * the same name was already registered.
   */
  virtual bool registerTimer(const std::string& timer_name,
                             const std::string& parent_timer_name = "root") = 0;

  /**
   * \brief Start the timer associated with the name passed as argument.
   * \param timer_name Name used to refer to a specific timer measurement.
   */
  virtual void start(const std::string& timer_name) = 0;

  /**
   * \brief Stops the timer associated with the name massed as argument.
   * \param timer_name Name used to refer to a specific timer measurement.
   * \return Measured time expressed in seconds.
   */
  virtual const ElapsedTimeType stop(const std::string& timer_name) = 0;

 protected:
  /// \brief A map between strings mapping a timer_name to the timer_name of
  /// its father.
  std::unordered_map<std::string, std::string> parent_timer_;

  /// \brief A map between strings mapping a timer_name to a list of
  /// timer_names of its children.
  std::unordered_map<std::string, std::set<std::string>> children_timers_;
};


}

#endif //X_VIEW_ABSTRACT_TIMER_H
