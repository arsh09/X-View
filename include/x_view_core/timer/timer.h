#ifndef X_VIEW_TIMER_H
#define X_VIEW_TIMER_H

#include <x_view_core/timer/abstract_timer.h>
#include <x_view_core/x_view_types.h>

#include <unordered_map>
#include <vector>

namespace x_view {

/**
 * \brief Implementation of the timer interface with measures the time
 * between the start and stop function calls.
 */
class Timer : public AbstractTimer {

  // Timer printer class is a friend class as it can have access to all
  // private variables of the Timer class.

  friend class TimerPrinter;

 public:

  Timer();

  virtual bool registerTimer(const std::string& timer_name,
                             const std::string& parent_timer_name = "root")  override;
  virtual void start(const std::string& timer_name) override;
  virtual const AbstractTimer::ElapsedTimeType stop(
      const std::string& timer_name) override;

  /**
   * \brief Generates a vector containing all measured timings associated to
   * the timer name passed as argument.
   * \param timer_name Name of timer to be returned.
   * \return A vector containing the raw timings associated to the timer name
   * expressed in seconds.
   */
  const std::vector<x_view::real_t> getTimes(
      const std::string& timer_name) const;

  /**
   * \brief Generates a datastructure mapping all registered timers to a
   * vector of measured timings.
   * \return An unordered map keyed by the timer name associated to a vector
   * of timing measurements.
   */
  const std::unordered_map<std::string, std::vector<x_view::real_t>>
  getAllTimings() const;

 private:
  class TimerNode {
   public:
    TimerNode();
    void start();
    const std::chrono::steady_clock::duration stop();
    const std::chrono::steady_clock::duration elapsedTime() const;

   private:

    bool has_been_started_;
    bool has_been_stopped_;

    std::chrono::steady_clock::time_point start_;
    std::chrono::steady_clock::duration elapsed_time_;
  };

  /// \brief Computes and returns the mean measured time associated to the
  /// vector of TimerNodes passed as argument expressed in seconds.
  static x_view::real_t getMean(const std::vector<TimerNode>& timers);

  /// \brief Computes and returns the standard deviation of the measured time
  /// associated to the vector of TimerNodes passed as argument expressed in
  /// seconds.
  static x_view::real_t getStd(const std::vector<TimerNode>& timers);

  /// \brief A map mapping strings (timer names) to the vector of measured
  /// timers.
  std::unordered_map<std::string, std::vector<TimerNode>> timer_map_;

  /// \brief Vector keeping track of the timer registration order.
  std::vector<std::string> insertion_order_;
};


}

#endif //X_VIEW_TIMER_H
