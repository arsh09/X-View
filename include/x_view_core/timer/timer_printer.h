#ifndef X_VIEW_TIMER_PRINTER_H
#define X_VIEW_TIMER_PRINTER_H

#include <x_view_core/timer/timer.h>

namespace x_view {

/**
 * \brief Class implementing methods to print a timer in a human readable
 * format.
 */
class TimerPrinter {

 public:

  static bool USE_COLORS;

  /**
   * \brief Generates a human readable table with information about the
   * registered timers.
   * \param timer Timer to be transformed into a table string.
   * \return A string containing the generated table.
   */
  static const std::string getTimingsTable(const Timer& timer);

  /**
   * \brief Generates a human readable tree structure with information about
   * the registered timers.
   * \param timer Timer to be transformed into a tree string.
   * \return A string containing the generated tree.
   */
  static const std::string getTimingsTree(const Timer& timer);

 private:

  static const std::string getSubTreeTimings(const Timer& timer,
                                             const std::string& parent_timer,
                                             const std::string& indentation);

  /// \brief Color codes for terminal output.
  enum COLOR {
    RED = 31,
    GREEN = 32,
    BLUE = 34,
    DEF = 39
  };

  /// \brief Generates a string which is printed as bold in the terminal.
  static const std::string boldify(const std::string& s);

  /// \brief Generates a string which is printed in the color specified in
  /// the second argument in the terminal.
  static const std::string color(const std::string& s, const COLOR color);

  static const std::string TIMER_INDENTATION;
};

}

#endif //X_VIEW_TIMER_PRINTER_H
