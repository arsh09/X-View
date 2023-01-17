#ifndef X_VIEW_SEMANTIC_LANDMARK_FACTORY_H
#define X_VIEW_SEMANTIC_LANDMARK_FACTORY_H

#include <x_view_core/x_view_types.h>

#include <opencv2/core/core.hpp>

namespace x_view {

/**
 * \brief Class responsible for creating new landmarks.
 */
class SemanticLandmarkFactory {

 public:

  /// \brief Function pointer passed to the factory to create new landmarks.
  typedef std::function<SemanticLandmarkPtr(const FrameData&)> CreateCallBack;

  /**
   * \brief Sets the landmark type the factory is going to generate.
   * \param callback Function pointer called to create a new landmark.
   */
  static void setCreatorFunction(CreateCallBack callback);

  /**
   * \brief Function exposed to the user to create new semantic landmark
   * objects.
   * \param frame_data Data associated to the current frame.
   * \return landmark Pointer to abstract base landmark class which is filled
   * up with a concrete landmark instance.
   */
  static SemanticLandmarkPtr createSemanticLandmark(
      const FrameData& frame_data);

 private:
  ///\brief A function pointer to the static function responsible for
  /// creating new semantic landmarks given an image and a pose.
  static CreateCallBack callback_;

};
}

#endif //X_VIEW_SEMANTIC_LANDMARK_FACTORY_H
