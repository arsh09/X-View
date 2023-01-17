#ifndef X_VIEW_ABSTRACT_MATCHER_H
#define X_VIEW_ABSTRACT_MATCHER_H

#include <x_view_core/x_view_types.h>

namespace x_view {

/**
 * \brief An interface each landmark-matcher must implement.
 */
class AbstractMatcher {

 public:
  AbstractMatcher() {}
  virtual ~AbstractMatcher() {}

  /**
   * \brief Each landmark matcher returns a different type of matching result.
   * For this reason, each class implementing the AbstractMatcher
   * interface also has to define an ReturnType for the match() function
   * which contains information on the matching result.
   */
  class AbstractMatchingResult {
   public:
    AbstractMatchingResult()
        : matching_found_(false),
          matching_landmark_index_(-1) {}
    virtual ~AbstractMatchingResult() {};

   protected:
    /**
     * \brief Indicates if the passed landmark has been matched or not to an
     * other previously inserted in the database.
     */
    bool matching_found_;

    /**
     * \brief If matching_found_ is true, then matching_landmark_index_
     * indicates which previously visited landmark is the closest match to
     * the queried one.
     */
    int matching_landmark_index_;
  };

  typedef std::shared_ptr<AbstractMatchingResult> MatchingResultPtr;

  /**
   * \brief Computes a match between the query landmark passed as parameter
   * and the features internally stored. The descriptor associated to the
   * query landmark is added to the internal matcher representation of
   * landmarks.
   * \param query_landmark Const reference to semantic landmark pointer.
   * \return Reference to matching result pointer containing matching results.
   */
  virtual MatchingResultPtr match(const SemanticLandmarkPtr& query_landmark)
  = 0;

  /**
   * \brief Since while processing the first frame in the program we don't have
   * to match the corresponding descriptor to anything, we simply want to add
   * the associated descriptor to the internal representation of the matcher.
   * \param descriptor Pointer pointing to descriptor to add to the matcher.
   * \note This function should conceptually be only called once per
   * AbstractMatcher instance.
   */
  virtual void addDescriptor(const ConstDescriptorPtr& descriptor) {}

};

}

#endif //X_VIEW_ABSTRACT_MATCHER_H
