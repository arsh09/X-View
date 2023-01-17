#ifndef X_VIEW_GRAPH_LANDMARK_TYPES_H
#define X_VIEW_GRAPH_LANDMARK_TYPES_H

#include<vector>

namespace x_view {

class Blob;
class GraphLandmark;

/// \brief Each semantic class might have multiple instances, all
/// contained inside a vector of Blobs.
typedef std::vector<Blob> ClassBlobs;

/// \brief Vector containing a list of ClassBlobs, one for each semantic
/// class.
typedef std::vector<ClassBlobs> ImageBlobs;

}

#endif //X_VIEW_GRAPH_LANDMARK_TYPES_H
