#ifndef X_VIEW_GRAPH_DRAWER_H
#define X_VIEW_GRAPH_DRAWER_H

#include <x_view_core/features/graph_descriptor.h>
#include <x_view_core/landmarks/graph_landmark/graph_landmark_types.h>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

namespace x_view {

class GraphDrawer {

 public:
  /**
   * \brief Prints the blob structure to the LOG(INFO).
   * \param blobs Blob datastructure to be printed.
   */
  static void printBlobs(const ImageBlobs& blobs);

  /**
   * \brief Generates a new image of size size representing the blobs
   * contained in the ImageBlobs datastructure passed as arguments with
   * additional information about the graph structure built upon it.
   * \param blobs ImageBlobs datastructure containing the blobs to be drawn.
   * \param graph Graph built upon the blob structure.
   * \param size Size og the image to be generated.
   * \return Image representing the blobs contained in the ImageBlobs
   * datastructure passed as argument and labels representing the graph
   * passed as argument.
   * \details This function is a utility function which internally calls
   * other drawing methods in the correct order, such that the resulting
   * image clearly shows all drawable data.
   */
  static cv::Mat createImageWithLabels(const ImageBlobs& blobs,
                                       const Graph& graph,
                                       const cv::Size& size);

  /**
   * \brief Generates a new image os size size representing the blobs
   * contained in the ImageBlobs datastructure passed as argument.
   * \param blobs ImageBlobs datastructure containing the blobs to be drawn.
   * \param size Size og the image to be generated.
   * \return Image representing the blobs contained in the ImageBlobs
   * datastructure passed as argument.
   */
  static cv::Mat createImageFromBlobs(const ImageBlobs& blobs,
                                      const cv::Size& size);

  /**
   * \brief Adds text/labels to the image positioned at the center of the
   * corresponding blob.
   * \param blobs ImageBlobs datastructure containing the labels to be rendered.
   * \param image The labels are rendered on top of the image passed as
   * argument.
   */
  static void addLabelsToImage(const ImageBlobs& blobs, cv::Mat* image);

  /**
   * \brief Adds ellipses to the image representing the blobs.
   * \param blobs ImageBlobs datastructure containing the labels to be rendered.
   * \param image The ellipses are rendered on top of the image passed as
   * argument.
   */
  static void addEllipsesToImage(const ImageBlobs& blobs, cv::Mat* image);

  /**
  * \brief Adds the graph nodes to the image representing the blobs.
  * \param graph Graph containing the nodes to be rendered.
  * \param image The nodes are rendered on top of the image passed as
  * argument.
  */
  static void addGraphNodesToImage(const Graph& graph, cv::Mat* image);

  /**
  * \brief Adds the graph edges to the image representing the blobs.
  * \param graph Graph containing the nodes to be rendered.
  * \param image The edges are rendered on top of the image passed as
  * argument.
  */
  static void addGraphEdgesToImage(const Graph& graph, cv::Mat* image);


  /**
   * \brief Adds coordinate labels to the vertices of the graph passed as
   * parameter.
   * \param graph Graph constaining the nodes to be rendered.
   * \param image The labels are rendered ontop of the image passed as argument.
   */
  static void addCoordinatesToImage(const Graph& graph, cv::Mat* image);

  //===================== Color and line specifications ======================//
  static void setVertexColor(const cv::Scalar& color) { vertex_color_ = color; }
  static void setVertexRadius(const int radius) { vertex_radius_ = radius; }
  static void setEdgeColor(const cv::Scalar& color) { edge_color_ = color; }
  static void setEdgeThickness(const int thickness) {
    edge_thickness_ = thickness;
  }
  static void setEllipseColorStatic(const cv::Scalar& color) {
    ellipse_color_static_ = color;
  }
  static void setEllipseColorDynamic(const cv::Scalar& color) {
    ellipse_color_dynamic_ = color;
  }
  static void setEllipseThickness(const int thickness) {
    ellipse_thickness_ = thickness;
  }
  static void setLabelColor(const cv::Scalar& color) { label_color_ = color; }
  static void setLabelScale(const real_t scale) { label_scale_ = scale; }

  /**
   * \brief Resets the color and line specifications to their default value.
   */
  static void resetProperties();

 private:
  static cv::Scalar vertex_color_;
  static int vertex_radius_;

  static cv::Scalar edge_color_;
  static int edge_thickness_;

  static cv::Scalar ellipse_color_static_;
  static cv::Scalar ellipse_color_dynamic_;
  static int ellipse_thickness_;

  static cv::Scalar label_color_;
  static real_t label_scale_;

};

}

#endif //X_VIEW_GRAPH_DRAWER_H
