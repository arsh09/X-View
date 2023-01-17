#include <x_view_core/landmarks/graph_landmark/graph_drawer.h>

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/landmarks/graph_landmark/blob.h>
#include <x_view_core/x_view_locator.h>
#include <x_view_core/x_view_tools.h>

namespace x_view {

cv::Scalar GraphDrawer::vertex_color_ = CV_RGB(141, 28,216);
int GraphDrawer::vertex_radius_ = 4;

cv::Scalar GraphDrawer::edge_color_ = CV_RGB(255, 189, 58);
int GraphDrawer::edge_thickness_ = 2;

cv::Scalar GraphDrawer::ellipse_color_static_ = CV_RGB(0, 40, 255);
cv::Scalar GraphDrawer::ellipse_color_dynamic_ = CV_RGB(255, 60, 10);
int GraphDrawer::ellipse_thickness_ = 2;

cv::Scalar GraphDrawer::label_color_ = CV_RGB(255, 255, 255);
real_t GraphDrawer::label_scale_ = 0.65;


void GraphDrawer::resetProperties() {
  GraphDrawer::vertex_color_ = CV_RGB(141, 28,216);
  GraphDrawer::vertex_radius_ = 4;

  GraphDrawer::edge_color_ = CV_RGB(255, 189, 58);
  GraphDrawer::edge_thickness_ = 2;

 GraphDrawer::ellipse_color_static_ = CV_RGB(0, 40, 255);
  GraphDrawer::ellipse_color_dynamic_ = CV_RGB(255, 60, 10);
  GraphDrawer::ellipse_thickness_ = 2;

  GraphDrawer::label_color_ = CV_RGB(255, 255, 255);
  GraphDrawer::label_scale_ = 0.65;
}

void GraphDrawer::printBlobs(const ImageBlobs& blobs) {
  for (int c = 0; c < blobs.size(); ++c) {
    LOG(INFO) << "Found " << blobs[c].size()
              << " instances of class " << c << ":";
    for (int i = 0; i < blobs[c].size(); ++i) {
      LOG(INFO) << "\tInstance " << i << " composed by "
                << blobs[c][i].num_pixels << " pixels with mean "
                    "pixel " << blobs[c][i].pixel_center;
    }
  }
}

cv::Mat GraphDrawer::createImageWithLabels(const ImageBlobs& blobs,
                                           const Graph& graph,
                                          const cv::Size& size) {
  cv::Mat image = GraphDrawer::createImageFromBlobs(blobs, size);
  GraphDrawer::addGraphEdgesToImage(graph, &image);
  GraphDrawer::addGraphNodesToImage(graph, &image);
  GraphDrawer::addEllipsesToImage(blobs, &image);
  GraphDrawer::addLabelsToImage(blobs, &image);
  GraphDrawer::addCoordinatesToImage(graph, &image);

  return image;
}

cv::Mat GraphDrawer::createImageFromBlobs(const ImageBlobs& blobs,
                                          const cv::Size& size) {
  cv::Mat image(size.height, size.width, CV_8UC3, cv::Scalar::all(0));

  const auto& dataset = Locator::getDataset();

  const std::vector<int>& labels_to_render =
      dataset->getLabelsToRender();

  // Draw the blobs onto the image
  for (int c = 0; c < blobs.size(); ++c) {
    // only add the blob if the label has not to be ignored, thus if it can
    // not be found in the passed parameter
    if (std::find(labels_to_render.begin(), labels_to_render.end(), c) !=
        std::end(labels_to_render))
      for (const Blob& blob : blobs[c]) {
        const int semantic_label = blob.semantic_label;
        const cv::Scalar color = getColorFromSemanticLabel(semantic_label);

        std::vector<std::vector<cv::Point2i>> v_contours;
        v_contours.push_back(blob.external_contour_pixels);

        cv::drawContours(image, v_contours, 0, color, CV_FILLED);
      }
  }
  return image;
}

void GraphDrawer::addLabelsToImage(const ImageBlobs& blobs, cv::Mat* image) {

  CHECK_NOTNULL(image);

  const auto& dataset = Locator::getDataset();

  const std::vector<int>& labels_to_render =
      dataset->getLabelsToRender();

  for (int c = 0; c < blobs.size(); ++c) {
    // only add the blob if the label has not to be ignored, thus if it can
    // not be found in the passed parameter
    if (std::find(labels_to_render.begin(), labels_to_render.end(), c) !=
        std::end(labels_to_render))
      for (const Blob& blob : blobs[c]) {
        const std::string label = std::to_string(blob.semantic_label);
        const std::string label_descr =
            dataset->label(blob.semantic_label);

        const std::string text = label + ") " + label_descr;
        cv::putText(*image, text, blob.pixel_center, cv::FONT_HERSHEY_DUPLEX,
                    GraphDrawer::label_scale_, GraphDrawer::label_color_,
                    1, CV_AA);
        // if the blob has an instance id associated to it, render it on a
        // new line.
        // new line.
        if (blob.instance != -1) {
          const std::string instance =
              "id: " + std::to_string(blob.instance);
          cv::putText(*image, instance, blob.pixel_center + cv::Point2i(0, 20),
                      cv::FONT_HERSHEY_DUPLEX, GraphDrawer::label_scale_,
                      GraphDrawer::label_color_, 1, CV_AA);

        }
      }
  }
}

void GraphDrawer::addEllipsesToImage(const ImageBlobs& blobs, cv::Mat* image) {

  CHECK_NOTNULL(image);

  const auto& dataset = Locator::getDataset();

  const std::vector<int>& labels_to_render =
      dataset->getLabelsToRender();

  // Draw the blobs onto the image
  for (int c = 0; c < blobs.size(); ++c) {
    // only add the blob if the label has not to be ignored, thus if it can
    // not be found in the passed parameter
    if (std::find(labels_to_render.begin(), labels_to_render.end(), c) !=
        std::end(labels_to_render))
      for (const Blob& blob : blobs[c]) {
        cv::Scalar ellipse_color;
        if (dataset->semanticEntities()[c].is_static)
          ellipse_color = GraphDrawer::ellipse_color_static_;
        else
          ellipse_color = GraphDrawer::ellipse_color_dynamic_;

        cv::ellipse(*image, blob.ellipse, ellipse_color,
                    GraphDrawer::ellipse_thickness_);

      }
  }
}

void GraphDrawer::addGraphNodesToImage(const Graph& graph, cv::Mat* image) {

  CHECK_NOTNULL(image);

  const auto& dataset = Locator::getDataset();

  const std::vector<int>& labels_to_render =
      dataset->getLabelsToRender();

  auto node_iter = boost::vertices(graph);
  for (; node_iter.first != node_iter.second; ++node_iter.first) {
    const VertexDescriptor& node_descriptor = *node_iter.first;
    const VertexProperty& node = graph[node_descriptor];

    const cv::Point2i& center = node.center;
    const int label = node.semantic_label;

    if (std::find(labels_to_render.begin(), labels_to_render.end(), label) !=
        std::end(labels_to_render)) {
      cv::circle(*image, center, GraphDrawer::vertex_radius_,
                 GraphDrawer::vertex_color_, CV_FILLED);
    }
  }
}

void GraphDrawer::addGraphEdgesToImage(const Graph& graph, cv::Mat* image) {

  CHECK_NOTNULL(image);

  const auto& dataset = Locator::getDataset();

  const std::vector<int>& labels_to_render =
      dataset->getLabelsToRender();

  // second, add the edges between the nodes of the graph
  auto edge_iter = boost::edges(graph);
  for (; edge_iter.first != edge_iter.second; ++edge_iter.first) {
    // get the two vertices connected by this edge
    const VertexDescriptor& vd_from = boost::source(*edge_iter.first, graph);
    const VertexDescriptor& vd_to = boost::target(*edge_iter.first, graph);

    const VertexProperty& v_from = graph[vd_from];
    const VertexProperty& v_to = graph[vd_to];

    const int from_label = v_from.semantic_label;
    const int to_label = v_to.semantic_label;

    // only draw the edge if both nodes are to render
    if (std::find(labels_to_render.begin(),
                  labels_to_render.end(),
                  from_label)
        != std::end(labels_to_render) &&
        std::find(labels_to_render.begin(), labels_to_render.end(), to_label)
            != std::end(labels_to_render)) {
      const cv::Point2i& from_center = v_from.center;
      const cv::Point2i& to_center = v_to.center;

      cv::line(*image, from_center, to_center, GraphDrawer::edge_color_,
               GraphDrawer::edge_thickness_);
    }
  }

}


void GraphDrawer::addCoordinatesToImage(const Graph& graph, cv::Mat* image) {

  CHECK_NOTNULL(image);

  const auto& dataset = Locator::getDataset();

  const std::vector<int>& labels_to_render =
      dataset->getLabelsToRender();

  auto node_iter = boost::vertices(graph);
  for (; node_iter.first != node_iter.second; ++node_iter.first) {
    const VertexDescriptor& node_descriptor = *node_iter.first;
    const VertexProperty& node = graph[node_descriptor];

    const cv::Point2i& center = node.center;
    const Vector3r& location_3d = node.location_3d;
    const int label = node.semantic_label;

    if(location_3d != Vector3r::Zero())
    if (std::find(labels_to_render.begin(), labels_to_render.end(), label) !=
        std::end(labels_to_render)) {
      const std::string coord =
          "coord: [" + std::to_string(location_3d[0]) + ", " + std::to_string
              (location_3d[1]) + ", " + std::to_string(location_3d[2]) + "]";
      cv::putText(*image, coord, center + cv::Point2i(0, 40),
                  cv::FONT_HERSHEY_DUPLEX, GraphDrawer::label_scale_ * 0.5,
                  GraphDrawer::label_color_, 1, CV_AA);
    }
  }
}

}
