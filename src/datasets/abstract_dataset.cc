#include <x_view_core/datasets/abstract_dataset.h>

#include <cv_bridge/cv_bridge.h>

namespace enc = sensor_msgs::image_encodings;

namespace x_view {

AbstractDataset::AbstractDataset(const int num_semantic_classes)
    : num_semantic_classes_(num_semantic_classes),
      camera_intrinsics_(500.0, 400, 300) {
  // Create simple semantic entities.
  for (int i = 0; i < num_semantic_classes_; ++i) {
    // Default semantic entities, all static and all to render.
    semantic_entities_.push_back(SemanticEntity(std::to_string(i), i));
  }

  // Set up trivial camera-to-image rotation.
  camera_to_image_rotation_ = Matrix3r::Identity();
}

const x_view::real_t AbstractDataset::getDepth(const cv::Point2i& pixel,
                                               const cv::Mat& depth_image) const {
  return depth_image.at < uint8_t > (pixel);
}

cv::Mat AbstractDataset::convertSemanticImage(
    const sensor_msgs::ImageConstPtr& msg) const {
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);

    cv::Mat image = cv_ptr->image;

    return image;
  }
  catch (cv_bridge::Exception& e) {
    LOG(FATAL) << "Could not convert from '" << msg->encoding
               << "' to '" << enc::BGR8 << "'\nError: " << e.what();
  }
}

const std::vector<int> AbstractDataset::getLabelsToRender() const {
  std::vector<int> labels_to_render;
  labels_to_render.reserve(num_semantic_classes_);
  for (auto const& c : semantic_entities_)
    if (c.is_to_render)
      labels_to_render.push_back(c.semantic_entity_id);

  return labels_to_render;
}

const std::vector<int> AbstractDataset::getStaticLabels() const {
  std::vector<int> static_labels;
  static_labels.reserve(num_semantic_classes_);
  for (auto const& c : semantic_entities_)
    if (c.is_static)
      static_labels.push_back(c.semantic_entity_id);

  return static_labels;
}

const std::vector<int> AbstractDataset::getDynamicLabels() const {
  std::vector<int> dynamic_labels;
  dynamic_labels.reserve(num_semantic_classes_);
  for (auto const& c : semantic_entities_)
    if (!c.is_static)
      dynamic_labels.push_back(c.semantic_entity_id);

  return dynamic_labels;
}

const std::vector<int> AbstractDataset::getLabelsToIncludeInGraph() const {
  std::vector<int> labels_to_include_in_graph;
  labels_to_include_in_graph.reserve(num_semantic_classes_);
  for (auto const& c : semantic_entities_)
    if (c.is_to_include_in_graph)
      labels_to_include_in_graph.push_back(c.semantic_entity_id);

  return labels_to_include_in_graph;
}

const uint64_t AbstractDataset::largestLabelSize() const {
  return std::max_element(semantic_entities_.begin(), semantic_entities_.end(),
                          [](const SemanticEntity& s1,
                             const SemanticEntity& s2) {
                            return s1.semantic_entity_name.length() <
                                s2.semantic_entity_name.length();
                          })->semantic_entity_name.length();
}

std::ostream& operator<<(std::ostream& out, const AbstractDataset& dataset) {
  out << "Dataset name: " << dataset.datasetName() << std::endl << std::endl;
  const uint64_t max_label_length = dataset.largestLabelSize() + 1;
  out << std::setfill(' ');
  out << std::left << std::setw(4) << "id:"
      << std::left << std::setw(max_label_length) << "label:"
      << std::left << std::setw(16) << "graph-relevant"
      << std::left << std::setw(17) << "static/dynamic"
      << std::left << std::setw(10) << "drawable";
  for (const auto& elem : dataset.semanticEntities()) {
    out << "\n" << std::left << std::setw(4) << elem.semantic_entity_id;
    out << std::left << std::setw(max_label_length)
        << elem.semantic_entity_name;
    out << std::left << std::setw(16) << (elem.is_to_include_in_graph ?
                                          "yes" : "no");
    out << std::left << std::setw(17) << (elem.is_static ? "static"
                                                          : "dynamic");
    out << std::left << std::setw(10) << (elem.is_to_render ? "yes" : "no");
  }
  return out;
}

std::ostream& operator<<(std::ostream& out,
                         const std::unique_ptr<AbstractDataset>& ptr) {
  return out << *(ptr.get());
}

}
