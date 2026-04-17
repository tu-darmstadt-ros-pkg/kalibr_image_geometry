#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#ifndef EXTENDED_CAMERA_LOADER__COLOR_MAPS_H_
#define EXTENDED_CAMERA_LOADER__COLOR_MAPS_H_

namespace extended_image_geometry {

static const std::unordered_map<std::string, cv::ColormapTypes> colormap_map = {
  {"autumn",  cv::COLORMAP_AUTUMN},
  {"bone",    cv::COLORMAP_BONE},
  {"jet",     cv::COLORMAP_JET},
  {"winter",  cv::COLORMAP_WINTER},
  {"rainbow", cv::COLORMAP_RAINBOW},
  {"ocean",   cv::COLORMAP_OCEAN},
  {"summer",  cv::COLORMAP_SUMMER},
  {"spring",  cv::COLORMAP_SPRING},
  {"cool",    cv::COLORMAP_COOL},
  {"hsv",     cv::COLORMAP_HSV},
  {"pink",    cv::COLORMAP_PINK},
  {"hot",     cv::COLORMAP_HOT},
  {"parula",  cv::COLORMAP_PARULA},
  {"inferno", cv::COLORMAP_INFERNO},
  {"magma",   cv::COLORMAP_MAGMA},
  {"plasma",  cv::COLORMAP_PLASMA},
  {"viridis", cv::COLORMAP_VIRIDIS},
};

inline cv::ColormapTypes getColormapType(const std::string& color_map_name) {
  auto it = colormap_map.find(color_map_name);
  if (it != colormap_map.end()) {
    return it->second;
  } else {
    return cv::COLORMAP_INFERNO;
  }
}

inline void applyColorMapRanged(const cv::Mat& input, cv::Mat& output, cv::ColormapTypes color_map, int min_value, int max_value) {
  cv::Mat fp32;
  input.convertTo(fp32, CV_32F);
  // TODO check whether higher res would make sense here
  cv::Mat norm8u;
  fp32.convertTo(norm8u, CV_8U,
            255.0 / (max_value - min_value),
            -min_value * 255.0 / (max_value - min_value));
  // cv::Mat norm = (fp32 - min_value) / (max_value - min_value);
  // cv::min(norm, 1.0f, norm);
  // cv::max(norm, 0.0f, norm);
  // cv::Mat norm8u;
  // norm.convertTo(norm8u, CV_8U, 255.0);
  cv::applyColorMap(norm8u, output, color_map);
  cv::cvtColor(output, output, cv::COLOR_BGR2RGB);
}

} // namespace extended_image_geometry

#endif // EXTENDED_CAMERA_LOADER__COLOR_MAPS_H_