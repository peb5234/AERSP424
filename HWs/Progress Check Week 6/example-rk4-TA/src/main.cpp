#include <vector>
#include <limits>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp> 

#include "plane.hpp"

std::pair<double, double> findMinMax(const std::vector<Eigen::Vector3d>& data) {
  double minVec = std::numeric_limits<double>::infinity();
  double maxVec = -std::numeric_limits<double>::infinity();

  for (const auto& vec : data) {
      minVec = std::min(minVec, vec.x());
      minVec = std::min(minVec, vec.y());
      minVec = std::min(minVec, vec.z());

      maxVec = std::max(maxVec, vec.x());
      maxVec = std::max(maxVec, vec.y());
      maxVec = std::max(maxVec, vec.z());
  }

  return std::make_pair(minVec, maxVec);
}

/**
 * @brief  The function to draw the line chart
 * @note   
 * @param  data: state data
 * @param  width: window width
 * @param  height: window height
 * @param  margin: the margin of the plot to the window
 * @param  windowName: the name of the window
 * @retval None
 */
void drawLineChart(std::vector<Eigen::Vector3d>& data, std::vector<double>& times, int width, int height, int margin, const std::string& windowName) {
    
    cv::Mat image = cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    int num_points = times.size();
    int step_index = 1;
    if (num_points > width - 2 * margin) {
      step_index = ceil(1.0f * num_points / (width - 2 * margin));
    }
// step_index = 1;
    int step_x = std::max((width - 2 * margin) / num_points, 1);
    std::pair<double, double> minMax = findMinMax(data);
    double scale_y = static_cast<double>(height - 2 * margin) / (minMax.second - minMax.first);

    // draw the x and y axis
    cv::line(image, cv::Point(margin, margin), cv::Point(margin, height - margin), cv::Scalar(0, 0, 0), 2);  // y axis
    cv::line(image, cv::Point(margin, height - margin), cv::Point(width - margin, height - margin), cv::Scalar(0, 0, 0), 2);  // x axis

    // draw the lines
    for (int i = 0; i * step_index < num_points && i * step_index + step_index < num_points; i += 1) {
      int x1 = margin + i * step_x;
      int y1 = height - margin - static_cast<int>((data[i * step_index](0, 0) - minMax.first) * scale_y);
      int x2 = margin + (i * step_x + step_index) * step_x;
      int y2 = height - margin - static_cast<int>((data[i * step_index + step_index](0, 0) - minMax.first) * scale_y);
      cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 0), 1);

      x1 = margin + i * step_x;
      y1 = height - margin - static_cast<int>((data[i * step_index](1, 0) - minMax.first) * scale_y);
      x2 = margin + (i * step_x + step_index) * step_x;
      y2 = height - margin - static_cast<int>((data[i * step_index + step_index](1, 0) - minMax.first) * scale_y);
      cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 1);

      x1 = margin + i * step_x;
      y1 = height - margin - static_cast<int>((data[i * step_index](2, 0) - minMax.first) * scale_y);
      x2 = margin + (i * step_x + step_index) * step_x;
      y2 = height - margin - static_cast<int>((data[i * step_index + step_index](2, 0) - minMax.first) * scale_y);
      cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 1);
    }

    // show the image
    cv::imshow(windowName, image);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

int main() {
  Plane plane(0, 0, 0);

  // fly the plane up to 60 seconds
  double dt = 0.01;
  double total_time = 0;
  while(total_time <= 30) {
    plane.fly(dt);
    total_time += dt;
  }

  drawLineChart(plane.ws, plane.times, 1300, 480, 10, "p, q, r (radiant)");
  drawLineChart(plane.euler_dots, plane.times, 1300, 480, 10, "change rate of euler angle (radiant)");
  drawLineChart(plane.eulers, plane.times, 1300, 480, 10, "euler angle (radiant)");
  drawLineChart(plane.vel_NEDs, plane.times, 1300, 480, 10, "velocity under NED frame (ft/sec)");
  drawLineChart(plane.pos_NEDs, plane.times, 1300, 480, 10, "position under NED frame (ft)");

  return 0;
}