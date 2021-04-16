#ifndef TESSERACT_ROS_ITERATIONS_PLOTTER_H
#define TESSERACT_ROS_ITERATIONS_PLOTTER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/example.h>

namespace tesseract_ros_examples
{
/**
 * @brief Basic example leveraging trajopt and tesseract for cartesian planning
 */
class IterationsPlotter : public Example
{
public:
  IterationsPlotter(const ros::NodeHandle& nh, bool plotting, bool rviz);
  ~IterationsPlotter() override = default;
  IterationsPlotter(const IterationsPlotter&) = default;
  IterationsPlotter& operator=(const IterationsPlotter&) = default;
  IterationsPlotter(IterationsPlotter&&) = default;
  IterationsPlotter& operator=(IterationsPlotter&&) = default;

  bool run() override;

private:
  ros::NodeHandle nh_;
};

}  // namespace tesseract_ros_examples

#endif 
