#ifndef TESSERACT_ROS_EXAMPLES_BASIC_CARTESIAN_EXAMPLE_H
#define TESSERACT_ROS_EXAMPLES_BASIC_CARTESIAN_EXAMPLE_H

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
class WireCutting : public Example
{
public:
  WireCutting(const ros::NodeHandle& nh, bool plotting, bool rviz);
  ~WireCutting() override = default;
  WireCutting(const WireCutting&) = default;
  WireCutting& operator=(const WireCutting&) = default;
  WireCutting(WireCutting&&) = default;
  WireCutting& operator=(WireCutting&&) = default;

  bool run() override;

private:
  ros::NodeHandle nh_;
  tesseract_environment::Command::Ptr addBoundingBox(Eigen::VectorXd position, Eigen::VectorXd size);

  bool iterationDebug = false;
};

}  // namespace tesseract_ros_examples

#endif  // TESSERACT_ROS_EXAMPLES_BASIC_CARTESIAN_EXAMPLE_H
