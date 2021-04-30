#ifndef TESSERACT_ROS_EXAMPLES_BASIC_CARTESIAN_EXAMPLE_H
#define TESSERACT_ROS_EXAMPLES_BASIC_CARTESIAN_EXAMPLE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <ros/ros.h>
#include <wc_utils.h>
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

  bool iterationDebug = true;
  TestType test_type = TestType::cut;
  std::string test_name;
  InitMethodCut init_method_cut = InitMethodCut::lvsPlanner;
  Methodp2p method_p2p = Methodp2p::trajopt_only;
  tesseract_common::JointState p2p_start;
  tesseract_common::JointState p2p_end;
};

}  // namespace tesseract_ros_examples

#endif  // TESSERACT_ROS_EXAMPLES_BASIC_CARTESIAN_EXAMPLE_H
