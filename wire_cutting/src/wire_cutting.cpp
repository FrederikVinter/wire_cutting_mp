/**
 * @file basic_cartesian_plan.cpp
 * @brief Basic cartesian example implementation
 *
 * @author Levi Armstrong
 * @date July 22, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <octomap_ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/wire_cutting.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_process_managers/taskflow_generators/trajopt_taskflow.h>
#include <tesseract_process_managers/core/process_planning_server.h>
#include <tesseract_process_managers/core/default_process_planners.h>
#include <tesseract_planning_server/tesseract_planning_server.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;
using namespace tesseract_visualization;

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

namespace tesseract_ros_examples
{

WireCutting::WireCutting(const ros::NodeHandle& nh, bool plotting, bool rviz)
  : Example(plotting, rviz), nh_(nh)
{
}

bool WireCutting::run()
{
  using tesseract_planning::CartesianWaypoint;
  using tesseract_planning::CompositeInstruction;
  using tesseract_planning::CompositeInstructionOrder;
  using tesseract_planning::Instruction;
  using tesseract_planning::ManipulatorInfo;
  using tesseract_planning::PlanInstruction;
  using tesseract_planning::PlanInstructionType;
  using tesseract_planning::ProcessPlanningFuture;
  using tesseract_planning::ProcessPlanningRequest;
  using tesseract_planning::ProcessPlanningServer;
  using tesseract_planning::StateWaypoint;
  using tesseract_planning::Waypoint;
  using tesseract_planning_server::ROSProcessEnvironmentCache;

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env_->init<OFKTStateSolver>(urdf_xml_string, srdf_xml_string, locator))
    return false;

  // Create monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz_)
    monitor_->startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);

  // Create octomap and add it to the local environment
  //Command::Ptr cmd = addPointCloud();
  //if (!monitor_->applyCommand(*cmd))
  //  return false;

  // Create plotting tool
  ROSPlottingPtr plotter = std::make_shared<ROSPlotting>(monitor_->getSceneGraph()->getRoot());
  if (rviz_)
    plotter->waitForConnection();
  
  plotter->waitForInput();

  // Set the robot initial state
  std::vector<std::string> joint_names;
  joint_names.push_back("joint_1");
  joint_names.push_back("joint_2");
  joint_names.push_back("joint_3");
  joint_names.push_back("joint_4");
  joint_names.push_back("joint_5");
  joint_names.push_back("joint_6");

  Eigen::VectorXd joint_pos(6);
  joint_pos(0) = -0.4;
  joint_pos(1) = 0.2762;
  joint_pos(2) = 0.0;
  joint_pos(3) = -1.3348;
  joint_pos(4) = 0.0;
  joint_pos(5) = 1.4959;

  env_->setState(joint_names, joint_pos);

  plotter->waitForInput();

  // Create Program
  CompositeInstruction program("cartesian_program", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator"));

  // Start Joint Position for the program
  Waypoint wp0 = StateWaypoint(joint_names, joint_pos);
  PlanInstruction start_instruction(wp0, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  // Create cartesian waypoint
  Waypoint wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1.0, -0.2, 0.62) *
                                   Eigen::Quaterniond(0, 0, 1.0, 0));

  Waypoint wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1.0, 0.3, 0.62) *
                                   Eigen::Quaterniond(0, 0, 1.0, 0));

  // Plan freespace from start
  PlanInstruction plan_f0(wp1, PlanInstructionType::FREESPACE, "freespace_profile");
  plan_f0.setDescription("from_start_plan");

  // Plan linear move
  PlanInstruction plan_c0(wp2, PlanInstructionType::LINEAR, "RASTER");

  // Plan freespace to end
  PlanInstruction plan_f1(wp0, PlanInstructionType::FREESPACE, "freespace_profile");
  plan_f1.setDescription("to_end_plan");

  // Add Instructions to program
  program.push_back(plan_f0);
  program.push_back(plan_c0);
  program.push_back(plan_f1);

  ROS_INFO("basic cartesian plan example");

  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 5);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  request.instructions = Instruction(program);

  // Print Diagnostics
  //request.instructions.print("Program: ");

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Plot Process Trajectory
  if (rviz_ && plotter != nullptr && plotter->isConnected())
  {
    plotter->waitForInput();
    const auto* ci = response.results->cast_const<tesseract_planning::CompositeInstruction>();
    tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(*ci, env_);
    tesseract_common::JointTrajectory trajectory = tesseract_planning::toJointTrajectory(*ci);
    plotter->plotMarker(ToolpathMarker(toolpath));
    plotter->plotTrajectory(trajectory, env_->getStateSolver());
  }

  ROS_INFO("Final trajectory is collision free");
  return true;
}

}  // namespace tesseract_ros_examples
