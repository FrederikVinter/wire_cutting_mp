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

#include <wire_cutting.h>
#include <trajopt_wire_cutting_plan_profile.h>
#include <four_bar_linkage_constraint.h>
#include <wc_utils.h>

#include <tesseract_environment/core/utils.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_process_managers/taskflow_generators/trajopt_taskflow.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_process_managers/core/process_planning_server.h>
#include <tesseract_process_managers/core/default_process_planners.h>
#include <tesseract_planning_server/tesseract_planning_server.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

#include <tinyxml2.h>

using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;
using namespace tesseract_visualization;
using namespace tinyxml2;

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string ROBOT_DESCRIPTION_FREESPACE_PARAM = "robot_description_freespace";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";
const std::string ROBOT_SEMANTIC_FREESPACE_PARAM = "robot_description_freespace_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

namespace tesseract_ros_examples
{

tesseract_environment::Command::Ptr WireCutting::addBoundingBox(Eigen::VectorXd position, Eigen::VectorXd size)
{
  // Add sphere to environment
  auto link_bounding_box = std::make_shared<Link>("bounding_box");

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = position;
  visual->geometry = std::make_shared<tesseract_geometry::Box>(size[0], size[1], size[2]);
  link_bounding_box->visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_bounding_box->collision.push_back(collision);

  auto joint_bounding_box = std::make_shared<Joint>("joint_bounding_box_attached");
  joint_bounding_box->parent_link_name = "base_link";
  joint_bounding_box->child_link_name = link_bounding_box->getName();
  joint_bounding_box->type = JointType::FIXED;

  return std::make_shared<tesseract_environment::AddCommand>(link_bounding_box, joint_bounding_box);
}

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

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string, urdf_xml_string_freespace, srdf_xml_string_freespace;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
  nh_.getParam(ROBOT_DESCRIPTION_FREESPACE_PARAM, urdf_xml_string_freespace);
  nh_.getParam(ROBOT_SEMANTIC_FREESPACE_PARAM, srdf_xml_string_freespace);


  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env_->init<OFKTStateSolver>(urdf_xml_string_freespace, srdf_xml_string_freespace, locator))
    return false; 
  
  // Create monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz_)
    monitor_->startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);

  // Create plotting tool
  ROSPlottingPtr plotter = std::make_shared<ROSPlotting>(monitor_->getSceneGraph()->getRoot());
  if (rviz_)
    plotter->waitForConnection();

  Eigen::VectorXd pos(3), size(3);
  pos << 2.0, 0, 0.8;
  size << 0.2, 0.3, 0.4;
  Command::Ptr cmd = addBoundingBox(pos, size);
  if (!monitor_->applyCommand(*cmd))
    return false;
  
  // Set the robot initial state
  std::vector<std::string> joint_names;
  joint_names.push_back("joint_1");
  joint_names.push_back("joint_2");
  joint_names.push_back("joint_3");
  joint_names.push_back("joint_4");
  joint_names.push_back("joint_5");
  joint_names.push_back("joint_6");

  Eigen::VectorXd joint_pos(6);
  joint_pos(0) = 0;
  joint_pos(1) = 0;
  joint_pos(2) = 0;
  joint_pos(3) = 0;
  joint_pos(4) = 1.57;
  joint_pos(5) = 0;

  /*SceneGraph::ConstPtr g = env_freespace.get()->getSceneGraph(); 
  g->saveDOT("/home/frederik/ws_tesseract_wirecut/scenegraph.dot");
  auto mimic = g->getJoint("joint_p")->mimic;
  std::string s = mimic->joint_name;
  ROS_INFO(s.c_str());
  SceneGraph::Ptr g1 = g->clone();*/

  env_->setState(joint_names, joint_pos);

  std::vector<tesseract_common::VectorIsometry3d> tool_poses = loadToolPosesFromPrg("test");

  plotter->waitForInput();

  // Create Program
  CompositeInstruction program("DEFAULT", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator"));

  // Create cartesian waypoint
  Waypoint wp = CartesianWaypoint(tool_poses[0][0]);
  PlanInstruction plan_instruction(wp, PlanInstructionType::START, "wire_cutting");
  plan_instruction.setDescription("from_start_plan");
  program.setStartInstruction(plan_instruction);

  for (std::size_t i = 0; i < tool_poses.size(); ++i)
  {
    if(i != 0)
      Waypoint wp = CartesianWaypoint(tool_poses[i][0]);
      PlanInstruction plan_instruction(wp, PlanInstructionType::FREESPACE, "FREESPACE");
      program.push_back(plan_instruction);
      plan_instruction.setDescription("waypoint_" + std::to_string(i) + ",0"); 
    for (std::size_t j = 1; j < tool_poses[i].size(); j++)
    {
      Waypoint wp = CartesianWaypoint(tool_poses[i][j]);
      PlanInstruction plan_instruction(wp, PlanInstructionType::LINEAR, "wire_cutting");
      plan_instruction.setDescription("waypoint_" + std::to_string(i) + "," + std::to_string(j)); 
      program.push_back(plan_instruction);
    }
  }

  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 5);
  planning_server.loadDefaultProcessPlanners();

  // Create TrajOpt Profile
  auto trajopt_plan_profile = std::make_shared<TrajOptWireCuttingPlanProfile>();
  FourBarLinkageConstraint cnt1(env_);
  JointTwoLimitsConstraint cnt2(env_);
  JointThreeLimitsConstraint cnt3(env_);
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> temp_function1 = cnt1;
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> temp_function2 = cnt2;
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> temp_function3 = cnt3;
  sco::VectorOfVector::func temp_1 = temp_function1;
  sco::VectorOfVector::func temp_2 = temp_function2;
  sco::VectorOfVector::func temp_3 = temp_function3;

  sco::ConstraintType a = sco::ConstraintType::EQ;
  Eigen::VectorXd error_coeff(1);
  error_coeff << 0.5;

  std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd> temp_tuple1(temp_1,nullptr,a,error_coeff);
  std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd> temp_tuple2(temp_2,nullptr,a,error_coeff);
  std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd> temp_tuple3(temp_3,nullptr,a,error_coeff);
  std::vector<std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd>>
  constraint_error_functions;

  constraint_error_functions.push_back(temp_tuple1);
  constraint_error_functions.push_back(temp_tuple2);
  constraint_error_functions.push_back(temp_tuple3);

  trajopt_plan_profile->constraint_error_functions = constraint_error_functions;

  auto trajopt_composite_profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
  trajopt_composite_profile->collision_constraint_config.enabled = false;
  trajopt_composite_profile->collision_cost_config.enabled = true;
  trajopt_composite_profile->collision_cost_config.safety_margin = 0.050;
  trajopt_composite_profile->collision_cost_config.type = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
  trajopt_composite_profile->collision_cost_config.coeff = 1;

  auto trajopt_solver_profile = std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();
  trajopt_solver_profile->convex_solver = sco::ModelType::OSQP;
  trajopt_solver_profile->opt_info.max_iter = 200;
  trajopt_solver_profile->opt_info.min_approx_improve = 1e-3;
  trajopt_solver_profile->opt_info.min_trust_box_size = 1e-3;
  trajopt_solver_profile->opt_info.cnt_tolerance = 1e-4;

  // Add profile to Dictionary
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptPlanProfile>("wire_cutting", trajopt_plan_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("DEFAULT",
                                                                                         trajopt_composite_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("DEFAULT",
                                                                                      trajopt_solver_profile);

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  request.instructions = Instruction(program);
  
  /*tesseract_planning::CompositeInstruction naive_seed;
  {
    auto lock = monitor_->lockEnvironmentRead();
    naive_seed = tesseract_planning::generateNaiveSeed(program, *(monitor_->getEnvironment()));
  }
  request.seed = Instruction(naive_seed);*/
  // Print Diagnostics
  //  plotter->waitForInput();request.instructions.print("Program: ");

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

  tinyxml2::XMLDocument xmlDoc;

  XMLNode * pRoot = xmlDoc.NewElement("Instructions");    // Creat root element
  xmlDoc.InsertFirstChild(pRoot);                 // Insert element

  XMLElement * pResults = response.results->toXML(xmlDoc);
  pRoot->InsertEndChild(pResults);                // insert element as child

  const char* fileName = "/home/frederik/ws_tesseract_wirecut/trajopt_results.xml";
  tinyxml2::XMLError eResult = xmlDoc.SaveFile(fileName);

  ROS_INFO("Final trajectory is collision free");
  return true;
}

} // Namespace 

