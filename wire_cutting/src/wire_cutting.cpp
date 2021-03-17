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
#include <trajopt_wire_cutting_composite_profile.h>
#include <wire_cutting_problem_generator.h>

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
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>

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
  if (!env_->init<OFKTStateSolver>(urdf_xml_string, srdf_xml_string, locator))
    return false;
 
  // Create monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz_)
    monitor_->startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);

  // Get manipulator
  tesseract_kinematics::ForwardKinematics::Ptr fwd_kin;
  {  // Need to lock monitor for read
    auto lock = monitor_->lockEnvironmentRead();
    fwd_kin = monitor_->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver("manipulator");
  }

  // Create plotting tool
  ROSPlottingPtr plotter = std::make_shared<ROSPlotting>(monitor_->getSceneGraph()->getRoot());
  if (rviz_)
    plotter->waitForConnection();

  Eigen::VectorXd pos(3), size(3);
  pos << 2.4, -2.4, 0.8;
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
  joint_pos(0) = 1.57;
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
  tesseract_common::VectorIsometry3d temp_poses = loadToolPoses();
  std::vector<tesseract_common::VectorIsometry3d> tool_poses;//  = loadToolPoses("test");;
  tool_poses.push_back(temp_poses);

  plotter->waitForInput();

  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 5);
  planning_server.loadDefaultProcessPlanners();

  auto trajopt_composite_profile = std::make_shared<tesseract_planning::TrajOptWireCuttingCompositeProfile>();
  trajopt_composite_profile->collision_constraint_config.enabled = false;
  trajopt_composite_profile->collision_cost_config.enabled = true;
  trajopt_composite_profile->collision_cost_config.safety_margin = 0.080;
  trajopt_composite_profile->collision_cost_config.type = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
  trajopt_composite_profile->collision_cost_config.coeff = 1;

  auto trajopt_solver_profile = std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();
  trajopt_solver_profile->convex_solver = sco::ModelType::OSQP;
  trajopt_solver_profile->opt_info.max_iter = 200;
  trajopt_solver_profile->opt_info.min_approx_improve = 1e-3;
  trajopt_solver_profile->opt_info.min_trust_box_size = 1e-3;
  trajopt_solver_profile->opt_info.cnt_tolerance = 1e-4;

  WireCuttingProblemGenerator problem_generator(nh_);
  problem_generator.m_env_cut = env_;

  // Add profiles to Dictionary
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("DEFAULT",
                                                                                         trajopt_composite_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("DEFAULT",
                                                                                      trajopt_solver_profile);
  
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptPlanProfile>("wire_cutting", problem_generator.m_plan_cut);
  auto req1 = problem_generator.construct_request_cut(tool_poses[0]);
  //auto req2 = problem_generator.construct_request_cut(tool_poses[1]);

  
  /*tesseract_planning::CompositeInstruction naive_seed;
  {
    auto lock = monitor_->lockEnvironmentRead();
    naive_seed = tesseract_planning::generateNaiveSeed(program, *(monitor_->getEnvironment()));
  }
  request.seed = Instruction(naive_seed);*/
  // Print Diagnostics
  //  plotter->waitForInput();request.instructions.print("Program: ");

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(req1);
  planning_server.waitForAll();
  
  // Plot Process Trajectory
  if (rviz_ && plotter != nullptr && plotter->isConnected())
  {
    plotter->waitForInput();
    //ProcessPlanningFuture response2 = planning_server.run(req2);
    //planning_server.waitForAll();
    //plotter->waitForInput();
    const auto* ci = response.results->cast_const<tesseract_planning::CompositeInstruction>();
    //const auto* ci2 = response2.results->cast_const<tesseract_planning::CompositeInstruction>();
    tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(*ci, env_);
    //tesseract_common::Toolpath toolpath2 = tesseract_planning::toToolpath(*ci2, env_);
    tesseract_common::JointTrajectory trajectory = tesseract_planning::toJointTrajectory(*ci);
    //tesseract_common::JointTrajectory trajectory2 = tesseract_planning::toJointTrajectory(*ci2);

    /*JointState end1 = trajectory.back();
    JointState start2 = trajectory2.front();

    Eigen::Isometry3d initial_pose;
    Eigen::Isometry3d end_pose;
    tesseract_common::VectorIsometry3d free_targets;

    fwd_kin->calcFwdKin(initial_pose, end1.position);
    fwd_kin->calcFwdKin(end_pose, start2.position);
    free_targets.push_back(initial_pose);
    free_targets.push_back(end_pose);

  
    auto req_free = problem_generator.construct_request_freespace(end1, start2);

    plotter->waitForInput();
    ProcessPlanningFuture response_free = planning_server.run(req_free);
    planning_server.waitForAll();
    plotter->waitForInput();
    
    const auto* ci_free = response_free.results->cast_const<tesseract_planning::CompositeInstruction>();
    
    tesseract_common::Toolpath toolpath_free = tesseract_planning::toToolpath(*ci_free, env_);
    
    tesseract_common::JointTrajectory trajectory_free = tesseract_planning::toJointTrajectory(*ci_free);
    

    toolpath.insert( toolpath.end(), toolpath_free.begin(), toolpath_free.end() );
    toolpath.insert( toolpath.end(), toolpath2.begin(), toolpath2.end() );*/
    plotter->plotMarker(ToolpathMarker(toolpath));

    /*trajectory.insert( trajectory.end(), trajectory_free.begin(), trajectory_free.end());
    trajectory.insert( trajectory.end(), trajectory2.begin(), trajectory2.end());*/
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

