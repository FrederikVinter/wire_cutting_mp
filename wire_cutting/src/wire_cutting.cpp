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

#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_motion_planners/descartes/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>

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
  ROS_INFO("Initialized cut env");

  std::cout << "Joints, Active: " << env_->getActiveJointNames().size() << " All: " << env_->getJointNames().size() << std::endl;

  auto env_freespace = std::make_shared<tesseract_environment::Environment>();
  if (!env_freespace->init<OFKTStateSolver>(urdf_xml_string_freespace, srdf_xml_string_freespace, locator))
  {
    ROS_INFO("Error initializing freespace environment");
    return false;
  } 
  ROS_INFO("Intialized freespace env");

  // Create monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz_)
    monitor_->startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);
    
  auto monitor_freespace = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_freespace, "Freespace");

 // Get manipulator

  // Create plotting tool
  ROSPlottingPtr plotter = std::make_shared<ROSPlotting>(monitor_->getSceneGraph()->getRoot());
  if (rviz_)
    plotter->waitForConnection();


  
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

  //Between alligners freespace path
  PathData pathData = loadToolPosesCFR("HardProb.txt");
  std::vector<tesseract_common::VectorIsometry3d> tool_poses;
  tool_poses = pathData.path;


  /* Freespace a void bbox 
  tesseract_common::VectorIsometry3d temp_poses = loadToolPoses();
  PathData pathData;
  std::vector<tesseract_common::VectorIsometry3d> tool_poses; 
  tool_poses.push_back(temp_poses);
  tool_poses.push_back(temp_poses);
  pathData.path = tool_poses;
  pathData.bbox_pos << 0, 2.2, -0.8;
  pathData.bbox_size << 0.1, 1.5, 1;
  pathData.has_bbox = true;*/
  
  
  assert(!tool_poses.empty());

  /*Eigen::VectorXd pos(3), size(3);
  pos << 0, 1.8, -0.5;
  size << 0.2, 0.3, 0.4;*/
  if(pathData.has_bbox)
  {
    std::cout << "Adding bbox with pos: " << std::endl << pathData.bbox_pos << std::endl << "size: " << std::endl << pathData.bbox_size << std::endl;
    Command::Ptr cmd = addBoundingBox(pathData.bbox_pos, pathData.bbox_size);
    if (!monitor_->applyCommand(*cmd))
      return false;
    if (!monitor_freespace->applyCommand(*cmd))
      return false;
  }

  plotter->waitForInput();

  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 5);
  planning_server.loadDefaultProcessPlanners();

  ProcessPlanningServer planning_server_freespace(env_freespace,1,5);
  planning_server_freespace.loadDefaultProcessPlanners();

  // Create Descartes profile
  /*auto descartes_plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();
  descartes_plan_profile->enable_collision = false;
  descartes_plan_profile->allow_collision = false;
  descartes_plan_profile->enable_edge_collision = true;
  descartes_plan_profile->debug = true;
  descartes_plan_profile->target_pose_sampler = [](const Eigen::Isometry3d& tool_pose) {
    // return tesseract_planning::sampleToolAxis(tool_pose, 60 * M_PI * 180.0, Eigen::Vector3d(0, 1, 0)); // sample around Y-axis
    return tesseract_planning::sampleToolYAxis(tool_pose, M_PI);
  };*/

  //planning_server.getProfiles()->addProfile<tesseract_planning::DescartesDefaultPlanProfileD>("DESCARTES", descartes_plan_profile);

  // Load plan profile
  tinyxml2::XMLDocument xml_plan_cut;
  std::string plan_cut_path = ros::package::getPath("wire_cutting") + "/planners/plan_cut_profile.xml";
  xml_plan_cut.LoadFile(plan_cut_path.c_str());
  XMLElement* planElement = xml_plan_cut.FirstChildElement("Planner")->FirstChildElement("TrajoptPlanProfile");
  
  auto plan_profile_cut = std::make_shared<TrajOptWireCuttingPlanProfile>(*planElement);
  std::cout << "Plan costs: " << std::endl << plan_profile_cut->cart_coeff_cost << std::endl;
  std::cout << "Plan cnt: " << std::endl << plan_profile_cut->cart_coeff_cnt << std::endl;
  plan_profile_cut->addFourBarLinkageConstraints();


  // Load composite profile
  tinyxml2::XMLDocument xml_composite_cut;
  std::string composite_cut_path = ros::package::getPath("wire_cutting") + "/planners/composite_cut_profile.xml";
  xml_composite_cut.LoadFile(composite_cut_path.c_str());
  XMLElement* compositeElement = xml_composite_cut.FirstChildElement("Planner")->FirstChildElement("TrajoptCompositeProfile");
  auto trajopt_composite_profile = std::make_shared<TrajOptWireCuttingCompositeProfile>(*compositeElement);
  trajopt_composite_profile->constrain_velocity = false;

  // Load composite p2p profile
  tinyxml2::XMLDocument xml_composite_p2p;
  std::string composite_p2p_path = ros::package::getPath("wire_cutting") + "/planners/composite_p2p_profile.xml";
  xml_composite_p2p.LoadFile(composite_p2p_path.c_str());
  XMLElement* compositeElement_p2p = xml_composite_p2p.FirstChildElement("Planner")->FirstChildElement("TrajoptCompositeProfile");
  auto trajopt_composite_p2p_profile = std::make_shared<TrajOptWireCuttingCompositeProfile>(*compositeElement_p2p);
  trajopt_composite_p2p_profile->constrain_velocity = false;

  // Solver profile
  auto trajopt_solver_profile = std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();
  trajopt_solver_profile->convex_solver = sco::ModelType::OSQP;
  trajopt_solver_profile->opt_info.max_iter = 500;
  trajopt_solver_profile->opt_info.min_approx_improve = 1e-3;
  trajopt_solver_profile->opt_info.min_trust_box_size = 1e-3;

  //trajopt_solver_profile->opt_info.cnt_tolerance = 1e-4;
  trajopt_solver_profile->opt_info.log_results = iterationDebug;
  


  // Add profiles to Dictionary
  planning_server_freespace.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("FREESPACE",
                                                                                      trajopt_solver_profile);
  planning_server_freespace.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("FREESPACE",
                                                                                         trajopt_composite_p2p_profile);                                                                                    
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("DEFAULT",
                                                                                         trajopt_composite_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("DEFAULT",
                                                                                      trajopt_solver_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptPlanProfile>("wire_cutting", plan_profile_cut);
  
  plotter->waitForInput();

  WireCuttingProblemGenerator problem_generator(monitor_);
  // Generate cut requests from tool poses
  std::size_t segments = tool_poses.size();
  std::vector<ProcessPlanningRequest> cut_requests(segments);
  for(std::size_t i = 0; i < segments; i++)
  {
    cut_requests[i] = problem_generator.construct_request_cut(tool_poses[i], env_);
  }


  // Solve process plans for cuts
  std::vector<ProcessPlanningFuture> cut_responses(segments);
  for(std::size_t i = 0; i < segments; i++)
  {
    cut_responses[i] = planning_server.run(cut_requests[i]);
    if(iterationDebug) {  
      planning_server.waitForAll(); 
      plotter->waitForInput();   
      ROS_INFO("Plotting path iterations");  
      plotIterations(env_, plotter, joint_names, "/tmp/trajopt_vars.log");
    }
  }
  planning_server.waitForAll();

  // // Plot optimization iterations
  /*if(iterationDebug) {    
     ROS_INFO("Plotting path iterations");  
     plotIterations(env_, plotter);
  }*/

  // Cast responses to composite instructions
  std::vector<const CompositeInstruction*> cis(segments);
  for(std::size_t i = 0; i < segments; i++)
    cis[i] = cut_responses[i].results->cast_const<tesseract_planning::CompositeInstruction>();


  // Convert CIs to joint trajectories
  std::vector<JointTrajectory> trajectories(segments);
  for(std::size_t i = 0; i < segments; i++)
    trajectories[i] = tesseract_planning::toJointTrajectory(*cis[i]);

  plotter->waitForInput();
  ROS_INFO("Generate point to point requests");
  // Generate point to point requests
  std::vector<ProcessPlanningRequest> p2p_requests;
  for(std::size_t i = 1; i < segments; i++)
  {
    JointState last = trajectories[i-1].back();
    JointState first = trajectories[i].front();

    p2p_requests.push_back(problem_generator.construct_request_p2p(last, first));
  }

  plotter->waitForInput();
  ROS_INFO("Solve process plans for point to point");
  // Solve process plans for point to point
  std::size_t p2p_moves = p2p_requests.size();
  std::vector<ProcessPlanningFuture> p2p_responses(p2p_moves);
  for(std::size_t i = 0; i < p2p_moves; i++)
  {
    p2p_responses[i] = planning_server_freespace.run(p2p_requests[i]);  
    //planning_server_freespace.waitForAll(); 
    //plotter->waitForInput();  
    if(iterationDebug) { 
      planning_server_freespace.waitForAll();  
      plotter->waitForInput(); 
      ROS_INFO("Plotting path iterations");  
      plotIterations(env_, plotter, joint_names, "/tmp/trajopt_vars.log");
    }
  }
  planning_server_freespace.waitForAll();



  plotter->waitForInput();
  ROS_INFO("Combine toolpath and trajectory for cut and p2p");
  // Combine toolpath and trajectory for cut and p2p
  Toolpath combined_toolpath;
  JointTrajectory combined_trajectory;

  assert(cut_responses.size() == p2p_responses.size()+1);
  for(std::size_t i = 0; i < cut_responses.size()+p2p_responses.size(); i++)
  {
    const CompositeInstruction* ci;
    if(i%2 == 0)
      ci = cut_responses[i/2].results->cast_const<tesseract_planning::CompositeInstruction>();
    else
      ci = p2p_responses[i/2].results->cast_const<tesseract_planning::CompositeInstruction>();
  
    tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(*ci, env_);
    tesseract_common::JointTrajectory trajectory = tesseract_planning::toJointTrajectory(*ci);

    combined_toolpath.insert(combined_toolpath.end(), toolpath.begin(), toolpath.end());
    combined_trajectory.insert(combined_trajectory.end(), trajectory.begin(), trajectory.end());
  }

  // Plot Process Trajectory
  if (rviz_ && plotter != nullptr && plotter->isConnected())
  {
    plotter->waitForInput();

    plotter->plotMarker(ToolpathMarker(combined_toolpath));
    plotter->plotTrajectory(combined_trajectory, env_->getStateSolver());
  }


  tinyxml2::XMLDocument xmlDoc;

  XMLNode * pRoot = xmlDoc.NewElement("Instructions");    // Creat root element
  xmlDoc.InsertFirstChild(pRoot);                 // Insert element

  //XMLElement* pResults = trajopt_composite_profile->toXML(xmlDoc);
  XMLElement * pResults = cut_responses[0].results->toXML(xmlDoc);
  pRoot->InsertEndChild(pResults);                // insert element as child

  const char* fileName = "/home/frederik/ws_tesseract_wirecut/trajopt_results.xml";
  tinyxml2::XMLError eResult = xmlDoc.SaveFile(fileName);



  ROS_INFO("Final trajectory is collision free");
  return true;
}

} // Namespace 

