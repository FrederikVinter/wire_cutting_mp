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
#include <test_wc.h>
#include <timer.h>

#include <tesseract_environment/core/utils.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_process_managers/taskflow_generators/trajopt_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/ompl_taskflow.h>
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

/** @brief test_name */
const std::string TEST_NAME = "test_name";

/** @brief test_name */
const std::string TEST_POSES = "test_poses";

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


  // Create manipulator information for program
  ManipulatorInfo mi("manipulator");
  mi.tcp = ToolCenterPoint("carbon_hotwire", false); 

  ManipulatorInfo mi_freespace("manipulator");
  mi_freespace.tcp = ToolCenterPoint("carbon_hotwire", false); 
  

 //Get manipulator
  tesseract_kinematics::ForwardKinematics::Ptr fwd_kin;
  tesseract_kinematics::InverseKinematics::Ptr inv_kin;
  {  // Need to lock monitor for read
    auto lock = monitor_->lockEnvironmentRead();
    fwd_kin = monitor_->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver("manipulator");

    // Get group names
    std::vector<std::string> group_names = monitor_->getEnvironment()->getManipulatorManager()->getGroupNames();
    std::cout << "Group names: " << "\n";
    for (size_t i = 0; i < group_names.size(); i++)
      std::cout << group_names[i] << "\n";

    std::cout << "Has group TCP: " << monitor_->getEnvironment()->getManipulatorManager()->hasGroupTCP("manipulator", "carbon_hotwire") << "\n";
    inv_kin = monitor_->getEnvironment()->getManipulatorManager()->getInvKinematicSolver("manipulator");

    std::cout << "Using inv kin solver: " << inv_kin->getSolverName() << "\n";
  }

  // Create plotting tool
  ROSPlottingPtr plotter = std::make_shared<ROSPlotting>(monitor_->getSceneGraph()->getRoot());
  if (rviz_)
    plotter->waitForConnection();

  // Eigen::VectorXd pos(3), size(3);
  // pos << 0, 2.2, -0.2;
  // size << 0.15, 0.8, 0.6;
  // Command::Ptr cmd = addBoundingBox(pos, size);
  // if (!monitor_->applyCommand(*cmd))
  //   return false;
  // if (!monitor_freespace->applyCommand(*cmd))
  //   return false;
  
  // Set the robot initial state
  std::vector<std::string> joint_names;
  joint_names.push_back("joint_1");
  joint_names.push_back("joint_2");
  joint_names.push_back("joint_3");
  joint_names.push_back("joint_4");
  joint_names.push_back("joint_5");
  joint_names.push_back("joint_6");

  Eigen::VectorXd joint_pos(6);
  joint_pos(0) = M_PI/2.0;
  joint_pos(1) = 0;
  joint_pos(2) = 0;
  joint_pos(3) = 0;
  joint_pos(4) = M_PI/2.0;
  joint_pos(5) = 0;


  env_->setState(joint_names, joint_pos);

  plotter->waitForInput(); 


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
  
  


  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 5);
  planning_server.loadDefaultProcessPlanners();

  ProcessPlanningServer planning_server_freespace(env_freespace,1,5);
  planning_server_freespace.loadDefaultProcessPlanners();

  // Create OMPL Profile
  double range = 0.01;
  double planning_time = 15.0;
  auto ompl_profile = std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>();
  auto ompl_planner_config = std::make_shared<tesseract_planning::RRTConnectConfigurator>();
  ompl_planner_config->range = range;
  ompl_profile->planning_time = planning_time;
  ompl_profile->collision_continuous = true;
  ompl_profile->planners = { ompl_planner_config, ompl_planner_config };

  // Create Descartes profile
  auto descartes_plan_profile = std::make_shared<DescartesDefaultPlanProfile<double>>();
  descartes_plan_profile->enable_collision = false;
  descartes_plan_profile->allow_collision = true;
  descartes_plan_profile->enable_edge_collision = true;
  descartes_plan_profile->num_threads = 2;
  descartes_plan_profile->target_pose_sampler = [](const Eigen::Isometry3d& tool_pose) {
    return sampleToolAxis_WC(tool_pose, M_PI/4.0, Eigen::Vector3d(0, 1, 0)); // sample around Y-axis
  };

  planning_server.getProfiles()->addProfile<tesseract_planning::DescartesPlanProfile<double>>("DESCARTES_CUT", descartes_plan_profile);

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
  
  // Load composite p2p profile
  tinyxml2::XMLDocument xml_composite_p2p;
  std::string composite_p2p_path = ros::package::getPath("wire_cutting") + "/planners/composite_p2p_profile.xml";
  xml_composite_p2p.LoadFile(composite_p2p_path.c_str());
  XMLElement* compositeElement_p2p = xml_composite_p2p.FirstChildElement("Planner")->FirstChildElement("TrajoptCompositeProfile");
  auto trajopt_composite_p2p_profile = std::make_shared<TrajOptWireCuttingCompositeProfile>(*compositeElement_p2p);


  // Solver profile
  auto trajopt_solver_profile = std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();
  trajopt_solver_profile->convex_solver = sco::ModelType::OSQP;
  trajopt_solver_profile->opt_info.max_iter = 500;
  trajopt_solver_profile->opt_info.max_merit_coeff_increases = 50;
  trajopt_solver_profile->opt_info.min_approx_improve = 1e-3;
  trajopt_solver_profile->opt_info.min_trust_box_size = 1e-3;

  trajopt_solver_profile->opt_info.log_results = iterationDebug;

  // Create a OMPL taskflow without post collision checking
  // disable continous contact check during RRT since it often fails
  const std::string ompl_planner_name = "OMPL_NO_POST_CHECK";
  tesseract_planning::OMPLTaskflowParams params;
  params.enable_post_contact_discrete_check = false;
  params.enable_post_contact_continuous_check = false;
  planning_server_freespace.registerProcessPlanner(ompl_planner_name,
                                                  std::make_unique<tesseract_planning::OMPLTaskflow>(params));

  


  // Add profiles to Dictionary                                                                             
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("DEFAULT",
                                                                                         trajopt_composite_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("DEFAULT",
                                                                                      trajopt_solver_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptPlanProfile>("wire_cutting", plan_profile_cut);

  planning_server_freespace.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("FREESPACE_TRAJOPT",
                                                                                         trajopt_composite_p2p_profile);    

  planning_server_freespace.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("FREESPACE_TRAJOPT",
                                                                                      trajopt_solver_profile);  
  planning_server_freespace.getProfiles()->addProfile<tesseract_planning::TrajOptPlanProfile>("FREESPACE_TRAJOPT", 
                                                                                      plan_profile_cut);
  planning_server_freespace.getProfiles()->addProfile<tesseract_planning::OMPLPlanProfile>("FREESPACE_OMPL", ompl_profile);
                                                                       
  

  // Load test data
  nh_.getParam(TEST_NAME, test_name);
  std::string poses_path;
  nh_.getParam(TEST_POSES, poses_path);

  plotter->waitForInput(); 
  PathData path_data = loadToolPosesCFR(poses_path);
  std::vector<tesseract_common::VectorIsometry3d> tool_poses = path_data.path;

  assert(!tool_poses.empty());

  // Eigen::VectorXd pos(3), size(3);
  // pos << 0, 1.8, -0.5;
  // size << 0.2, 0.3, 0.4;
  // if(pathData.has_bbox)
  // {
  //   std::cout << "Adding bbox with pos: " << std::endl << pathData.bbox_pos << std::endl << "size: " << std::endl << pathData.bbox_size << std::endl;
  //   Command::Ptr cmd = addBoundingBox(pathData.bbox_pos, pathData.bbox_size);
  //   if (!monitor_->applyCommand(*cmd))
  //     return false;
  //   if (!monitor_freespace->applyCommand(*cmd))
  //     return false;
  // }

  std::cout << "Path loaded" << std::endl;
  
  loadTestData(test_type, iterationDebug, test_name, init_method_cut, method_p2p, p2p_start, p2p_end, p2p_bbox, bbox_pos, bbox_size);

  std::cout << "Test data loaded" << std::endl;

  WireCuttingProblemGenerator problem_generator;
  // Generate cut requests from tool poses
  std::size_t segments = tool_poses.size();
  
    // Combine toolpath and trajectory for cut and p2p
  Toolpath combined_toolpath;
  JointTrajectory combined_trajectory;
  // Solve process plans for cuts
  if(test_type == TestType::cut || test_type == TestType::full)
  {
  std::vector<CompositeInstruction> cut_seed(segments);
  std::vector<Timer<std::milli, size_t>> timers_init(segments); // milliseconds
  for(std::size_t i = 0; i < segments; i++)
  {
    auto manipinfo = ManipulatorInfo("manipulator");
    timers_init[i].start();
    switch(init_method_cut)
    {
      case InitMethodCut::decartes : 
        cut_seed[i] = problem_generator.generate_descartes_seed(tool_poses[i], mi, env_, descartes_plan_profile);
        break;
      case InitMethodCut::lvsPlanner :
        cut_seed[i] = problem_generator.generate_conf_interpolated_seed(tool_poses[i], mi, env_);
        break;
      case InitMethodCut::naive :
        cut_seed[i] = problem_generator.generate_naive_seed(tool_poses[i], mi, env_);
        break;
    }
    timers_init[i].stop();
  }

  //const CompositeInstruction* ci_seed = cut_seed[0].cast_const<tesseract_planning::CompositeInstruction>();
  tesseract_common::Toolpath toolpath_seed = tesseract_planning::toToolpath(cut_seed[0], env_);
  std::cout << toolpath_seed.size() << std::endl;
  plotter->plotMarker(ToolpathMarker(toolpath_seed));

  plotter->waitForInput(); 


  std::vector<ProcessPlanningRequest> cut_requests(segments);

  for(std::size_t i = 0; i < segments; i++) {
    cut_requests[i] = problem_generator.construct_request_cut(tool_poses[i], mi);
    std::cout << "Constructed initial request " << std::endl;
    cut_requests[i].seed = cut_seed[i];
    std::cout << "Using generated seed as input" << std::endl;
  }

  std::vector<ProcessPlanningFuture> cut_responses(segments);
  Timer<std::milli, size_t> timer; 
  timer.start();
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
  timer.stop();

  // Cast responses to composite instructions
  std::vector<const CompositeInstruction*> cis(segments);
  for(std::size_t i = 0; i < segments; i++)
    cis[i] = cut_responses[i].results->cast_const<tesseract_planning::CompositeInstruction>();

  saveInstructionsAsXML(cis, test_name);
 
  std::vector<std::vector<std::vector<Isometry3d>>> segment_coordinates = loadInstructionsFromXML(cis, env_, test_name);

  for(std::size_t i = 0; i < segment_coordinates.size(); i++)
  {
    std::cout << "tp size: " << tool_poses[i].size() << " segment size: " << segment_coordinates[i].size() << std::endl;
    evaluate_path(tool_poses[i], segment_coordinates[i], test_name, i+1);
  }


  // Convert CIs to joint trajectories
  std::vector<JointTrajectory> trajectories(segments);
  for(std::size_t i = 0; i < segments; i++)
    trajectories[i] = tesseract_planning::toJointTrajectory(*cis[i]);

  // Generate point to point requests
  ROS_INFO("Generate point to point requests");
  std::vector<ProcessPlanningRequest> p2p_requests;
  std::vector<ProcessPlanningFuture> p2p_responses;

  for(std::size_t i = 1; i < segments; i++)
  {
    JointState last = trajectories[i-1].back();
    JointState first = trajectories[i].front();

    std::cout << "Start p2p: " << std::endl << last.position << std::endl;
    std::cout << "End p2p: " << std::endl << first.position << std::endl;

    switch(method_p2p)
    {
      case Methodp2p::trajopt_only :
        p2p_requests.push_back(problem_generator.construct_request_p2p(last, first, "FREESPACE_TRAJOPT", mi_freespace));
        break;

      case Methodp2p::rrt_only :
        p2p_requests.push_back(problem_generator.construct_request_p2p(last, first, "FREESPACE_OMPL", mi_freespace));
        break;

      case Methodp2p::rrt_trajopt :
        p2p_requests.push_back(problem_generator.construct_request_p2p(last, first, "FREESPACE_OMPL", mi_freespace));
        break;
    
    } 
  }

  Timer<std::milli, size_t> timer_p2p; // milliseconds
  timer_p2p.start();
  problem_generator.run_request_p2p(p2p_requests, plotter, planning_server_freespace, p2p_responses);
  
  timer_p2p.stop();

  ROS_INFO("Combine toolpath and trajectory for cut and p2p");

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



  } else if (test_type == TestType::p2p)
  {
    std::vector<ProcessPlanningRequest> p2p_requests;

    if(p2p_bbox)
    {
      std::cout << "Adding bbox with pos: " << std::endl << bbox_pos << std::endl << "size: " << std::endl << bbox_size << std::endl;
      Command::Ptr cmd = addBoundingBox(bbox_pos, bbox_size);
      if (!monitor_->applyCommand(*cmd))
        return false;
      if (!monitor_freespace->applyCommand(*cmd))
        return false;
    }
    
    VectorXd seed_inv(6);
    seed_inv << 0, 0, 0, 0, 0, 0;


    auto invkin = monitor_->getEnvironment()->getManipulatorManager()->getInvKinematicSolver("manipulator");
    
    Eigen::Isometry3d tcp = env_->findTCP(mi);

    Isometry3d p2p_start_local = p2p_start * tcp.inverse();
    Isometry3d p2p_end_local = p2p_end * tcp.inverse();

    std::cout << "Start Rotation: " << std::endl << p2p_start.rotation() << std::endl;
    
    std::array<Eigen::VectorXd, 2> sol = getClosestJointSolution(p2p_start_local, p2p_end_local, invkin, invkin, seed_inv);

    JointState s, e;
    s.joint_names = joint_names;
    e.joint_names = joint_names;

    for(std::size_t i = 0; i < s.joint_names.size(); i++)
    {
      std::cout << s.joint_names[i] << std::endl;
    }
    s.position = sol[0];
    e.position = sol[1];


    switch(method_p2p)
    {
      case Methodp2p::trajopt_only :
        p2p_requests.push_back(problem_generator.construct_request_p2p(s, e, "FREESPACE_TRAJOPT", mi_freespace));
        break;

      case Methodp2p::rrt_only :
        p2p_requests.push_back(problem_generator.construct_request_p2p(s, e, "FREESPACE_OMPL", mi_freespace));
        break;

      case Methodp2p::rrt_trajopt :
        p2p_requests.push_back(problem_generator.construct_request_p2p(s, e, "FREESPACE_OMPL", mi_freespace));
        break;
    
    }   

    std::vector<ProcessPlanningFuture> p2p_responses;

    if (problem_generator.run_request_p2p(p2p_requests, plotter, planning_server_freespace, p2p_responses)) {
        // Trajopt after seed
        if(method_p2p == Methodp2p::rrt_trajopt || method_p2p == Methodp2p::naive_trajopt ) {
          p2p_requests.clear();
          ProcessPlanningRequest request = problem_generator.construct_request_p2p_cart(p2p_start, p2p_end, "FREESPACE_TRAJOPT", mi_freespace);
          request.seed = *p2p_responses[0].results.get();
          p2p_requests.push_back(request);

          p2p_responses.clear();
          if(problem_generator.run_request_p2p(p2p_requests, plotter, planning_server_freespace, p2p_responses)) {
            ROS_INFO("Succeeded with seed");
          }
        }
      }

    if(iterationDebug) {  
      plotter->waitForInput();   
      ROS_INFO("Plotting path iterations");  
      plotIterations(env_, plotter, joint_names, "/tmp/trajopt_vars.log");
    }
    const CompositeInstruction* ci;

    ci = p2p_responses[0].results->cast_const<tesseract_planning::CompositeInstruction>();
  
    combined_toolpath = tesseract_planning::toToolpath(*ci, env_);
    combined_trajectory = tesseract_planning::toJointTrajectory(*ci);
  }



  // Plot Process Trajectory
  if (rviz_ && plotter != nullptr && plotter->isConnected())
  {
    plotter->waitForInput();

    plotter->plotMarker(ToolpathMarker(combined_toolpath));
    plotter->plotTrajectory(combined_trajectory, env_->getStateSolver());
  }


  /*tinyxml2::XMLDocument xmlDoc;

  XMLNode * pRoot = xmlDoc.NewElement("Instructions");    // Creat root element
  xmlDoc.InsertFirstChild(pRoot);                 // Insert element

  //XMLElement* pResults = trajopt_composite_profile->toXML(xmlDoc);
  XMLElement * pResults = cut_responses[0].results->toXML(xmlDoc);
  pRoot->InsertEndChild(pResults);                // insert element as child

  const char* fileName = "/home/frederik/ws_tesseract_wirecut/trajopt_results.xml";
  tinyxml2::XMLError eResult = xmlDoc.SaveFile(fileName);*/



  ROS_INFO("Final trajectory is collision free");
  return true;
}

} // Namespace 

