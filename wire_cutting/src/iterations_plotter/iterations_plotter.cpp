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

#include <iterations_plotter.h>
#include <wc_utils.h>

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

/** @brief test_name */
const std::string TEST_NAME = "test_name";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

namespace tesseract_ros_examples
{

IterationsPlotter::IterationsPlotter(const ros::NodeHandle& nh, bool plotting, bool rviz)
  : Example(plotting, rviz), nh_(nh)
{
}

bool IterationsPlotter::run()
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


  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env_->init<OFKTStateSolver>(urdf_xml_string, srdf_xml_string, locator))
    return false;
  ROS_INFO("Initialized cut env");


  // Create monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz_)
    monitor_->startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);

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


  env_->setState(joint_names, joint_pos);

  
  plotter->waitForInput();

  

 
  std::string temp;
  nh_.getParam(TEST_NAME, temp);
  std::cout << temp << std::endl;
  std::string filepath = ros::package::getPath("wire_cutting") + "/results/" + temp + "/trajopt_vars.log";
  std::cout << "PATH: " << filepath << std::endl;

  plotter->waitForInput();   
  ROS_INFO("Plotting path iterations");  
  plotIterations(env_, plotter, joint_names, filepath);

 
  // Plot Process Trajectory
  /*if (rviz_ && plotter != nullptr && plotter->isConnected())
  {
    plotter->waitForInput();

    plotter->plotMarker(ToolpathMarker(combined_toolpath));
    plotter->plotTrajectory(combined_trajectory, env_->getStateSolver());
  }*/



  return true;
}

}  // namespace tesseract_ros_examples
