#include <iterations_plotter.h>

using namespace tesseract_ros_examples;

int main(int argc, char** argv)
{
  std::cout << argc << std::endl;
  std::cout << argv << std::endl;
  ros::init(argc, argv, "iterations_plotter_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  bool plotting = true;
  bool rviz = true;

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);

  IterationsPlotter example(nh, plotting, rviz);
  example.run();
}
