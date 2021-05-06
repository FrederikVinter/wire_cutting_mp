#ifndef TEST_WC_H
#define TEST_WC_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/example.h>

void evaluate_path(tesseract_common::VectorIsometry3d tool_poses, 
                   std::vector<std::vector<Eigen::Isometry3d>> path);



#endif 
