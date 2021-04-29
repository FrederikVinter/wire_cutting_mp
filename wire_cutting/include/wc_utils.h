#ifndef WC_UTILS_H
#define WC_UTILS_H

#pragma once

#include <tesseract_common/macros.h>
#include <tesseract_common/types.h>
#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <tesseract_rosutils/plotting.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tinyxml2.h>

struct PathData
{
  PathData() {bbox_pos.resize((3)); bbox_size.resize(3); };
  std::vector<tesseract_common::VectorIsometry3d> path;
  Eigen::VectorXd bbox_pos, bbox_size;
  bool has_bbox = false;
};

std::vector<tesseract_common::VectorIsometry3d> loadToolPosesFromPrg(const std::string& file);
tesseract_common::VectorIsometry3d loadToolPoses();

PathData loadToolPosesCFR(std::string file);

std::vector<std::vector<Eigen::VectorXd>> loadOptimizationResults();
void plotIterations(const tesseract_environment::Environment::Ptr& env, const tesseract_rosutils::ROSPlottingPtr& plotter);

tesseract_common::VectorIsometry3d sampleToolAxis_WC(const Eigen::Isometry3d& tool_pose,
                                                    double resolution,
                                                    const Eigen::Vector3d& axis);

trajopt::TermInfo::Ptr createVelocityTermInfo(double max_displacement,
                                                int start_index,
                                                int end_index,
                                                const std::string& link,
                                                trajopt::TermType type);

#endif