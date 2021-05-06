#ifndef WC_UTILS_H
#define WC_UTILS_H

#pragma once

#include <tesseract_common/macros.h>
#include <tesseract_common/types.h>
#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <tesseract_rosutils/plotting.h>
#include <term_info_wc.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tinyxml2.h>

enum TestType{
  cut,
  p2p,
  full
};

enum InitMethodCut{
  naive,
  lvsPlanner,
  decartes
};

enum Methodp2p{
  trajopt_only,
  rrt_only,
  rrt_trajopt,
  naive_trajopt
};

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

std::vector<std::vector<Eigen::VectorXd>> loadOptimizationResults(std::string path);
void plotIterations(const tesseract_environment::Environment::Ptr& env, const tesseract_rosutils::ROSPlottingPtr& plotter, const std::vector<std::string>& joint_names, std::string path);

tesseract_common::VectorIsometry3d sampleToolAxis_WC(const Eigen::Isometry3d& tool_pose,
                                                    double resolution,
                                                    const Eigen::Vector3d& axis);

trajopt::TermInfo::Ptr createVelocityTermInfo(double max_displacement,
                                                int start_index,
                                                int end_index,
                                                const std::string& link,
                                                trajopt::TermType type,
                                                const Eigen::VectorXd& coeff,
                                                sco::PenaltyType penalty_type);

trajopt::TermInfo::Ptr createRotationalVelocityTermInfo(double max_displacement,
                                                //Eigen::Vector3d rot_coeffs,
                                                int start_index,
                                                int end_index,
                                                const std::string& link,
                                                trajopt::TermType type,
                                                const Eigen::VectorXd& coeff,
                                                sco::PenaltyType penalty_type);

trajopt::TermInfo::Ptr createCartesianWaypointTermInfoWC(const Eigen::Isometry3d& c_wp,
                                                       int index,
                                                       std::string working_frame,
                                                       Eigen::Isometry3d tcp,
                                                       const Eigen::VectorXd& coeffs,
                                                       std::string link,
                                                       trajopt::TermType type,
                                                       sco::PenaltyType penalty_type);

void loadTestData(TestType &test_type,
                bool &show_iterations,
                const std::string &test_name, 
                InitMethodCut &init_method_cut, 
                Methodp2p &method_p2p, 
                Isometry3d &p2p_start, 
                Isometry3d &p2p_end,
                bool &bbox,
                VectorXd &bbox_pos,
                VectorXd &bbox_size);

#endif