#ifndef WIRE_CUTTING_PROLEM_GENERATOR_H
#define WIRE_CUTTING_PROLEM_GENERATOR_H

#pragma once

#include <tesseract_common/macros.h>
#include <tesseract_common/types.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_process_managers/core/process_planning_server.h>
#include <ros/ros.h>
#include <tesseract_monitoring/environment_monitor.h>

#include <tesseract_motion_planners/interface_utils.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <trajopt_wire_cutting_composite_profile.h>

#include <tesseract_visualization/markers/toolpath_marker.h>
#include <tesseract_rosutils/plotting.h>

#include <four_bar_linkage_constraint.h>
#include <trajopt_wire_cutting_plan_profile.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_environment/core/utils.h>
#include <string>
#include <vector>

#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_motion_planners/descartes/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>

#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/problem_generators/default_problem_generator.h>

TESSERACT_COMMON_IGNORE_WARNINGS_POP

using namespace tesseract_common;
using namespace tesseract_environment;
using namespace tesseract_planning;


class WireCuttingProblemGenerator
{
public:
    WireCuttingProblemGenerator();
    
    ProcessPlanningRequest construct_request_cut(const VectorIsometry3d& tool_poses, ManipulatorInfo& mi);

    CompositeInstruction generate_descartes_seed(const VectorIsometry3d& tool_poses,
                                                 ManipulatorInfo& mi, 
                                                 const tesseract_environment::Environment::Ptr& env,
                                                 std::shared_ptr<DescartesDefaultPlanProfile<double>> descartes_plan_profile);

    CompositeInstruction generate_conf_interpolated_seed(const VectorIsometry3d& tool_poses,
                                                         ManipulatorInfo& mi, 
                                                         const tesseract_environment::Environment::Ptr& env);

    CompositeInstruction generate_naive_seed(const VectorIsometry3d& tool_poses,
                                             ManipulatorInfo& mi, 
                                             const tesseract_environment::Environment::Ptr& env);

    CompositeInstruction generate_ompl_seed(const VectorIsometry3d& tool_poses,
                                            ManipulatorInfo& mi, 
                                            const tesseract_environment::Environment::Ptr& env,
                                            std::shared_ptr<tesseract_planning::OMPLDefaultPlanProfile> ompl_plan_profile);


    ProcessPlanningRequest construct_request_p2p(const JointState& start, 
                                                 const JointState& end, 
                                                 const std::string& planner_name, 
                                                 ManipulatorInfo& mi, 
                                                 bool post_collision_check);

    ProcessPlanningRequest construct_request_p2p_cart(const Isometry3d& start,
                                                                          const Isometry3d& end, 
                                                                          const std::string& planner_name, 
                                                                          ManipulatorInfo& mi,
                                                                          bool post_collision_check);

    bool run_request_p2p(std::vector<ProcessPlanningRequest>& p2p_requests, 
                        const tesseract_rosutils::ROSPlottingPtr& plotter,
                        ProcessPlanningServer& planning_server_freespace,
                        std::vector<ProcessPlanningFuture>& p2p_responses);

    TrajOptWireCuttingPlanProfile::Ptr m_plan_cut;
    Environment::Ptr m_env_cut;
    Environment::Ptr m_env_free;
private:  
    TrajOptPlanProfile::Ptr m_plan_free;
};

#endif