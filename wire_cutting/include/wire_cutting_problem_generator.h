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

#include <four_bar_linkage_constraint.h>
#include <trajopt_wire_cutting_plan_profile.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_environment/core/utils.h>
#include <string>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

using namespace tesseract_common;
using namespace tesseract_environment;
using namespace tesseract_planning;


class WireCuttingProblemGenerator
{
public:
    WireCuttingProblemGenerator(tesseract_monitoring::EnvironmentMonitor::Ptr monitor);
    
    ProcessPlanningRequest construct_request_cut(const VectorIsometry3d& tool_poses, const Environment::ConstPtr& env, ManipulatorInfo& manip);
    ProcessPlanningRequest construct_request_cut_descartes(const VectorIsometry3d& tool_poses);
    ProcessPlanningRequest construct_request_p2p(const JointState& start, const JointState& end);

    TrajOptWireCuttingPlanProfile::Ptr m_plan_cut;
    Environment::Ptr m_env_cut;
    Environment::Ptr m_env_free;
private:  
    TrajOptPlanProfile::Ptr m_plan_free;
    tesseract_monitoring::EnvironmentMonitor::Ptr monitor_; 

};

#endif