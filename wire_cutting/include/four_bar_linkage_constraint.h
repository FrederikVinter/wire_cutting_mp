#ifndef FOUR_BAR_LINKAGE_CONSTRAINT_H_
#define FOUR_BAR_LINKAGE_CONSTRAINT_H_

#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/common.hpp>
#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/modeling_utils.hpp>


struct FourBarLinkageConstraint : public trajopt::TrajOptVectorOfVector
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    tesseract_environment::Environment::Ptr env_;

    FourBarLinkageConstraint(tesseract_environment::Environment::Ptr env):env_(env){}

    void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const Eigen::VectorXd& dof_vals) override;

    Eigen::VectorXd operator()(const Eigen::VectorXd& current_joints_pos) const override;
};

struct JointTwoLimitsConstraint : public trajopt::TrajOptVectorOfVector
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    tesseract_environment::Environment::Ptr env_;

    JointTwoLimitsConstraint(tesseract_environment::Environment::Ptr env):env_(env){}

    void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const Eigen::VectorXd& dof_vals) override;

    Eigen::VectorXd operator()(const Eigen::VectorXd& current_joints_pos) const override;
};

struct JointThreeLimitsConstraint : public trajopt::TrajOptVectorOfVector
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    tesseract_environment::Environment::Ptr env_;

    JointThreeLimitsConstraint(tesseract_environment::Environment::Ptr env):env_(env){}

    void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const Eigen::VectorXd& dof_vals) override;

    Eigen::VectorXd operator()(const Eigen::VectorXd& current_joints_pos) const override;
};

struct JointThreeAbsoluteLimitsConstraint : public trajopt::TrajOptVectorOfVector
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    tesseract_environment::Environment::Ptr env_;

    JointThreeAbsoluteLimitsConstraint(tesseract_environment::Environment::Ptr env):env_(env){}

    void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const Eigen::VectorXd& dof_vals) override;

    Eigen::VectorXd operator()(const Eigen::VectorXd& current_joints_pos) const override;
};



#endif