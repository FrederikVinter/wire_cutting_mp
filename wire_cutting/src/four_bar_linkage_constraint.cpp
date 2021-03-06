#include <four_bar_linkage_constraint.h>

using namespace Eigen;

// current_joints_pos(0) -> joint_1
// current_joints_pos(1) -> joint_2
// current_joints_pos(2) -> joint_3
// current_joints_pos(3) -> joint_4
// current_joints_pos(4) -> joint_5
// current_joints_pos(5) -> joint_6

#define DEGREE2RADIAN M_PI/180.0
#define RADIAN2DEGREE  180.0/M_PI


VectorXd JointTwoLimitsConstraint::operator()(const VectorXd& current_joints_pos) const
{
    Eigen::VectorXd violation(2);
    violation << 0, 0;

    double joint3_pos = current_joints_pos(1) + current_joints_pos(2);

    if(current_joints_pos(1) < joint3_pos - 67*DEGREE2RADIAN)
        violation(0) = (joint3_pos - 67*DEGREE2RADIAN) - current_joints_pos(1);

    if(current_joints_pos(1) > joint3_pos + 65*DEGREE2RADIAN)
        violation(1) =  current_joints_pos(1) - (joint3_pos + 65*DEGREE2RADIAN);

    return violation;
}

void JointTwoLimitsConstraint::Plot(const tesseract_visualization::Visualization::Ptr& plotter, const VectorXd& dof_vals)
{

}

VectorXd JointThreeLimitsConstraint::operator()(const VectorXd& current_joints_pos) const
{
    Eigen::VectorXd violation(2);
    violation << 0, 0;

    double joint3_pos = current_joints_pos(1) + current_joints_pos(2);

    // Sliding limits
    if(joint3_pos < current_joints_pos(1) - 65*DEGREE2RADIAN)
        violation(0) = (current_joints_pos(1) - 65*DEGREE2RADIAN) - joint3_pos;

    if(joint3_pos > current_joints_pos(1) + 67*DEGREE2RADIAN)
        violation(1) =  joint3_pos - (current_joints_pos(1) + 67*DEGREE2RADIAN);

    return violation;
}

void JointThreeLimitsConstraint::Plot(const tesseract_visualization::Visualization::Ptr& plotter, const VectorXd& dof_vals)
{

}

VectorXd JointThreeAbsoluteLimitsConstraint::operator()(const VectorXd& current_joints_pos) const
{
    Eigen::VectorXd violation(2);
    violation << 0, 0;

    double joint3_pos = current_joints_pos(1) + current_joints_pos(2);

    // Absolute limits based on joint3_pos
    if(joint3_pos > 1.91)
        violation(0) = joint3_pos - 1.91;

    if(joint3_pos < -0.489)
        violation(1) = joint3_pos + 0.489;

    return violation;
}

void JointThreeAbsoluteLimitsConstraint::Plot(const tesseract_visualization::Visualization::Ptr& plotter, const VectorXd& dof_vals)
{

}