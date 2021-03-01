#include <four_bar_linkage_constraint.h>

using namespace Eigen;

// current_joints_pos(0) -> joint_1
// current_joints_pos(1) -> joint_2
// current_joints_pos(2) -> joint_p VIRTUAL
// current_joints_pos(3) -> joint_3
// current_joints_pos(4) -> joint_4
// current_joints_pos(5) -> joint_5
// current_joints_pos(6) -> joint_6

#define DEGREE2RADIAN M_PI/180.0
#define RADIAN2DEGREE  180.0/M_PI

VectorXd FourBarLinkageConstraint::operator()(const VectorXd& current_joints_pos) const
{
    static Eigen::VectorXd violation(1);
    violation << current_joints_pos(1) + current_joints_pos(2); // joint_p = -joint_2
    return violation;
}

void FourBarLinkageConstraint::Plot(const tesseract_visualization::Visualization::Ptr& plotter, const VectorXd& dof_vals)
{

}

VectorXd JointTwoLimitsConstraint::operator()(const VectorXd& current_joints_pos) const
{
    static Eigen::VectorXd violation(1);
    violation << 0;
    if(current_joints_pos(1) < current_joints_pos(3) - 67*DEGREE2RADIAN)
        violation << (current_joints_pos(3) - 67*DEGREE2RADIAN) - current_joints_pos(1);

    if(current_joints_pos(1) > current_joints_pos(3) + 65*DEGREE2RADIAN)
        violation <<  current_joints_pos(1) - (current_joints_pos(3) + 65*DEGREE2RADIAN);

    return violation;
}

void JointTwoLimitsConstraint::Plot(const tesseract_visualization::Visualization::Ptr& plotter, const VectorXd& dof_vals)
{

}

VectorXd JointThreeLimitsConstraint::operator()(const VectorXd& current_joints_pos) const
{
    static Eigen::VectorXd violation(1);
    violation << 0;
    if(current_joints_pos(3) < current_joints_pos(1) - 65*DEGREE2RADIAN)
        violation << (current_joints_pos(1) - 65*DEGREE2RADIAN) - current_joints_pos(3);

    if(current_joints_pos(3) > current_joints_pos(1) + 67*DEGREE2RADIAN)
        violation <<  current_joints_pos(3) - (current_joints_pos(1) + 67*DEGREE2RADIAN);

    return violation;
}

void JointThreeLimitsConstraint::Plot(const tesseract_visualization::Visualization::Ptr& plotter, const VectorXd& dof_vals)
{

}