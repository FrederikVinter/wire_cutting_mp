#ifndef TRAJOPT_WIRE_CUTTING_PLAN_PROFILE_H
#define TRAJOPT_WIRE_CUTTING_PLAN_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt_sco/modeling_utils.hpp>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include "four_bar_linkage_constraint.h"

#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>

using namespace tesseract_planning;

class TrajOptWireCuttingPlanProfile : public TrajOptPlanProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptWireCuttingPlanProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptWireCuttingPlanProfile>;

  TrajOptWireCuttingPlanProfile() = default;
  ~TrajOptWireCuttingPlanProfile() override = default;
  TrajOptWireCuttingPlanProfile(const tinyxml2::XMLElement& xml_element);
  TrajOptWireCuttingPlanProfile(const TrajOptWireCuttingPlanProfile&) = default;
  TrajOptWireCuttingPlanProfile& operator=(const TrajOptWireCuttingPlanProfile&) = default;
  TrajOptWireCuttingPlanProfile(TrajOptWireCuttingPlanProfile&&) = default;
  TrajOptWireCuttingPlanProfile& operator=(TrajOptWireCuttingPlanProfile&&) = default;

  Eigen::VectorXd cart_coeff_cnt{ Eigen::VectorXd::Constant(6, 1, 5) };
  Eigen::VectorXd cart_coeff_cost{ Eigen::VectorXd::Constant(6, 1, 0) };
  Eigen::VectorXd joint_coeff{ Eigen::VectorXd::Constant(1, 1, 5) };
  trajopt::TermType term_type{ trajopt::TermType::TT_CNT };

  /** @brief Error function that is set as a constraint for each timestep.
   *
   * This is a vector of std::tuple<Error Function, Error Function Jacobian, Constraint Type, Coeff>, the error
   * function, constraint type, and coeff is required, but the jacobian is optional (nullptr).
   *
   * Error Function:
   *   arg: VectorXd will be all of the joint values for one timestep.
   *   return: VectorXd of violations for each joint. Anything != 0 will be a violation
   *
   * Error Function Jacobian:
   *   arg: VectorXd will be all of the joint values for one timestep.
   *   return: Eigen::MatrixXd that represents the change in the error function with respect to joint values
   *
   * Error Constraint Type
   *
   * Coefficients/Weights
   *
   */
  std::vector<std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd>>
      constraint_error_functions;

  void apply(trajopt::ProblemConstructionInfo& pci,
             const CartesianWaypoint& cartesian_waypoint,
             const Instruction& parent_instruction,
             const ManipulatorInfo& manip_info,
             const std::vector<std::string>& active_links,
             int index) const override;

  void apply(trajopt::ProblemConstructionInfo& pci,
             const JointWaypoint& joint_waypoint,
             const Instruction& parent_instruction,
             const ManipulatorInfo& manip_info,
             const std::vector<std::string>& active_links,
             int index) const override;

  void addFourBarLinkageConstraints();

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const override;

protected:
  void addConstraintErrorFunctions(trajopt::ProblemConstructionInfo& pci, int index) const;

  void addAvoidSingularity(trajopt::ProblemConstructionInfo& pci, const std::vector<int>& fixed_steps) const;
};

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_PLAN_PROFILE_H
