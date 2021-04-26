#include <term_info_wc.h>

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <boost/algorithm/string.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/collision_terms.hpp>
#include <trajopt/common.hpp>
#include <trajopt/kinematic_terms.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt/trajectory_costs.hpp>
#include <trajopt_sco/expr_op_overloads.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/eigen_slicing.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/vector_ops.hpp>
#include <tesseract_kinematics/core/utils.h>

#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>


void ensure_only_members(const Json::Value& v, const char** fields, int nvalid)
{
  for (Json::ValueConstIterator it = v.begin(); it != v.end(); ++it)
  {
    bool valid = false;
    for (int j = 0; j < nvalid; ++j)
    {
      JSONCPP_STRING member_name = it.name();
      if (strcmp(member_name.c_str(), fields[j]) == 0)
      {
        valid = true;
        break;
      }
    }
    if (!valid)
    {
      PRINT_AND_THROW(boost::format("invalid field found: %s") % it.name());
    }
  }
}

CartPoseTermInfoWC::CartPoseTermInfoWC() : TermInfo(TT_COST | TT_CNT)
{
  pos_coeffs = Eigen::Vector3d::Ones();
  rot_coeffs = Eigen::Vector3d::Ones();
  tcp.setIdentity();
}

void CartPoseTermInfoWC::fromJson(ProblemConstructionInfo& pci, const Json::Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  Eigen::Vector3d tcp_xyz = Eigen::Vector3d::Zero();
  Eigen::Vector4d tcp_wxyz = Eigen::Vector4d(1, 0, 0, 0);

  const Json::Value& params = v["params"];
  json_marshal::childFromJson(params, timestep, "timestep", pci.basic_info.n_steps - 1);
  json_marshal::childFromJson(params, xyz, "xyz");
  json_marshal::childFromJson(params, wxyz, "wxyz");
  json_marshal::childFromJson(params, pos_coeffs, "pos_coeffs", Eigen::Vector3d(1, 1, 1));
  json_marshal::childFromJson(params, rot_coeffs, "rot_coeffs", Eigen::Vector3d(1, 1, 1));
  json_marshal::childFromJson(params, link, "link");
  json_marshal::childFromJson(params, tcp_xyz, "tcp_xyz", Eigen::Vector3d(0, 0, 0));
  json_marshal::childFromJson(params, tcp_wxyz, "tcp_wxyz", Eigen::Vector4d(1, 0, 0, 0));
  json_marshal::childFromJson(params, target, "target", std::string(""));

  Eigen::Quaterniond q(tcp_wxyz(0), tcp_wxyz(1), tcp_wxyz(2), tcp_wxyz(3));
  tcp.linear() = q.matrix();
  tcp.translation() = tcp_xyz;

  const std::vector<std::string>& link_names = pci.kin->getActiveLinkNames();
  if (std::find(link_names.begin(), link_names.end(), link) == link_names.end())
  {
    PRINT_AND_THROW(boost::format("invalid link name: %s") % link);
  }

  if (target.empty())
  {
    target = pci.env->getRootLinkName();
  }
  else
  {
    const std::vector<std::string>& all_links = pci.env->getLinkNames();
    if (std::find(all_links.begin(), all_links.end(), target) == all_links.end())
    {
      PRINT_AND_THROW(boost::format("invalid target link name: %s") % target);
    }
  }

  const char* all_fields[] = { "timestep", "xyz",     "wxyz",     "pos_coeffs", "rot_coeffs",
                               "link",     "tcp_xyz", "tcp_wxyz", "target" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void CartPoseTermInfoWC::hatch(TrajOptProb& prob)
{
  int n_dof = static_cast<int>(prob.GetKin()->numJoints());

  Eigen::Isometry3d input_pose;
  Eigen::Quaterniond q(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
  input_pose.linear() = q.matrix();
  input_pose.translation() = xyz;

  tesseract_environment::EnvState::ConstPtr state = prob.GetEnv()->getCurrentState();
  Eigen::Isometry3d world_to_base = Eigen::Isometry3d::Identity();
  try
  {
    world_to_base = state->link_transforms.at(prob.GetKin()->getBaseLinkName());
  }
  catch (const std::exception&)
  {
    PRINT_AND_THROW(boost::format("Failed to find transform for link '%s'") % prob.GetKin()->getBaseLinkName());
  }

  Eigen::Isometry3d world_to_target = Eigen::Isometry3d::Identity();
  if (!target.empty())
  {
    try
    {
      world_to_target = state->link_transforms.at(target);
    }
    catch (const std::exception& ex)
    {
      PRINT_AND_THROW(boost::format("Failed to find transform for link '%s'") % target);
    }
  }

  tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      prob.GetEnv()->getSceneGraph(), prob.GetKin()->getActiveLinkNames(), state->link_transforms);

  // Next parse the coeff and if not zero add the indice and coeff
  std::vector<int> ic;
  std::vector<double> c;
  ic.reserve(6);
  c.reserve(6);
  for (int i = 0; i < 3; ++i)
  {
    if (std::abs(pos_coeffs[i]) > 1e-5)
    {
      ic.push_back(i);
      c.push_back(pos_coeffs[i]);
    }
  }

  for (int i = 0; i < 3; ++i)
  {
    if (std::abs(rot_coeffs[i]) > 1e-5)
    {
      ic.push_back(i + 3);
      c.push_back(rot_coeffs[i]);
    }
  }

  Eigen::VectorXi indices = Eigen::Map<Eigen::VectorXi>(ic.data(), static_cast<long>(ic.size()));
  Eigen::VectorXd coeff = Eigen::Map<Eigen::VectorXd>(c.data(), static_cast<long>(c.size()));

  if (term_type == (TT_COST | TT_USE_TIME))
  {
    CONSOLE_BRIDGE_logError("Use time version of this term has not been defined.");
  }
  else if (term_type == (TT_CNT | TT_USE_TIME))
  {
    CONSOLE_BRIDGE_logError("Use time version of this term has not been defined.");
  }
  else if ((term_type & TT_COST) && ~(term_type | ~TT_USE_TIME))
  {
    auto f = std::make_shared<CartPoseErrCalculator>(
        world_to_target * input_pose, prob.GetKin(), adjacency_map, world_to_base, link, tcp, indices);

    // This is currently not being used. There is an intermittent bug that needs to be tracked down it is not used.
    auto dfdx = std::make_shared<CartPoseJacCalculator>(
        input_pose, prob.GetKin(), adjacency_map, world_to_base, link, tcp, indices);
    prob.addCost(
        std::make_shared<TrajOptCostFromErrFunc>(f, prob.GetVarRow(timestep, 0, n_dof), coeff, sco::SQUARED, name));
  }
  else if ((term_type & TT_CNT) && ~(term_type | ~TT_USE_TIME))
  {
    auto f = std::make_shared<CartPoseErrCalculator>(
        world_to_target * input_pose, prob.GetKin(), adjacency_map, world_to_base, link, tcp, indices);

    // This is currently not being used. There is an intermittent bug that needs to be tracked down it is not used.
    auto dfdx = std::make_shared<CartPoseJacCalculator>(
        input_pose, prob.GetKin(), adjacency_map, world_to_base, link, tcp, indices);
    prob.addConstraint(
        std::make_shared<TrajOptConstraintFromErrFunc>(f, prob.GetVarRow(timestep, 0, n_dof), coeff, sco::EQ, name));
  }
  else
  {
    CONSOLE_BRIDGE_logWarn("CartPoseTermInfoWC does not have a valid term_type defined. No cost/constraint applied");
  }
}

void CartRotVelTermInfo::fromJson(ProblemConstructionInfo& pci, const Json::Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Json::Value& params = v["params"];
  json_marshal::childFromJson(params, first_step, "first_step");
  json_marshal::childFromJson(params, last_step, "last_step");
  json_marshal::childFromJson(params, max_displacement, "max_displacement");

  FAIL_IF_FALSE((first_step >= 0) && (first_step <= pci.basic_info.n_steps - 1) && (first_step < last_step));
  FAIL_IF_FALSE((last_step > 0) && (last_step <= pci.basic_info.n_steps - 1));

  json_marshal::childFromJson(params, link, "link");
  const std::vector<std::string>& link_names = pci.kin->getActiveLinkNames();
  if (std::find(link_names.begin(), link_names.end(), link) == link_names.end())
  {
    PRINT_AND_THROW(boost::format("invalid link name: %s") % link);
  }

  const char* all_fields[] = { "first_step", "last_step", "max_displacement", "link" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void CartRotVelTermInfo::hatch(TrajOptProb& prob)
{
  int n_dof = static_cast<int>(prob.GetKin()->numJoints());

  tesseract_environment::EnvState::ConstPtr state = prob.GetEnv()->getCurrentState();
  Eigen::Isometry3d world_to_base = Eigen::Isometry3d::Identity();
  try
  {
    world_to_base = state->link_transforms.at(prob.GetKin()->getBaseLinkName());
  }
  catch (const std::exception&)
  {
    PRINT_AND_THROW(boost::format("Failed to find transform for link '%s'") % prob.GetKin()->getBaseLinkName());
  }

  tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      prob.GetEnv()->getSceneGraph(), prob.GetKin()->getActiveLinkNames(), state->link_transforms);

  if (term_type == (TT_COST | TT_USE_TIME))
  {
    CONSOLE_BRIDGE_logError("Use time version of this term has not been defined.");
  }
  else if (term_type == (TT_CNT | TT_USE_TIME))
  {
    CONSOLE_BRIDGE_logError("Use time version of this term has not been defined.");
  }
  else if ((term_type & TT_COST) && ~(term_type | ~TT_USE_TIME))
  {
    for (int iStep = first_step; iStep <= last_step; ++iStep)
    {
      auto f =
          std::make_shared<CartRotVelErrCalculator>(prob.GetKin(), adjacency_map, world_to_base, link, max_displacement);

      auto dfdx =
          std::make_shared<CartRotVelJacCalculator>(prob.GetKin(), adjacency_map, world_to_base, link, max_displacement);
      
      Eigen::VectorXd coeff(6);
      coeff << 0, 1, 0, 0, 1, 0;
      prob.addCost(std::make_shared<TrajOptCostFromErrFunc>(
          f,
          dfdx,
          concat(prob.GetVarRow(iStep, 0, n_dof), prob.GetVarRow(iStep + 1, 0, n_dof)),
          coeff,
          sco::HINGE,
          "CartRotVel"));
    }
  }
  else if ((term_type & TT_CNT) && ~(term_type | ~TT_USE_TIME))
  {
    for (int iStep = first_step; iStep <= last_step; ++iStep)
    {
      auto f =
          std::make_shared<CartRotVelErrCalculator>(prob.GetKin(), adjacency_map, world_to_base, link, max_displacement);
     
      auto dfdx =
          std::make_shared<CartRotVelJacCalculator>(prob.GetKin(), adjacency_map, world_to_base, link, max_displacement);
      prob.addConstraint(std::make_shared<TrajOptConstraintFromErrFunc>(
          f,
          dfdx,
          concat(prob.GetVarRow(iStep, 0, n_dof), prob.GetVarRow(iStep + 1, 0, n_dof)),
          Eigen::VectorXd::Ones(0),
          sco::INEQ,
          "CartRotVel"));
    }
  }
  else
  {
    CONSOLE_BRIDGE_logWarn("CartVelTermInfo does not have a valid term_type defined. No cost/constraint applied");
  }
}

void CartVelTermInfoWC::fromJson(ProblemConstructionInfo& pci, const Json::Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Json::Value& params = v["params"];
  json_marshal::childFromJson(params, first_step, "first_step");
  json_marshal::childFromJson(params, last_step, "last_step");
  json_marshal::childFromJson(params, max_displacement, "max_displacement");

  FAIL_IF_FALSE((first_step >= 0) && (first_step <= pci.basic_info.n_steps - 1) && (first_step < last_step));
  FAIL_IF_FALSE((last_step > 0) && (last_step <= pci.basic_info.n_steps - 1));

  json_marshal::childFromJson(params, link, "link");
  const std::vector<std::string>& link_names = pci.kin->getActiveLinkNames();
  if (std::find(link_names.begin(), link_names.end(), link) == link_names.end())
  {
    PRINT_AND_THROW(boost::format("invalid link name: %s") % link);
  }

  const char* all_fields[] = { "first_step", "last_step", "max_displacement", "link" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void CartVelTermInfoWC::hatch(TrajOptProb& prob)
{
  int n_dof = static_cast<int>(prob.GetKin()->numJoints());

  tesseract_environment::EnvState::ConstPtr state = prob.GetEnv()->getCurrentState();
  Eigen::Isometry3d world_to_base = Eigen::Isometry3d::Identity();
  try
  {
    world_to_base = state->link_transforms.at(prob.GetKin()->getBaseLinkName());
  }
  catch (const std::exception&)
  {
    PRINT_AND_THROW(boost::format("Failed to find transform for link '%s'") % prob.GetKin()->getBaseLinkName());
  }

  tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      prob.GetEnv()->getSceneGraph(), prob.GetKin()->getActiveLinkNames(), state->link_transforms);

  if (term_type == (TT_COST | TT_USE_TIME))
  {
    CONSOLE_BRIDGE_logError("Use time version of this term has not been defined.");
  }
  else if (term_type == (TT_CNT | TT_USE_TIME))
  {
    CONSOLE_BRIDGE_logError("Use time version of this term has not been defined.");
  }
  else if ((term_type & TT_COST) && ~(term_type | ~TT_USE_TIME))
  {
    for (int iStep = first_step; iStep <= last_step; ++iStep)
    {
      Eigen::VectorXd coeff(6);
      coeff << 0, 5, 0, 0, 5, 0;
      auto f =
          std::make_shared<CartVelErrCalculator>(prob.GetKin(), adjacency_map, world_to_base, link, max_displacement);
      auto dfdx =
          std::make_shared<CartVelJacCalculator>(prob.GetKin(), adjacency_map, world_to_base, link, max_displacement);
      prob.addCost(std::make_shared<TrajOptCostFromErrFunc>(
          f,
          dfdx,
          concat(prob.GetVarRow(iStep, 0, n_dof), prob.GetVarRow(iStep + 1, 0, n_dof)),
          coeff,
          sco::ABS,
          name));
    }
  }
  else if ((term_type & TT_CNT) && ~(term_type | ~TT_USE_TIME))
  {
    for (int iStep = first_step; iStep <= last_step; ++iStep)
    {
      auto f =
          std::make_shared<CartVelErrCalculator>(prob.GetKin(), adjacency_map, world_to_base, link, max_displacement);
      auto dfdx =
          std::make_shared<CartVelJacCalculator>(prob.GetKin(), adjacency_map, world_to_base, link, max_displacement);
      prob.addConstraint(std::make_shared<TrajOptConstraintFromErrFunc>(
          f,
          dfdx,
          concat(prob.GetVarRow(iStep, 0, n_dof), prob.GetVarRow(iStep + 1, 0, n_dof)),
          Eigen::VectorXd::Ones(0),
          sco::INEQ,
          "CartVel"));
    }
  }
  else
  {
    CONSOLE_BRIDGE_logWarn("CartVelTermInfo does not have a valid term_type defined. No cost/constraint applied");
  }
}


VectorXd CartRotVelErrCalculator::operator()(const VectorXd& dof_vals) const
{
    auto n_dof = static_cast<int>(manip_->numJoints());
    Isometry3d pose0, pose1;

    manip_->calcFwdKin(pose0, dof_vals.topRows(n_dof), kin_link_->link_name);
    manip_->calcFwdKin(pose1, dof_vals.bottomRows(n_dof), kin_link_->link_name);

    pose0 = world_to_base_ * pose0 * kin_link_->transform * tcp_;
    pose1 = world_to_base_ * pose1 * kin_link_->transform * tcp_;

    
    /*auto pose0_RPY = pose0.rotation().eulerAngles(0, 1, 2);
    auto pose1_RPY = pose1.rotation().eulerAngles(0, 1, 2);

    VectorXd out(2);
    out[0] = pose0_RPY[1] - pose1_RPY[1];
    out[1] = pose1_RPY[1] - pose0_RPY[1];

    std::cout << out[0] << std::endl << std::endl;
    std::cout << out[1] << std::endl << std::endl;*/

    VectorXd out(6);
    out.topRows(3) = (pose1.rotation().eulerAngles(0, 1, 2) - pose0.rotation().eulerAngles(0, 1, 2) - Vector3d(limit_, limit_, limit_));
    out.bottomRows(3) = (pose0.rotation().eulerAngles(0, 1, 2) - pose1.rotation().eulerAngles(0, 1, 2) - Vector3d(limit_, limit_, limit_));
    return out;
}

MatrixXd CartRotVelJacCalculator::operator()(const VectorXd& dof_vals) const
{
  auto n_dof = static_cast<int>(manip_->numJoints());
  MatrixXd out(6, 2 * n_dof);

  MatrixXd jac0, jac1;
  Eigen::Isometry3d tf0, tf1;

  jac0.resize(6, manip_->numJoints());
  jac1.resize(6, manip_->numJoints());

  if (tcp_.translation().isZero())
  {
    manip_->calcFwdKin(tf0, dof_vals.topRows(n_dof), kin_link_->link_name);
    manip_->calcJacobian(jac0, dof_vals.topRows(n_dof), kin_link_->link_name);
    tesseract_kinematics::jacobianChangeBase(jac0, world_to_base_);
    tesseract_kinematics::jacobianChangeRefPoint(jac0,
                                                 (world_to_base_ * tf0).linear() * kin_link_->transform.translation());

    manip_->calcFwdKin(tf1, dof_vals.bottomRows(n_dof), kin_link_->link_name);
    manip_->calcJacobian(jac1, dof_vals.bottomRows(n_dof), kin_link_->link_name);
    tesseract_kinematics::jacobianChangeBase(jac1, world_to_base_);
    tesseract_kinematics::jacobianChangeRefPoint(jac1,
                                                 (world_to_base_ * tf1).linear() * kin_link_->transform.translation());
  }
  else
  {
    manip_->calcFwdKin(tf0, dof_vals.topRows(n_dof), kin_link_->link_name);
    manip_->calcJacobian(jac0, dof_vals.topRows(n_dof), kin_link_->link_name);
    tesseract_kinematics::jacobianChangeBase(jac0, world_to_base_);
    tesseract_kinematics::jacobianChangeRefPoint(
        jac0, (world_to_base_ * tf0).linear() * (kin_link_->transform * tcp_).translation());

    manip_->calcFwdKin(tf1, dof_vals.bottomRows(n_dof), kin_link_->link_name);
    manip_->calcJacobian(jac1, dof_vals.bottomRows(n_dof), kin_link_->link_name);
    tesseract_kinematics::jacobianChangeBase(jac1, world_to_base_);
    tesseract_kinematics::jacobianChangeRefPoint(
        jac1, (world_to_base_ * tf1).linear() * (kin_link_->transform * tcp_).translation());
  }

  out.block(0, 0, 3, n_dof) = -jac0.bottomRows(3);
  out.block(0, n_dof, 3, n_dof) = jac1.bottomRows(3);
  out.block(3, 0, 3, n_dof) = jac0.bottomRows(3);
  out.block(3, n_dof, 3, n_dof) = -jac1.bottomRows(3);
  
  return out;
}
