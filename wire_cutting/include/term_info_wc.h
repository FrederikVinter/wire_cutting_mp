#ifndef TERM_INFO_WC_H
#define TERM_INFO_WC_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <unordered_map>
TRAJOPT_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <trajopt/common.hpp>
#include <trajopt/json_marshal.hpp>
#include <trajopt_sco/optimizers.hpp>

#include <trajopt/problem_description.hpp>
#include <trajopt/kinematic_terms.hpp>


using namespace trajopt;
using namespace std;
using namespace sco;
using namespace Eigen;
using namespace util;

struct CartPoseTermInfoWC : public TermInfo
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief Timestep at which to apply term */
  int timestep;
  /** @brief  Cartesian position */
  Eigen::Vector3d xyz;
  /** @brief Rotation quaternion */
  Eigen::Vector4d wxyz;
  /** @brief coefficients for position and rotation */
  Eigen::Vector3d pos_coeffs, rot_coeffs;
  /** @brief Link which should reach desired pose */
  std::string link;
  /** @brief Static transform applied to the link */
  Eigen::Isometry3d tcp;
  /** @brief The frame relative to which the target position is defined. If empty, frame is assumed to the root,
   * "world", frame */
  std::string target;

  sco::PenaltyType penalty_type;

  CartPoseTermInfoWC();

  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) override;
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob) override;
  DEFINE_CREATE(CartPoseTermInfoWC)
};


struct CartRotVelTermInfo : public TermInfo
{
  /** @brief Timesteps over which to apply term */
  int first_step, last_step;
  /** @brief Link to which the term is applied */
  std::string link;
  Eigen::VectorXd rot_coeffs;
  sco::PenaltyType penalty_type;
  double max_displacement;
  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) override;
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob) override;
  DEFINE_CREATE(CartVelTermInfo)

  /** @brief Initialize term with it's supported types */
  CartRotVelTermInfo() : TermInfo(trajopt::TermType::TT_COST | trajopt::TermType::TT_CNT) {}
};

struct CartVelTermInfoWC : public TermInfo
{
  /** @brief Timesteps over which to apply term */
  int first_step, last_step;
  /** @brief Link to which the term is applied */
  std::string link;
  Eigen::VectorXd coeffs;
  sco::PenaltyType penalty_type;
  double max_displacement;
  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) override;
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob) override;
  DEFINE_CREATE(CartVelTermInfoWC)

  /** @brief Initialize term with it's supported types */
  CartVelTermInfoWC() : TermInfo(trajopt::TermType::TT_COST | trajopt::TermType::TT_CNT) {}
};

struct CartVelErrCalculatorWC : sco::VectorOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  std::string link_;
  tesseract_environment::AdjacencyMapPair::ConstPtr kin_link_;
  double limit_;
  Eigen::Isometry3d tcp_;
  CartVelErrCalculatorWC(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                       tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                       const Eigen::Isometry3d& world_to_base,
                       std::string link,
                       double limit,
                       const Eigen::Isometry3d& tcp = Eigen::Isometry3d::Identity())
    : manip_(std::move(manip))
    , adjacency_map_(std::move(adjacency_map))
    , world_to_base_(world_to_base)
    , link_(std::move(link))
    , limit_(limit)
    , tcp_(tcp)
  {
    kin_link_ = adjacency_map_->getLinkMapping(link_);
  }

  Eigen::VectorXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

struct CartRotVelErrCalculator : sco::VectorOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  std::string link_;
  tesseract_environment::AdjacencyMapPair::ConstPtr kin_link_;
  double limit_;
  Eigen::Isometry3d tcp_;
  CartRotVelErrCalculator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                       tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                       const Eigen::Isometry3d& world_to_base,
                       std::string link,
                       double limit,
                       const Eigen::Isometry3d& tcp = Eigen::Isometry3d::Identity())
    : manip_(std::move(manip))
    , adjacency_map_(std::move(adjacency_map))
    , world_to_base_(world_to_base)
    , link_(std::move(link))
    , limit_(limit)
    , tcp_(tcp)
  {
    kin_link_ = adjacency_map_->getLinkMapping(link_);
  }

  Eigen::VectorXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

struct CartAccTermInfoWC : public TermInfo
{
  /** @brief Timesteps over which to apply term */
  int first_step, last_step;
  /** @brief Link to which the term is applied */
  std::string link;
  Eigen::VectorXd coeffs;
  sco::PenaltyType penalty_type;

  std::vector<double> displacements;
  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) override;
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob) override;
  DEFINE_CREATE(CartAccTermInfoWC)

  /** @brief Initialize term with it's supported types */
  CartAccTermInfoWC() : TermInfo(trajopt::TermType::TT_COST | trajopt::TermType::TT_CNT | trajopt::TermType::TT_USE_TIME) {}
};

struct CartAccErrCalculator : sco::VectorOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  std::string link_;
  tesseract_environment::AdjacencyMapPair::ConstPtr kin_link_;
  Eigen::Isometry3d tcp_;
  CartAccErrCalculator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                       tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                       const Eigen::Isometry3d& world_to_base,
                       std::string link,
                       const Eigen::Isometry3d& tcp = Eigen::Isometry3d::Identity())
    : manip_(std::move(manip))
    , adjacency_map_(std::move(adjacency_map))
    , world_to_base_(world_to_base)
    , link_(std::move(link))
    , tcp_(tcp)
  {
    kin_link_ = adjacency_map_->getLinkMapping(link_);
  }

  Eigen::VectorXd operator()(const Eigen::VectorXd& dof_vals) const override;
};


struct CartRotVelJacCalculator : sco::MatrixOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  std::string link_;
  tesseract_environment::AdjacencyMapPair::ConstPtr kin_link_;
  double limit_;
  Eigen::Isometry3d tcp_;
  CartRotVelJacCalculator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                       tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                       const Eigen::Isometry3d& world_to_base,
                       std::string link,
                       double limit,
                       const Eigen::Isometry3d& tcp = Eigen::Isometry3d::Identity())
    : manip_(std::move(manip))
    , adjacency_map_(std::move(adjacency_map))
    , world_to_base_(world_to_base)
    , link_(std::move(link))
    , limit_(limit)
    , tcp_(tcp)
  {
    kin_link_ = adjacency_map_->getLinkMapping(link_);
  }

  Eigen::MatrixXd operator()(const Eigen::VectorXd& dof_vals) const override;
};




#endif 
