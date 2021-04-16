#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <trajopt_wire_cutting_plan_profile.h>

#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/instruction_type.h>

using namespace tesseract_planning;

TrajOptWireCuttingPlanProfile::TrajOptWireCuttingPlanProfile(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* cartesian_coeff_cnt_element = xml_element.FirstChildElement("CartesianCoefficientsConstraint");
  const tinyxml2::XMLElement* cartesian_coeff_cost_element = xml_element.FirstChildElement("CartesianCoefficientsCost");
  const tinyxml2::XMLElement* joint_coeff_element = xml_element.FirstChildElement("JointCoefficients");
  const tinyxml2::XMLElement* term_type_element = xml_element.FirstChildElement("Term");
  const tinyxml2::XMLElement* cnt_error_fn_element = xml_element.FirstChildElement("ConstraintErrorFunctions");

  tinyxml2::XMLError status;

  if (cartesian_coeff_cnt_element)
  {
    std::vector<std::string> cart_coeff_cnt_tokens;
    std::string cart_coeff_cnt_string;
    status = tesseract_common::QueryStringText(cartesian_coeff_cnt_element, cart_coeff_cnt_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("TrajoptPlanProfile: Error parsing CartesianCoeffCNT string");

    boost::split(cart_coeff_cnt_tokens, cart_coeff_cnt_string, boost::is_any_of(" "), boost::token_compress_on);

    if (!tesseract_common::isNumeric(cart_coeff_cnt_tokens))
      throw std::runtime_error("TrajoptPlanProfile: CartesianCoeffCOST are not all numeric values.");

    cart_coeff_cnt.resize(static_cast<long>(cart_coeff_cnt_tokens.size()));
    for (std::size_t i = 0; i < cart_coeff_cnt_tokens.size(); ++i)
      tesseract_common::toNumeric<double>(cart_coeff_cnt_tokens[i], cart_coeff_cnt[static_cast<long>(i)]);
  }

  if (cartesian_coeff_cost_element)
  {
    std::vector<std::string> cart_coeff_cost_tokens;
    std::string cart_coeff_cost_string;
    status = tesseract_common::QueryStringText(cartesian_coeff_cost_element, cart_coeff_cost_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("TrajoptPlanProfile: Error parsing CartesianCoeffCOST string");

    boost::split(cart_coeff_cost_tokens, cart_coeff_cost_string, boost::is_any_of(" "), boost::token_compress_on);

    if (!tesseract_common::isNumeric(cart_coeff_cost_tokens))
      throw std::runtime_error("TrajoptPlanProfile: CartesianCoeffCOST are not all numeric values.");

    cart_coeff_cost.resize(static_cast<long>(cart_coeff_cost_tokens.size()));
    for (std::size_t i = 0; i < cart_coeff_cost_tokens.size(); ++i)
      tesseract_common::toNumeric<double>(cart_coeff_cost_tokens[i], cart_coeff_cost[static_cast<long>(i)]);
  }

  if (joint_coeff_element)
  {
    std::vector<std::string> joint_coeff_tokens;
    std::string joint_coeff_string;
    status = tesseract_common::QueryStringText(joint_coeff_element, joint_coeff_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("TrajoptPlanProfile: Error parsing JointCoeff string");

    boost::split(joint_coeff_tokens, joint_coeff_string, boost::is_any_of(" "), boost::token_compress_on);

    if (!tesseract_common::isNumeric(joint_coeff_tokens))
      throw std::runtime_error("TrajoptPlanProfile: JointCoeff are not all numeric values.");

    joint_coeff.resize(static_cast<long>(joint_coeff_tokens.size()));
    for (std::size_t i = 0; i < joint_coeff_tokens.size(); ++i)
      tesseract_common::toNumeric<double>(joint_coeff_tokens[i], joint_coeff[static_cast<long>(i)]);
  }

  if (term_type_element)
  {
    int type = static_cast<int>(trajopt::TermType::TT_CNT);
    status = term_type_element->QueryIntAttribute("type", &type);
    if (status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("TrajoptPlanProfile: Error parsing Term type attribute.");

    term_type = static_cast<trajopt::TermType>(type);
  }

  if (cnt_error_fn_element)
  {
    std::string error_fn_name;
    status = tesseract_common::QueryStringAttribute(cnt_error_fn_element, "type", error_fn_name);
    if (status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("TrajoptPlanProfile: Error parsing ConstraintErrorFunctions plugin attribute.");

    // TODO: Implement plugin capabilities
  }
}
void TrajOptWireCuttingPlanProfile::apply(trajopt::ProblemConstructionInfo& pci,
                                      const CartesianWaypoint& cartesian_waypoint,
                                      const Instruction& parent_instruction,
                                      const ManipulatorInfo& manip_info,
                                      const std::vector<std::string>& active_links,
                                      int index) const
{
  assert(isPlanInstruction(parent_instruction));
  const auto* base_instruction = parent_instruction.cast_const<PlanInstruction>();
  assert(!(manip_info.empty() && base_instruction->getManipulatorInfo().empty()));
  ManipulatorInfo mi = manip_info.getCombined(base_instruction->getManipulatorInfo());
  Eigen::Isometry3d tcp = pci.env->findTCP(mi);

  auto ti1 = createCartesianWaypointTermInfo(
      cartesian_waypoint, index, mi.working_frame, tcp, cart_coeff_cnt, pci.kin->getTipLinkName(), trajopt::TermType::TT_CNT);
  
  auto ti2 = createCartesianWaypointTermInfoWC(
      cartesian_waypoint, index, mi.working_frame, tcp, cart_coeff_cost, pci.kin->getTipLinkName(), trajopt::TermType::TT_COST);
 
  pci.cnt_infos.push_back(ti1);
  pci.cost_infos.push_back(ti2);

  addConstraintErrorFunctions(pci, index);
}

void TrajOptWireCuttingPlanProfile::apply(trajopt::ProblemConstructionInfo& pci,
                                      const JointWaypoint& joint_waypoint,
                                      const Instruction& /*parent_instruction*/,
                                      const ManipulatorInfo& /*manip_info*/,
                                      const std::vector<std::string>& /*active_links*/,
                                      int index) const
{
  trajopt::TermInfo::Ptr ti;
  if (joint_waypoint.isToleranced())
    ti = createTolerancedJointWaypointTermInfo(
        joint_waypoint, joint_waypoint.lower_tolerance, joint_waypoint.upper_tolerance, index, joint_coeff, term_type);
  else
    ti = createJointWaypointTermInfo(joint_waypoint, index, joint_coeff, term_type);

  if (term_type == trajopt::TermType::TT_CNT)
    pci.cnt_infos.push_back(ti);
  else
    pci.cost_infos.push_back(ti);

  addConstraintErrorFunctions(pci, index);
}

void TrajOptWireCuttingPlanProfile::addConstraintErrorFunctions(trajopt::ProblemConstructionInfo& pci, int index) const
{
  for (std::size_t i = 0; i < constraint_error_functions.size(); ++i)
  {
    auto& c = constraint_error_functions[i];
    trajopt::TermInfo::Ptr ti =
        createUserDefinedTermInfo(index, index, std::get<0>(c), std::get<1>(c), trajopt::TT_CNT);

    // Update the term info with the (possibly) new start and end state indices for which to apply this cost
    std::shared_ptr<trajopt::UserDefinedTermInfo> ef = std::static_pointer_cast<trajopt::UserDefinedTermInfo>(ti);
    ef->constraint_type = std::get<2>(c);
    ef->coeff = std::get<3>(c);

    pci.cnt_infos.push_back(ef);
  }
}



void TrajOptWireCuttingPlanProfile::addFourBarLinkageConstraints()
{
  JointThreeAbsoluteLimitsConstraint cnt1;
  JointTwoLimitsConstraint cnt2;
  JointThreeLimitsConstraint cnt3;
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> temp_function1 = cnt1;
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> temp_function2 = cnt2;
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> temp_function3 = cnt3;
  sco::VectorOfVector::func temp_1 = temp_function1;
  sco::VectorOfVector::func temp_2 = temp_function2;
  sco::VectorOfVector::func temp_3 = temp_function3;

  sco::ConstraintType a = sco::ConstraintType::INEQ;
  Eigen::VectorXd error_coeff(2);
  error_coeff << 1, 1;

  std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd> temp_tuple1(temp_1,nullptr,a,error_coeff);
  std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd> temp_tuple2(temp_2,nullptr,a,error_coeff);
  std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd> temp_tuple3(temp_3,nullptr,a,error_coeff);

  constraint_error_functions.push_back(temp_tuple1);
  constraint_error_functions.push_back(temp_tuple2);
  constraint_error_functions.push_back(temp_tuple3);
}

tinyxml2::XMLElement* TrajOptWireCuttingPlanProfile::toXML(tinyxml2::XMLDocument& doc) const
{
  Eigen::IOFormat eigen_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");

  tinyxml2::XMLElement* xml_planner = doc.NewElement("Planner");
  xml_planner->SetAttribute("type", std::to_string(1).c_str());

  tinyxml2::XMLElement* xml_trajopt = doc.NewElement("TrajoptPlanProfile");

  tinyxml2::XMLElement* xml_cart_coeff_cnt = doc.NewElement("CartesianCoefficientsConstraint");
  std::stringstream cart_coeff_cnt_ss;
  cart_coeff_cnt_ss << cart_coeff_cnt.format(eigen_format);
  xml_cart_coeff_cnt->SetText(cart_coeff_cnt_ss.str().c_str());
  xml_trajopt->InsertEndChild(xml_cart_coeff_cnt);

  tinyxml2::XMLElement* xml_cart_coeff_cost = doc.NewElement("CartesianCoefficientsCost");
  std::stringstream cart_coeff_cost_ss;
  cart_coeff_cost_ss << cart_coeff_cost.format(eigen_format);
  xml_cart_coeff_cost->SetText(cart_coeff_cost_ss.str().c_str());
  xml_trajopt->InsertEndChild(xml_cart_coeff_cost);

  tinyxml2::XMLElement* xml_joint_coeff = doc.NewElement("JointCoefficients");
  std::stringstream jnt_coeff;
  jnt_coeff << joint_coeff.format(eigen_format);
  xml_joint_coeff->SetText(jnt_coeff.str().c_str());
  xml_trajopt->InsertEndChild(xml_joint_coeff);

  tinyxml2::XMLElement* xml_term_type = doc.NewElement("Term");
  xml_term_type->SetAttribute("type", static_cast<int>(term_type));
  xml_trajopt->InsertEndChild(xml_term_type);

  xml_planner->InsertEndChild(xml_trajopt);

  return xml_planner;
}

