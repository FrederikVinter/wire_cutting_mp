#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <trajopt_wire_cutting_plan_profile.h>

#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/instruction_type.h>

using namespace tesseract_planning;

TrajOptWireCuttingPlanProfile::TrajOptWireCuttingPlanProfile(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* cartesian_coeff_element = xml_element.FirstChildElement("CartesianCoeff");
  const tinyxml2::XMLElement* joint_coeff_element = xml_element.FirstChildElement("JointCoeff");
  const tinyxml2::XMLElement* term_type_element = xml_element.FirstChildElement("Term");
  const tinyxml2::XMLElement* cnt_error_fn_element = xml_element.FirstChildElement("ConstraintErrorFunctions");

  tinyxml2::XMLError status;

  if (cartesian_coeff_element)
  {
    std::vector<std::string> cart_coeff_tokens;
    std::string cart_coeff_string;
    status = tesseract_common::QueryStringText(cartesian_coeff_element, cart_coeff_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("TrajoptPlanProfile: Error parsing CartesianCoeff string");

    boost::split(cart_coeff_tokens, cart_coeff_string, boost::is_any_of(" "), boost::token_compress_on);

    if (!tesseract_common::isNumeric(cart_coeff_tokens))
      throw std::runtime_error("TrajoptPlanProfile: CartesianCoeff are not all numeric values.");

    cartesian_coeff.resize(static_cast<long>(cart_coeff_tokens.size()));
    for (std::size_t i = 0; i < cart_coeff_tokens.size(); ++i)
      tesseract_common::toNumeric<double>(cart_coeff_tokens[i], cartesian_coeff[static_cast<long>(i)]);
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

  Eigen::VectorXd cart_coeff1 = Eigen::VectorXd::Constant(6, 1, 1);
  cart_coeff1(1) = 0;
  cart_coeff1(4) = 0;

  Eigen::VectorXd cart_coeff2 = Eigen::VectorXd::Constant(6, 1, 0);
  cart_coeff2(1) = 10;
  cart_coeff2(4) = 5;

  auto ti1 = createCartesianWaypointTermInfo(
      cartesian_waypoint, index, mi.working_frame, tcp, cart_coeff1, pci.kin->getTipLinkName(), trajopt::TermType::TT_CNT);
  
  auto ti2 = createCartesianWaypointTermInfo(
      cartesian_waypoint, index, mi.working_frame, tcp, cart_coeff2, pci.kin->getTipLinkName(), trajopt::TermType::TT_COST);
  
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

tinyxml2::XMLElement* TrajOptWireCuttingPlanProfile::toXML(tinyxml2::XMLDocument& doc) const
{
  Eigen::IOFormat eigen_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");

  tinyxml2::XMLElement* xml_planner = doc.NewElement("Planner");
  xml_planner->SetAttribute("type", std::to_string(1).c_str());

  tinyxml2::XMLElement* xml_trajopt = doc.NewElement("TrajoptPlanProfile");

  tinyxml2::XMLElement* xml_cart_coeff = doc.NewElement("CartesianCoefficients");
  std::stringstream cart_coeff;
  cart_coeff << cartesian_coeff.format(eigen_format);
  xml_cart_coeff->SetText(cart_coeff.str().c_str());
  xml_trajopt->InsertEndChild(xml_cart_coeff);

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
