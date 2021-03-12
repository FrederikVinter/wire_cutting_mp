#include <wire_cutting_problem_generator.h>


WireCuttingProblemGenerator::WireCuttingProblemGenerator(const ros::NodeHandle& nh)
{
    const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
    const std::string ROBOT_DESCRIPTION_FREESPACE_PARAM = "robot_description_freespace";

    const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";
    const std::string ROBOT_SEMANTIC_FREESPACE_PARAM = "robot_description_freespace_semantic";

    // Initial setup
    std::string urdf_xml_string, srdf_xml_string, urdf_xml_string_freespace, srdf_xml_string_freespace;
    nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
    nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
    nh.getParam(ROBOT_DESCRIPTION_FREESPACE_PARAM, urdf_xml_string_freespace);
    nh.getParam(ROBOT_SEMANTIC_FREESPACE_PARAM, srdf_xml_string_freespace);


    m_plan_cut = std::make_shared<TrajOptWireCuttingPlanProfile>();
    JointThreeAbsoluteLimitsConstraint cnt1(m_env_cut);
    JointTwoLimitsConstraint cnt2(m_env_cut);
    JointThreeLimitsConstraint cnt3(m_env_cut);
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> temp_function1 = cnt1;
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> temp_function2 = cnt2;
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> temp_function3 = cnt3;
    sco::VectorOfVector::func temp_1 = temp_function1;
    sco::VectorOfVector::func temp_2 = temp_function2;
    sco::VectorOfVector::func temp_3 = temp_function3;

    sco::ConstraintType a = sco::ConstraintType::EQ;
    Eigen::VectorXd error_coeff(1);
    error_coeff << 0.5;

    std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd> temp_tuple1(temp_1,nullptr,a,error_coeff);
    std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd> temp_tuple2(temp_2,nullptr,a,error_coeff);
    std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd> temp_tuple3(temp_3,nullptr,a,error_coeff);
    std::vector<std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd>>
    constraint_error_functions;

    constraint_error_functions.push_back(temp_tuple1);
    constraint_error_functions.push_back(temp_tuple2);
    constraint_error_functions.push_back(temp_tuple3);

    //m_plan_cut->constraint_error_functions = constraint_error_functions;
}

WireCuttingProblemGenerator::WireCuttingProblemGenerator(const Environment::Ptr env_cut,
                                const Environment::Ptr env_free,
                                const TrajOptWireCuttingPlanProfile::Ptr plan_cut,
                                const TrajOptPlanProfile::Ptr plan_free)
                                : m_env_cut(env_cut),
                                  m_env_free(env_free),
                                  m_plan_cut(plan_cut),
                                  m_plan_free(plan_free) { };


ProcessPlanningRequest WireCuttingProblemGenerator::construct_request_cut(const VectorIsometry3d& tool_poses)
{
  CompositeInstruction program("DEFAULT", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator"));
  
  assert(!tool_poses.empty());
  Waypoint wp = CartesianWaypoint(tool_poses[0]);
  PlanInstruction plan_instruction(wp, PlanInstructionType::START, "wire_cutting");
  plan_instruction.setDescription("from_start_plan");
  program.setStartInstruction(plan_instruction);

  for (std::size_t i = 1; i < tool_poses.size(); i++)
  {
    Waypoint wp = CartesianWaypoint(tool_poses[i]);
    PlanInstruction plan_instruction(wp, PlanInstructionType::LINEAR, "wire_cutting");
    plan_instruction.setDescription("waypoint_" + std::to_string(i)); 
    program.push_back(plan_instruction);
  }
  ProcessPlanningRequest request;
  request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  request.instructions = Instruction(program);

  return request;
}

ProcessPlanningRequest WireCuttingProblemGenerator::construct_request_freespace(const JointState& start, const JointState& end)
{
  /*for(auto joint : start.joint_names)
    std::cout << joint << std::endl;

  std::cout << std::endl << start.position << std::endl << std::endl;
  std::cout << end.position << std::endl;*/

  CompositeInstruction program("DEFAULT", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator"));

  Waypoint wp_start = StateWaypoint(start.joint_names, start.position);
  Waypoint wp_end = StateWaypoint(end.joint_names, end.position);

  PlanInstruction start_instruction(wp_start, PlanInstructionType::START, "wire_cutting");
  program.setStartInstruction(start_instruction);

  PlanInstruction plan_f0(wp_end, PlanInstructionType::FREESPACE, "wire_cutting");

  program.push_back(plan_f0);

  ProcessPlanningRequest request;
  request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  request.instructions = Instruction(program);

  return request;
}