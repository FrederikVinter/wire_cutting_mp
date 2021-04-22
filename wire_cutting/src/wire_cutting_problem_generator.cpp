#include <wire_cutting_problem_generator.h>


WireCuttingProblemGenerator::WireCuttingProblemGenerator()
{
    m_plan_cut = std::make_shared<TrajOptWireCuttingPlanProfile>();
    JointThreeAbsoluteLimitsConstraint cnt1;
    JointTwoLimitsConstraint cnt2;
    JointThreeLimitsConstraint cnt3;
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

    m_plan_cut->constraint_error_functions = constraint_error_functions;
}


ProcessPlanningRequest WireCuttingProblemGenerator::construct_request_cut(const VectorIsometry3d& tool_poses, ManipulatorInfo& mi)
{
  CompositeInstruction program("DEFAULT", CompositeInstructionOrder::ORDERED, mi);
  
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

ProcessPlanningRequest WireCuttingProblemGenerator::construct_request_cut_descartes(const VectorIsometry3d& tool_poses, ManipulatorInfo& mi)
{
  CompositeInstruction program("DESCARTES", CompositeInstructionOrder::ORDERED, mi);
  
  assert(!tool_poses.empty());
  Waypoint wp = CartesianWaypoint(tool_poses[0]);
  PlanInstruction plan_instruction(wp, PlanInstructionType::START, "DESCARTES");
  plan_instruction.setDescription("from_start_plan");
  program.setStartInstruction(plan_instruction);

    std::cout << "Start point: "  << std::endl;
    wp.print();


  for (std::size_t i = 1; i < tool_poses.size(); i++)
  {
    Waypoint wp = CartesianWaypoint(tool_poses[i]);
    PlanInstruction plan_instruction(wp, PlanInstructionType::LINEAR, "DESCARTES");
    plan_instruction.setDescription("waypoint_" + std::to_string(i)); 
    program.push_back(plan_instruction);

    if(i ==  tool_poses.size()-1) {
          std::cout << "End point: "  << std::endl;
          wp.print();
    }
  }
  ProcessPlanningRequest request;
  request.name = tesseract_planning::process_planner_names::DESCARTES_PLANNER_NAME;
  request.instructions = Instruction(program);

  return request;
}

ProcessPlanningRequest WireCuttingProblemGenerator::construct_request_p2p(const JointState& start, const JointState& end, const std::string& planner_name, ManipulatorInfo& mi)
{
  std::cout << start.position << std::endl << std::endl;;
  std::cout << end.position << std::endl << std::endl;
  CompositeInstruction program(planner_name, CompositeInstructionOrder::ORDERED, mi);

  Waypoint wp_start = StateWaypoint(start.joint_names, start.position);
  Waypoint wp_end = StateWaypoint(start.joint_names, end.position);

  PlanInstruction start_instruction(wp_start, PlanInstructionType::START, planner_name);
  program.setStartInstruction(start_instruction);

  PlanInstruction plan_end(wp_end, PlanInstructionType::FREESPACE, planner_name);

  program.push_back(plan_end);

  ProcessPlanningRequest request;
  if (planner_name == "FREESPACE_TRAJOPT") {
    request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  }
  else if(planner_name == "FREESPACE_OMPL") {
    request.name = "OMPL_NO_POST_CHECK";
  }
  request.instructions = Instruction(program);

  return request;
}

bool WireCuttingProblemGenerator::run_request_p2p(std::vector<ProcessPlanningRequest>& p2p_requests, 
                                                  const tesseract_rosutils::ROSPlottingPtr& plotter,
                                                  ProcessPlanningServer& planning_server_freespace,
                                                  std::vector<ProcessPlanningFuture>& p2p_responses) {
  
  plotter->waitForInput();

  std::size_t p2p_moves = p2p_requests.size();
  p2p_responses.resize(p2p_moves);

  ROS_INFO("Solve process plans for point to point");
  // Solve process plans for point to point
  assert(p2p_requests.size() == p2p_moves);
  for(std::size_t i = 0; i < p2p_requests.size(); i++) {
    p2p_responses[i] = planning_server_freespace.run(p2p_requests[i]);
  }
  planning_server_freespace.waitForAll();

  // check if planning was succesful
  for(std::size_t i = 0; i < p2p_moves; i++) {
    if(p2p_responses[i].interface->isAborted()) {
      ROS_INFO("run_request_p2p: 1 or more requests did not succeed");
      p2p_requests.clear();
      p2p_responses.clear();
      return false;
    }
  }
  return true;
}