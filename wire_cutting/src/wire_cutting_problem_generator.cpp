#include <wire_cutting_problem_generator.h>



WireCuttingProblemGenerator::WireCuttingProblemGenerator()
{

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

CompositeInstruction WireCuttingProblemGenerator::generate_descartes_seed(const VectorIsometry3d& tool_poses,
                                                                    ManipulatorInfo& mi, 
                                                                    const tesseract_environment::Environment::Ptr& env,
                                                                    std::shared_ptr<DescartesDefaultPlanProfile<double>> descartes_plan_profile)
{
  CompositeInstruction program("DESCARTES_CUT", CompositeInstructionOrder::ORDERED, mi);

  assert(!tool_poses.empty());
  Waypoint wps = CartesianWaypoint(tool_poses[0]);
  PlanInstruction plan_instruction(wps, PlanInstructionType::START, "DESCARTES_CUT");
  plan_instruction.setDescription("from_start_plan");
  program.setStartInstruction(plan_instruction);

  std::cout << "Start point: "  << std::endl;
  wps.print();

  for (std::size_t i = 1; i < tool_poses.size(); i++)
  {
    Waypoint wp = CartesianWaypoint(tool_poses[i]);
    PlanInstruction plan_instruction(wp, PlanInstructionType::LINEAR, "DESCARTES_CUT");
    plan_instruction.setDescription("waypoint_" + std::to_string(i)); 
    program.push_back(plan_instruction);

    if(i ==  tool_poses.size()-1) {
          std::cout << "End point: "  << std::endl;
          wp.print();
    }
  }

  // Create Planning Request
  PlannerRequest request;
  request.seed = generateSeed(program, env->getCurrentState(), env);
  request.instructions = program;
  request.env = env;
  request.env_state = env->getCurrentState();
  
  // Solve Descartes Plan
  PlannerResponse descartes_response;
  DescartesMotionPlannerD descartes_planner;
  descartes_planner.plan_profiles["DESCARTES_CUT"] = descartes_plan_profile;
  descartes_planner.problem_generator = tesseract_planning::DefaultDescartesProblemGenerator<double>;
  auto descartes_status = descartes_planner.solve(request, descartes_response);

  if(descartes_status) {
    ROS_INFO("Descartes succeeded");
  } else {
    ROS_INFO("Descartes failed");
  }

  return descartes_response.results;
}

CompositeInstruction WireCuttingProblemGenerator::generate_ompl_seed(const VectorIsometry3d& tool_poses,
                                                                     ManipulatorInfo& mi, 
                                                                     const tesseract_environment::Environment::Ptr& env,
                                                                     std::shared_ptr<tesseract_planning::OMPLDefaultPlanProfile> ompl_plan_profile)
{
  CompositeInstruction program("OMPL", CompositeInstructionOrder::ORDERED, mi);

  assert(!tool_poses.empty());
  Waypoint wps = CartesianWaypoint(tool_poses[0]);
  PlanInstruction plan_instruction(wps, PlanInstructionType::START, "OMPL");
  plan_instruction.setDescription("from_start_plan");
  program.setStartInstruction(plan_instruction);

  std::cout << "Start point: "  << std::endl;
  wps.print();

  Waypoint wpe = CartesianWaypoint(tool_poses.back());
  PlanInstruction plan_instruction_e(wpe, PlanInstructionType::FREESPACE, "OMPL");
  plan_instruction.setDescription("end point"); 
  program.push_back(plan_instruction_e);

  std::cout << "End point: "  << std::endl;
  wpe.print();

  // Create Planning Request
  PlannerRequest request;
  request.seed = generateSeed(program, env->getCurrentState(), env);
  request.instructions = program;
  request.env = env;
  request.env_state = env->getCurrentState();
  
  // Solve Descartes Plan
  PlannerResponse ompl_response;
  OMPLMotionPlanner ompl_planner;
  ompl_planner.plan_profiles["OMPL"] = ompl_plan_profile;
  ompl_planner.problem_generator = tesseract_planning::DefaultOMPLProblemGenerator;
  auto status = ompl_planner.solve(request, ompl_response);

  if(status) {
    ROS_INFO("OMPL succeeded");
  } else {
    ROS_INFO("OMPL failed");
  }

  return ompl_response.results;
}

CompositeInstruction WireCuttingProblemGenerator::generate_conf_interpolated_seed(const VectorIsometry3d& tool_poses,
                                                                                  ManipulatorInfo& mi, 
                                                                                  const tesseract_environment::Environment::Ptr& env)
{
  // Change IK solver to KDL
  std::vector<std::string>  ik_solvers = env->getManipulatorManager()->getAvailableInvKinematicsSolvers();
  std::cout << "Available solver:" << "\n";
  for (const auto& solver : ik_solvers) { // search for it
      std::cout << solver << "\n";
      if (solver == "KDLInvKinChainLMA") {
        env->getManipulatorManager()->setDefaultInvKinematicSolver("manipulator", "KDLInvKinChainLMA");
        std::cout << "Using KDLInvKinChainLMA to generate configuration-interpolated seed\n";
      }
  }

  CompositeInstruction program("DEFAULT", CompositeInstructionOrder::ORDERED, mi);

  assert(!tool_poses.empty());
  Waypoint wps = CartesianWaypoint(tool_poses[0]);
  PlanInstruction plan_instruction(wps, PlanInstructionType::START, "DEFAULT");
  plan_instruction.setDescription("from_start_plan");
  program.setStartInstruction(plan_instruction);

  std::cout << "Start point: "  << std::endl;
  wps.print();

  for (std::size_t i = 1; i < tool_poses.size(); i++)
  {
    Waypoint wp = CartesianWaypoint(tool_poses[i]);
    PlanInstruction plan_instruction(wp, PlanInstructionType::LINEAR, "DEFAULT");
    plan_instruction.setDescription("waypoint_" + std::to_string(i)); 
    program.push_back(plan_instruction);

    if(i ==  tool_poses.size()-1) {
          std::cout << "End point: "  << std::endl;
          wp.print();
    }
  }

  CompositeInstruction seed = generateSeed(program, env->getCurrentState(), env);
  env->getManipulatorManager()->setDefaultInvKinematicSolver("manipulator", "OPWInvKin"); // reset solver back to OPW

  return seed;
}

CompositeInstruction WireCuttingProblemGenerator::generate_naive_seed(const VectorIsometry3d& tool_poses,
                                                                                  ManipulatorInfo& mi, 
                                                                                  const tesseract_environment::Environment::Ptr& env)
{
  CompositeInstruction program("DEFAULT", CompositeInstructionOrder::ORDERED, mi);

  assert(!tool_poses.empty());
  Waypoint wps = CartesianWaypoint(tool_poses[0]);
  PlanInstruction plan_instruction(wps, PlanInstructionType::START, "DEFAULT");
  plan_instruction.setDescription("from_start_plan");
  program.setStartInstruction(plan_instruction);

  std::cout << "Start point: "  << std::endl;
  wps.print();

  for (std::size_t i = 1; i < tool_poses.size(); i++)
  {
    Waypoint wp = CartesianWaypoint(tool_poses[i]);
    PlanInstruction plan_instruction(wp, PlanInstructionType::LINEAR, "DEFAULT");
    plan_instruction.setDescription("waypoint_" + std::to_string(i)); 
    program.push_back(plan_instruction);

    if(i ==  tool_poses.size()-1) {
          std::cout << "End point: "  << std::endl;
          wp.print();
    }
  }

  CompositeInstruction seed = generateNaiveSeed(program, *env);

  return seed;
}


ProcessPlanningRequest WireCuttingProblemGenerator::construct_request_p2p(const JointState& start,
                                                                          const JointState& end, 
                                                                          const std::string& planner_name, 
                                                                          ManipulatorInfo& mi)
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