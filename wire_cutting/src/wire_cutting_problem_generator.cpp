#include <wire_cutting_problem_generator.h>



WireCuttingProblemGenerator::WireCuttingProblemGenerator(tesseract_monitoring::EnvironmentMonitor::Ptr monitor)
{
  monitor_ = monitor;
}


ProcessPlanningRequest WireCuttingProblemGenerator::construct_request_cut(const VectorIsometry3d& tool_poses, const Environment::ConstPtr& env)
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
  request.env_state = env->getCurrentState();
  request.seed = tesseract_planning::generateSeed(program, env->getCurrentState(), env);
  
  /*tesseract_planning::CompositeInstruction naive_seed;
  {
    auto lock = monitor_->lockEnvironmentRead();
    naive_seed = tesseract_planning::generateNaiveSeed(program, *(monitor_->getEnvironment()));
  }
  request.seed = Instruction(naive_seed);*/

  return request;
}

ProcessPlanningRequest WireCuttingProblemGenerator::construct_request_cut_descartes(const VectorIsometry3d& tool_poses)
{
  CompositeInstruction program("DESCARTES", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator"));
  
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

ProcessPlanningRequest WireCuttingProblemGenerator::construct_request_p2p(const JointState& start, const JointState& end)
{
  std::cout << start.position << std::endl;
  std::cout << end.position << std::endl << std::endl;
  CompositeInstruction program("FREESPACE", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator"));


  Waypoint wp_start = StateWaypoint(start.joint_names, start.position);
  Waypoint wp_end = StateWaypoint(start.joint_names, end.position);

  PlanInstruction start_instruction(wp_start, PlanInstructionType::START, "FREESPACE");
  program.setStartInstruction(start_instruction);

  PlanInstruction plan_end(wp_end, PlanInstructionType::FREESPACE, "FREESPACE");

  program.push_back(plan_end);

  ProcessPlanningRequest request;
  request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  request.instructions = Instruction(program);

  return request;
}