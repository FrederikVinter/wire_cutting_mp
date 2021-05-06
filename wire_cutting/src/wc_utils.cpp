#include <wc_utils.h>
#include <fstream>
#include <ros/package.h>
#include <iostream>

std::vector<tesseract_common::VectorIsometry3d> loadToolPosesFromPrg(const std::string& file)
{
  std::vector<tesseract_common::VectorIsometry3d> path;  // results
  tesseract_common::VectorIsometry3d segment;
  std::ifstream indata;                     // input file


  std::string filename = ros::package::getPath("wire_cutting") + "/config/" + file + ".prg";

  indata.open(filename);
  assert(indata.is_open());

  std::string line;
  int movel_instructions = 0;
  int segment_num = 0;
  while (std::getline(indata, line))
  {
    std::stringstream lineStream(line);
    std::string cell;
    Eigen::Matrix<double, 7, 1> xyzWXYZ;

    bool movel_not_found = 1;
    while (std::getline(lineStream, cell, ' ') && movel_not_found == 1)
    {
      if(cell == "PROC" && !segment.empty())
      {
        if(!segment.empty())
          path.push_back(segment);
        segment.clear();
      }
      if(!(cell == "MoveL"))
        continue;  
      movel_not_found = 0;  
      int i = 0;
      while (std::getline(lineStream, cell, ',') && i < xyzWXYZ.size())
      {
        cell.erase(std::remove(cell.begin(), cell.end(), '['), cell.end()); 
        cell.erase(std::remove(cell.begin(), cell.end(), ']'), cell.end());
        xyzWXYZ(i) = std::stod(cell);
        i++;
      }
    }
    
    if(movel_not_found == 0) {
      movel_instructions++;
      if(movel_instructions >= 0){
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity() * Eigen::Translation3d(xyzWXYZ(0) / 1000.0, xyzWXYZ(1) / 1000.0 , xyzWXYZ(2) / 1000.0 ) *
                                      Eigen::Quaterniond(xyzWXYZ(3), xyzWXYZ(4), xyzWXYZ(5), xyzWXYZ(6));
        if(movel_instructions%100 == 0)
          segment.push_back(pose);
      }
    }
  }
  if(!segment.empty())
    path.push_back(segment);
  indata.close();
  std::cout << "PATH SIZE: " << path.size() << std::endl;
  for(auto seg : path)
    std::cout << seg.size() << std::endl;

  return path;
}


PathData loadToolPosesCFR(std::string file)
{
  PathData pathData;
  tesseract_common::VectorIsometry3d segment;

  std::ifstream indata;                     // input file

  // You could load your parts from anywhere, but we are transporting them with
  std::string filename = ros::package::getPath("wire_cutting") + "/config/" + file;
  indata.open(filename);
  assert(indata.is_open());
  
  std::string line;
  while (std::getline(indata, line))
  {
    std::stringstream lineStream(line);
    std::string cell;
    Eigen::Matrix<double, 7, 1> xyzWXYZ;


    std::getline(lineStream, cell, ',');
    
      if(cell == "bbox")
      {
        pathData.has_bbox = true;
        int i = 0;
        while (std::getline(lineStream, cell, ','))
        {
          assert(i < 6);
          if(i < 3)
            pathData.bbox_pos(i) = (std::stod(cell)/1000);
          else
            pathData.bbox_size(i-3) = std::stod(cell)/1000;
          i++;
        }
      }
      else if(cell == "p2p")
      {
        if(!segment.empty())
          pathData.path.push_back(segment);
        segment.clear();
      }
      else if(cell == "cut")
      {
        int i = 0;
        while (std::getline(lineStream, cell, ',') && i < xyzWXYZ.size())
        {
          assert(i < 7);
          xyzWXYZ(i) = std::stod(cell);
          i++;
        }
        //std::cout << "Pose: " << std::endl << xyzWXYZ << std::endl;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity() * Eigen::Translation3d(xyzWXYZ(0), xyzWXYZ(1), xyzWXYZ(2)) *
                                   Eigen::Quaterniond(xyzWXYZ(3), xyzWXYZ(4), xyzWXYZ(5), xyzWXYZ(6));
   
        segment.push_back(pose);
      }
      else
      {
        std::cout << "Invalid type: " << cell << std::endl;
      }
  }
  indata.close();

  // Last segment
  if(!segment.empty())
  {
    pathData.path.push_back(segment);
    segment.clear();
  }

  // Tesseract uses pos as middle of box
  if(pathData.has_bbox)
  {
    pathData.bbox_pos(0) = pathData.bbox_pos(0) + pathData.bbox_size(0)/2;
    pathData.bbox_pos(1) = pathData.bbox_pos(1) + pathData.bbox_size(1)/2;
    pathData.bbox_pos(2) = pathData.bbox_pos(2) + pathData.bbox_size(2)/2;
  }

  std::cout << "PATH SIZE: " << pathData.path.size() << std::endl;
  for(auto seg : pathData.path)
    std::cout << seg.size() << std::endl;

  return pathData;
}

tesseract_common::VectorIsometry3d loadToolPoses()
{
  tesseract_common::VectorIsometry3d path;  // results
  std::ifstream indata;                     // input file

  // You could load your parts from anywhere, but we are transporting them with
  std::string filename = ros::package::getPath("wire_cutting") + "/config/tool_poses.csv";

  indata.open(filename);
  assert(indata.is_open());

  std::string line;
  int lnum = 0;
  while (std::getline(indata, line))
  {
    ++lnum;
    if (lnum < 3)
      continue;

    std::stringstream lineStream(line);
    std::string cell;
    Eigen::Matrix<double, 7, 1> xyzWXYZ;
    int i = -2;
    while (std::getline(lineStream, cell, ','))
    {
      ++i;
      if (i == -1)
        continue;

      xyzWXYZ(i) = std::stod(cell);
    }


    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity() * Eigen::Translation3d(xyzWXYZ(0), xyzWXYZ(1), xyzWXYZ(2)) *
                                   Eigen::Quaterniond(xyzWXYZ(3), xyzWXYZ(4), xyzWXYZ(5), 0);
   
    path.push_back(pose);
  }
  indata.close();

  return path;
}

std::vector<std::vector<Eigen::VectorXd>> loadOptimizationResults(std::string path)
{
    std::vector<std::vector<Eigen::VectorXd>> vector_of_paths;  // results
    std::ifstream indata;      // input file

    // std::string filename = ros::package::getPath("wire_cutting") + "/config/trajopt_vars.log";
    std::string filename = path;

    indata.open(filename);
    assert(indata.is_open());

    std::string line;
    int lnum = 0;
    while (std::getline(indata, line))
    {
        std::vector<Eigen::VectorXd> path;
        Eigen::VectorXd joint_pos(6);
        ++lnum;
        if (lnum < 2) // skip first line
            continue;

        std::stringstream lineStream(line);
        std::string cell;
        int i = -2;
        while (std::getline(lineStream, cell, ','))
        {
            ++i;
            if (i == -1)
                continue;
            if (i != 0 && i % 6 == 0) {
                path.push_back(joint_pos);
                joint_pos.fill(0);
            }
            joint_pos(i%6) = std::stod(cell);
        }
        vector_of_paths.push_back(path);
    }
    indata.close();

    return vector_of_paths;
}

void plotIterations(const tesseract_environment::Environment::Ptr& env, const tesseract_rosutils::ROSPlottingPtr& plotter, const std::vector<std::string>& joint_names , std::string path) {
    std::vector<std::vector<Eigen::VectorXd>> opt_joint_results = loadOptimizationResults(path);
    for(size_t i = 0; i < opt_joint_results.size(); i++) {

      // Calculate poses
      auto kin = env->getManipulatorManager()->getFwdKinematicSolver("manipulator");
      tesseract_common::VectorIsometry3d opt_poses;
      for (size_t j = 0; j < opt_joint_results[i].size(); ++j) {
        Eigen::Isometry3d opt_pose;
        Eigen::Vector3d tcp_pose(0,0,1.861);
        kin->calcFwdKin(opt_pose, opt_joint_results[i][j]);
        opt_pose.translate(tcp_pose);
        opt_poses.push_back(opt_pose);
      }

      // Create toolpath
      tesseract_common::Toolpath opt_toolpath;
      opt_toolpath.push_back(opt_poses);

      // Plot toolpath
      if (plotter != nullptr && plotter->isConnected())
      {
        std::cout << "Iteration: " << i << "\n";
        plotter->plotMarker(tesseract_visualization::ToolpathMarker(opt_toolpath));
        plotter->waitForInput();
        plotter->clear();
      }
    }

}

void calculateRotError(const tesseract_environment::Environment::Ptr& env,
                       const tesseract_rosutils::ROSPlottingPtr& plotter,
                       const std::vector<std::string>& joint_names,
                       std::string path) 
{
    std::vector<std::vector<Eigen::VectorXd>> opt_joint_results = loadOptimizationResults(path);
    std::vector<Eigen::VectorXd> final_iteration = opt_joint_results.back();

    // Calculate poses
    auto kin = env->getManipulatorManager()->getFwdKinematicSolver("manipulator");
    tesseract_common::VectorIsometry3d waypoints;

    for (size_t i = 0; i < final_iteration.size(); ++i) {
      Eigen::Isometry3d wp;
      Eigen::Vector3d tcp_pose(0,0,1.861);
      kin->calcFwdKin(wp, final_iteration[i]);
      wp.translate(tcp_pose);
      waypoints.push_back(wp);
    }


    // Create toolpath
    tesseract_common::Toolpath toolpath;
    toolpath.push_back(waypoints);

    for(size_t i = 0; i < waypoints.size(); i++) {
      std::cout << "WP " << i << "\n";
      // std::cout << "matrix:\n";
      // std::cout << waypoints[i].matrix() << "\n";

      Eigen::Matrix3d m = waypoints[i].rotation();
      // Eigen::Vector3d v = waypoints[i].translation();
      // std::cout << "Rotation: " << std::endl << m << std::endl;
      // std::cout << "Translation: " << std::endl << v << std::endl;
      Eigen::Vector3d wp_rpy = waypoints[i].rotation().eulerAngles(0, 1, 2);
      // std::cout << "Rotation RPY: " << std::endl << wp_rpy << std::endl;

    }

    // Plot toolpath
    if (plotter != nullptr && plotter->isConnected())
    {
      plotter->plotMarker(tesseract_visualization::ToolpathMarker(toolpath));
      plotter->waitForInput();
      plotter->clear();
    }
}

tesseract_common::VectorIsometry3d sampleToolAxis_WC(const Eigen::Isometry3d& tool_pose,
                                                    double resolution,
                                                    const Eigen::Vector3d& axis)
{
  tesseract_common::VectorIsometry3d samples;
  int cnt = static_cast<int>(std::ceil(2.0 * M_PI / resolution)) + 1;
  Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(cnt, -M_PI/2, M_PI/2);
  samples.reserve(static_cast<size_t>(angles.size()) - 1ul);
  for (long i = 0; i < static_cast<long>(angles.size() - 1); ++i)
  {
    Eigen::Isometry3d p = tool_pose * Eigen::AngleAxisd(angles(i), axis);
    samples.push_back(p);
  }
  return samples;
}

trajopt::TermInfo::Ptr createVelocityTermInfo(double max_displacement,
                                                int start_index,
                                                int end_index,
                                                const std::string& link,
                                                trajopt::TermType type,
                                                const Eigen::VectorXd& coeff,
                                                sco::PenaltyType penalty_type)
{
  if ((end_index - start_index) < 2)
    throw std::runtime_error("TrajOpt CartVelTermInfo requires at least two states!");

  std::shared_ptr<CartVelTermInfoWC> term = std::make_shared<CartVelTermInfoWC>();
  term->first_step = start_index;

  // end_index-1 is done since velocity requires step_i and step_i+1
  term->last_step = end_index-1;
  term->max_displacement = max_displacement;
  term->link = link;
  term->term_type = type;  
  term->coeffs = coeff;
  term->penalty_type = penalty_type;

  return term;
}

trajopt::TermInfo::Ptr createRotationalVelocityTermInfo(double max_displacement,
                                                //Eigen::Vector3d rot_coeffs,
                                                int start_index,
                                                int end_index,
                                                const std::string& link,
                                                trajopt::TermType type,
                                                const Eigen::VectorXd& coeff,
                                                sco::PenaltyType penalty_type)
{
  if ((end_index - start_index) < 2)
    throw std::runtime_error("TrajOpt CartVelTermInfo requires at least two states!");

  std::shared_ptr<CartRotVelTermInfo> term = std::make_shared<CartRotVelTermInfo>();
  term->first_step = start_index;

  // end_index-1 is done since velocity requires step_i and step_i+1
  term->last_step = end_index-1;
  term->max_displacement = max_displacement;
  //term->rot_coeffs = rot_coeffs;
  term->link = link;
  term->term_type = type;
  term->rot_coeffs = coeff;
  term->penalty_type = penalty_type;  

  return term;
}

trajopt::TermInfo::Ptr createCartesianWaypointTermInfoWC(const Eigen::Isometry3d& c_wp,
                                                       int index,
                                                       std::string working_frame,
                                                       Eigen::Isometry3d tcp,
                                                       const Eigen::VectorXd& coeffs,
                                                       std::string link,
                                                       trajopt::TermType type,
                                                       sco::PenaltyType penalty_type)
{
  auto pose_info = std::make_shared<CartPoseTermInfoWC>();
  pose_info->term_type = type;
  pose_info->name = "cartesian_waypoint_" + std::to_string(index);

  pose_info->link = link;
  pose_info->tcp = tcp;
  pose_info->penalty_type = penalty_type;

  pose_info->timestep = index;
  pose_info->xyz = c_wp.translation();
  Eigen::Quaterniond q(c_wp.linear());
  pose_info->wxyz = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
  pose_info->target = working_frame;

  if (coeffs.size() == 1)
  {
    pose_info->pos_coeffs = Eigen::Vector3d::Constant(coeffs(0));
    pose_info->rot_coeffs = Eigen::Vector3d::Constant(coeffs(0));
  }
  else if (coeffs.size() == 6)
  {
    pose_info->pos_coeffs = coeffs.head<3>();
    pose_info->rot_coeffs = coeffs.tail<3>();
  }

  return pose_info;
}

void saveInstructionsAsXML(const std::vector<const tesseract_planning::CompositeInstruction*>& cis) 
{
    for(size_t i = 0; i < cis.size(); i++) {
      tinyxml2::XMLDocument xmlDoc;
      tinyxml2::XMLNode * pRoot = xmlDoc.NewElement("Instructions");    // Creat root element
      xmlDoc.InsertFirstChild(pRoot);                         // Insert element

      tinyxml2::XMLElement* pResults = cis[i]->toXML(xmlDoc);
      pRoot->InsertEndChild(pResults);                        // insert element as child

      // save results
      std::string fileName_string = ros::package::getPath("wire_cutting") + "/test/test_1/path_" + std::to_string(i) + ".xml";
      const char* fileName = fileName_string.c_str();
      std::cout << "Saved at path: " << fileName << "\n";

      //const char* fileName = "/home/jonathan/projects/wirecutting_ws/src/wire_cutting_mp/wire_cutting/test/test_1.xml";
      tinyxml2::XMLError eResult = xmlDoc.SaveFile(fileName);
  }
}

std::vector<std::vector<std::vector<Isometry3d>>> loadInstructionsFromXML(const std::vector<const tesseract_planning::CompositeInstruction*>& cis, const tesseract_environment::Environment::Ptr& env)
{
  using namespace tinyxml2;

  // SEGMENTS
  //  -> WAYPOINTS
  //  --> COORDINATES    

    std::vector<std::vector<std::vector<Isometry3d>>> segment_coordinates;

    for(size_t i = 0; i < cis.size(); i++) {
      std::vector<std::vector<Isometry3d>> wp_to_wp;
      tinyxml2::XMLError status;

      tinyxml2::XMLDocument results;
      std::string fileName = ros::package::getPath("wire_cutting") + "/test/test_1/path_" + std::to_string(i) + ".xml";
      results.LoadFile(fileName.c_str());

      // navigate to root of all instructions
      XMLElement* instructions_root = results.FirstChildElement("Instructions")->FirstChildElement("Instruction")->FirstChildElement("CompositeInstruction");

      // Position of 1st instruction of waypoint_1
      XMLElement* ci = instructions_root->FirstChildElement("Instruction");

      while(ci) {
        std::vector<Isometry3d> coordinates; // coordinates for each waypoint_N

        // Navigate to position of 1st instruction
        XMLElement* instruction =  ci->FirstChildElement("CompositeInstruction")->FirstChildElement("Instruction");

        // Get all sibilings
        while(instruction) {
          XMLElement* instruction_pos = instruction->FirstChildElement("MoveInstruction")->FirstChildElement("Waypoint")->FirstChildElement("StateWaypoint")->FirstChildElement("Position");

          // Print position
          std::string pos;
          status = tesseract_common::QueryStringText(instruction_pos, pos);
          if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
            throw std::runtime_error("loadInstructionsAsXML failed: entry 'Position' not found");


          // Put joint angles in vector
          auto iss = std::istringstream{pos};
          auto str = std::string{};
          VectorXd joint_angles(6);
          int j = 0;
          while (iss >> str) {
            joint_angles(j) = std::stod(str);
            j++;
          }

          // use fwd kin to calculate cartesian coordinates
          Eigen::Isometry3d cartesian_coordinates;
          auto kin = env->getManipulatorManager()->getFwdKinematicSolver("manipulator");
          kin->calcFwdKin(cartesian_coordinates, joint_angles);
          Eigen::Vector3d tcp_pose(0,0,1.861);
          cartesian_coordinates.translate(tcp_pose);
          coordinates.push_back(cartesian_coordinates);

          instruction = instruction->NextSiblingElement(); // get next instruction
        }
        wp_to_wp.push_back(coordinates);

        ci = ci->NextSiblingElement(); //advance from waypoint_N -> waypoint_N+1
      }
      segment_coordinates.push_back(wp_to_wp);
    }
    return segment_coordinates;
}