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
  std::string filename = ros::package::getPath("wire_cutting") + "/programs/" + file;
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

tesseract_common::VectorIsometry3d sampleToolAxis_WC(const Eigen::Isometry3d& tool_pose,
                                                    double resolution,
                                                    const Eigen::Vector3d& axis)
{
  tesseract_common::VectorIsometry3d samples;
  int cnt = static_cast<int>(std::ceil(2.0 * M_PI / resolution)) + 1;
  Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(cnt, -M_PI/4, M_PI/4);
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

trajopt::TermInfo::Ptr createCartAccelTermInfo(std::vector<double> displacements,
                                                int start_index,
                                                int end_index,
                                                const std::string& link,
                                                trajopt::TermType type,
                                                const Eigen::VectorXd& coeff,
                                                sco::PenaltyType penalty_type)
{
  if ((end_index - start_index) < 3)
    throw std::runtime_error("TrajOpt CartAccTermInfo requires at least three states!");

  std::shared_ptr<CartAccTermInfoWC> term = std::make_shared<CartAccTermInfoWC>();
  
  term->first_step = start_index+1;
  term->last_step = end_index-1;
  term->displacements = displacements;

  term->link = link;
  term->term_type = type;
  term->coeffs = coeff;
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

void loadTestData(TestType &test_type,
                bool &show_iterations,
                const std::string &test_name, 
                InitMethodCut &init_method_cut, 
                Methodp2p &method_p2p, 
                Isometry3d &p2p_start, 
                Isometry3d &p2p_end,
                bool &bbox,
                VectorXd &bbox_pos,
                VectorXd &bbox_size)
{
  tinyxml2::XMLDocument test_def;
  std::string test_def_s = ros::package::getPath("wire_cutting") + "/test/" + test_name + "/test_def.xml";
  std::cout << "Test def path: " << test_def_s << std::endl;
  test_def.LoadFile(test_def_s.c_str());
  tinyxml2::XMLElement* test_element = test_def.FirstChildElement("Test");

  if(!test_element)
  {
    throw std::runtime_error("Wrong test def path or incomplete file");
  }

  const tinyxml2::XMLElement* init_method = test_element->FirstChildElement("InitMethodCut");
  const tinyxml2::XMLElement* iterationsDebug = test_element->FirstChildElement("ShowIterations");
  const tinyxml2::XMLElement* p2p = test_element->FirstChildElement("p2p");

  tinyxml2::XMLError status;

  if(!test_element)
  {
    throw std::runtime_error("Test must be defined");
  } else {
      int temp;
      status = test_element->QueryIntAttribute("type", &temp);
      if (status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("test type was not read correctly");

      test_type = static_cast<TestType>(temp);
      std::cout << "Test type: " << test_type << std::endl;
  }

  if(init_method)
  {
    int temp;
    status = init_method->QueryIntAttribute("type", &temp);
    if (status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("init method cut was not read correctly");

    init_method_cut = static_cast<InitMethodCut>(temp);
    std::cout << "Init method cut: " << init_method_cut << std::endl;
  }

  if(iterationsDebug)
  {
    tinyxml2::XMLError status = iterationsDebug->QueryBoolText(&show_iterations);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("Test def Error parsing show iterations");

    std::cout << "Show iterations: " << show_iterations << std::endl;
  }

  if(p2p)
  {
    const tinyxml2::XMLElement* method = p2p->FirstChildElement("Method");
    const tinyxml2::XMLElement* p2p_start_ele = p2p->FirstChildElement("Start");
    const tinyxml2::XMLElement* p2p_end_ele = p2p->FirstChildElement("End");
    const tinyxml2::XMLElement* bbox_enabled = p2p->FirstChildElement("BboxEnabled");
    const tinyxml2::XMLElement* bbox_pos_el = p2p->FirstChildElement("BboxPos");
    const tinyxml2::XMLElement* bbox_size_el = p2p->FirstChildElement("BboxSize");

    if(!method)
      throw std::runtime_error("p2p must contain method!");
    else {
      int temp;
      status = method->QueryIntAttribute("type", &temp);
      if (status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("method p2p was not read correctly");
      method_p2p = static_cast<Methodp2p>(temp);

      std::cout << "Method p2p: " << method_p2p << std::endl;
    }


  if(bbox_enabled)
  {
    tinyxml2::XMLError status = bbox_enabled->QueryBoolText(&bbox);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("Test def Error parsing show iterations");

    std::cout << "bbox: " << bbox << std::endl;
  }

  if(bbox_pos_el)
  {
    std::vector<std::string> start_tokens;
    std::string start_string;
    status = tesseract_common::QueryStringText(bbox_pos_el, start_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("Error parsing p2p start");

    boost::split(start_tokens, start_string, boost::is_any_of(" "), boost::token_compress_on);

    std::size_t length = start_tokens.size();
    if(length != 3)
      throw std::runtime_error("bbox pos must contain 3 values");

    if (!tesseract_common::isNumeric(start_tokens))
      throw std::runtime_error("bbox pos are not all numeric values.");

    std::vector<double> temp;
    temp.resize(static_cast<long>(length));
    for (std::size_t i = 0; i < start_tokens.size(); ++i)
      tesseract_common::toNumeric<double>(start_tokens[i], temp[static_cast<long>(i)]);


    bbox_pos.resize(3);
    bbox_pos << temp[0], temp[1], temp[2];
  }

  if(bbox_size_el)
  {
    std::vector<std::string> start_tokens;
      std::string start_string;
      status = tesseract_common::QueryStringText(bbox_size_el, start_string);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("Error parsing p2p start");

      boost::split(start_tokens, start_string, boost::is_any_of(" "), boost::token_compress_on);

      std::size_t length = start_tokens.size();
      if(length != 3)
        throw std::runtime_error("bbox pos must contain 3 values");

      if (!tesseract_common::isNumeric(start_tokens))
        throw std::runtime_error("bbox pos are not all numeric values.");

      std::vector<double> temp;
      temp.resize(static_cast<long>(length));
      for (std::size_t i = 0; i < start_tokens.size(); ++i)
        tesseract_common::toNumeric<double>(start_tokens[i], temp[static_cast<long>(i)]);

      bbox_size.resize(3);
      bbox_size << temp[0], temp[1], temp[2];
  }

    

    if(!p2p_start_ele)
      throw std::runtime_error("p2p must contain start!");
    else {
      std::vector<std::string> start_tokens;
      std::string start_string;
      status = tesseract_common::QueryStringText(p2p_start_ele, start_string);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("Error parsing p2p start");

      boost::split(start_tokens, start_string, boost::is_any_of(" "), boost::token_compress_on);

      std::size_t length = start_tokens.size();
      if(length != 7)
        throw std::runtime_error("Start must contain 6 values");

      if (!tesseract_common::isNumeric(start_tokens))
        throw std::runtime_error("p2p start are not all numeric values.");

      std::vector<double> temp;
      temp.resize(static_cast<long>(length));
      for (std::size_t i = 0; i < start_tokens.size(); ++i)
        tesseract_common::toNumeric<double>(start_tokens[i], temp[static_cast<long>(i)]);
        
      p2p_start = Eigen::Isometry3d::Identity() * Eigen::Translation3d(temp[0], temp[1], temp[2]) * Eigen::Quaterniond(temp[3], temp[4], temp[5], temp[6]);

      std::cout << "Start: " << std::endl << "Translation:" << p2p_start.translation() << std::endl;
      std::cout << "Rotation: " << p2p_start.rotation() << std::endl;
      
    }

    if(!p2p_end_ele)
      throw std::runtime_error("p2p must contain end!");
    else {
      std::vector<std::string> end_tokens;
      std::string end_string;
      status = tesseract_common::QueryStringText(p2p_end_ele, end_string);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("Error parsing p2p end");

      boost::split(end_tokens, end_string, boost::is_any_of(" "), boost::token_compress_on);

      std::size_t length = end_tokens.size();
      if(length != 7)
        throw std::runtime_error("End must contain 6 values");

      if (!tesseract_common::isNumeric(end_tokens))
        throw std::runtime_error("p2p end are not all numeric values.");

      std::vector<double> temp;
      temp.resize(static_cast<long>(length));
      for (std::size_t i = 0; i < end_tokens.size(); ++i)
        tesseract_common::toNumeric<double>(end_tokens[i], temp[static_cast<long>(i)]);
        
      p2p_end = Eigen::Isometry3d::Identity() * Eigen::Translation3d(temp[0], temp[1], temp[2]) * Eigen::Quaterniond(temp[3], temp[4], temp[5], temp[6]);

      std::cout << "End: " << std::endl << "Translation:" << p2p_end.translation() << std::endl;
      std::cout << "Rotation: " << p2p_end.rotation() << std::endl;
    }

  }

}

void saveInstructionsAsXML(const std::vector<const tesseract_planning::CompositeInstruction*>& cis, std::string test_name) 
{
    for(size_t i = 0; i < cis.size(); i++) {
      tinyxml2::XMLDocument xmlDoc;
      tinyxml2::XMLNode * pRoot = xmlDoc.NewElement("Instructions");    // Creat root element
      xmlDoc.InsertFirstChild(pRoot);                         // Insert element

      tinyxml2::XMLElement* pResults = cis[i]->toXML(xmlDoc);
      pRoot->InsertEndChild(pResults);                        // insert element as child

      // save results
      std::string fileName_string = ros::package::getPath("wire_cutting") + "/test/" + test_name +  "/paths/path_" + std::to_string(i) + ".xml";
      const char* fileName = fileName_string.c_str();
      std::cout << "Saved at path: " << fileName << "\n";

      //const char* fileName = "/home/jonathan/projects/wirecutting_ws/src/wire_cutting_mp/wire_cutting/test/test_1.xml";
      tinyxml2::XMLError eResult = xmlDoc.SaveFile(fileName);
  }
}

std::vector<std::vector<std::vector<Isometry3d>>> loadInstructionsFromXML(const std::vector<const tesseract_planning::CompositeInstruction*>& cis,
                                                                          const tesseract_environment::Environment::Ptr& env, 
                                                                          std::string test_name)
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
      std::string fileName = ros::package::getPath("wire_cutting") + "/test/" + test_name +  "/paths/path_" + std::to_string(i) + ".xml";
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

std::vector<std::vector<std::vector<VectorXd>>> loadJointAnglesFromXML(const std::vector<const tesseract_planning::CompositeInstruction*>& cis,
                                                                          const tesseract_environment::Environment::Ptr& env, 
                                                                          std::string test_name)
{
  using namespace tinyxml2;

  // SEGMENTS
  //  -> WAYPOINTS
  //  --> COORDINATES    

    std::vector<std::vector<std::vector<VectorXd>>> segment_coordinates;

    for(size_t i = 0; i < cis.size(); i++) {
      std::vector<std::vector<VectorXd>> wp_to_wp;
      tinyxml2::XMLError status;

      tinyxml2::XMLDocument results;
      std::string fileName = ros::package::getPath("wire_cutting") + "/test/" + test_name +  "/paths/path_" + std::to_string(i) + ".xml";
      results.LoadFile(fileName.c_str());

      // navigate to root of all instructions
      XMLElement* instructions_root = results.FirstChildElement("Instructions")->FirstChildElement("Instruction")->FirstChildElement("CompositeInstruction");

      // Position of 1st instruction of waypoint_1
      XMLElement* ci = instructions_root->FirstChildElement("Instruction");

      while(ci) {
        std::vector<VectorXd> joint_config; // joint config for each waypoint_N

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
          joint_config.push_back(joint_angles);

          instruction = instruction->NextSiblingElement(); // get next instruction
        }
        wp_to_wp.push_back(joint_config);

        ci = ci->NextSiblingElement(); //advance from waypoint_N -> waypoint_N+1
      }
      segment_coordinates.push_back(wp_to_wp);
    }
    return segment_coordinates;
}

VectorXd calculate_joint_displacement(std::vector<std::vector<std::vector<VectorXd>>> segment_coordinates) {

  VectorXd total_displacement(6);
  total_displacement << 0, 0, 0, 0, 0, 0;

  for (size_t i = 0; i < segment_coordinates.size(); i++) {
    for (size_t j = 0; j < segment_coordinates[i].size(); j++) {
      for (size_t k = 1; k < segment_coordinates[i][j].size(); k++) {
        VectorXd displacement = segment_coordinates[i][j][k-1] - segment_coordinates[i][j][k];
        total_displacement = total_displacement + displacement.cwiseAbs();
      }
    }
  }
  std::cout << "TOTAL DISPLACEMENT" << "\n";
  std::cout << setw(20) << total_displacement << "\n";

  return total_displacement;
}

std::vector<double> createDisplacementVector(const tesseract_common::VectorIsometry3d& tool_path)
{
  std::vector<double> displacements(tool_path.size()-1);
  for(std::size_t i = 0; i < tool_path.size()-1; i++)
  {
    displacements[i] = (tool_path[i+1].translation()-tool_path[i].translation()).norm();
    std::cout << "disp_" << i << " " << displacements[i] << std::endl;
  }
}

