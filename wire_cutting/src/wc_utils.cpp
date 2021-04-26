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

        //std::cout << "JOINT POSE " << j << "\n" << opt_joint_results[i][j] << "\n";

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

trajopt::TermInfo::Ptr createVelocityTermInfo(double max_displacement,
                                                int start_index,
                                                int end_index,
                                                const std::string& link,
                                                trajopt::TermType type)
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

  return term;
}

trajopt::TermInfo::Ptr createRotationalVelocityTermInfo(double max_displacement,
                                                //Eigen::Vector3d rot_coeffs,
                                                int start_index,
                                                int end_index,
                                                const std::string& link,
                                                trajopt::TermType type)
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

  return term;
}

trajopt::TermInfo::Ptr createCartesianWaypointTermInfoWC(const Eigen::Isometry3d& c_wp,
                                                       int index,
                                                       std::string working_frame,
                                                       Eigen::Isometry3d tcp,
                                                       const Eigen::VectorXd& coeffs,
                                                       std::string link,
                                                       trajopt::TermType type)
{
  auto pose_info = std::make_shared<CartPoseTermInfoWC>();
  pose_info->term_type = type;
  pose_info->name = "cartesian_waypoint_" + std::to_string(index);

  pose_info->link = link;
  pose_info->tcp = tcp;

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
