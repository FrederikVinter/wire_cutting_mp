#include <wc_utils.h>
#include <fstream>
#include <ros/package.h>
#include <iostream>

tesseract_common::VectorIsometry3d loadToolPosesFromPrg(const std::string& file)
{
  tesseract_common::VectorIsometry3d path;  // results
  std::ifstream indata;                     // input file

  // You could load your parts from anywhere, but we are transporting them with
  // the git repo
  std::string filename = ros::package::getPath("wire_cutting") + "/config/" + file + ".prg";

  // In a non-trivial app, you'll of course want to check that calls like 'open'
  // succeeded
  indata.open(filename);

  std::string line;
  int movel_instructions = 0;
  while (std::getline(indata, line))
  {
    std::stringstream lineStream(line);
    std::string cell;
    Eigen::Matrix<double, 7, 1> xyzWXYZ;

    bool movel_not_found = 1;
    while (std::getline(lineStream, cell, ' ') && movel_not_found == 1)
    {
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
      if(movel_instructions > 10){
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity() * Eigen::Translation3d(xyzWXYZ(0) / 1000.0, xyzWXYZ(1) / 1000.0 , xyzWXYZ(2) / 1000.0 ) *
                                      Eigen::Quaterniond(xyzWXYZ(3), xyzWXYZ(4), xyzWXYZ(5), xyzWXYZ(6));
        if(movel_instructions%10 == 0)
          path.push_back(pose);
      }
    }
    if(path.size() > 400)
      break;
  }
  indata.close();

  return path;
}