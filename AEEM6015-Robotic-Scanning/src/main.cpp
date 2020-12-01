#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <boost/filesystem.hpp>

#include "scanner/rs_scanner.h"
#include "viewer/pcl_viewer.h"
#include "util/geometry.h"
#include "util/file_io.h"
#include "util/config.h"

#include "robot/state/cartesian_pose_state.h"
#include "robot/state/joint_position_state.h"
#include "robot/command/joint_position_command.h"
#include "robot/iiwa_util.h"

using namespace s3d;
using namespace std;

const string rootPath = "/home/yufeng/ros_ws/src/robotic_scan/";

bool bStopIIWA = false;
void IIWARobotCallBack() { bStopIIWA = true;}
void MoveIIWAtoPosition(JointPositionCmd* jpCmd, const iiwa_msgs::JointPosition& pos, double tSleep)
{
  //std::cout << "send command" << std::endl;
  jpCmd->SetPosition(pos, &IIWARobotCallBack);
  int count = 0;
  while (bStopIIWA == false)
  {
    count++;
    ros::Duration(tSleep).sleep();
    if (count > 5)
    {
      // resend the command after a while if no message recieved
      //std::cout << "send command" << std::endl;
      jpCmd->SetPosition(pos, &IIWARobotCallBack);
      count = 0;
    }
  }
  bStopIIWA = false;
}

void PrintHelp();
bool UserInput();
int ParseArguments(int argc, char** argv, std::string& file, bool& viewMesh, int& imageCount, double& timeDuration);
void ViewPointCloud(const std::string& strFile, double timeDuration);
// void Calibration(const std::string& file, int count, const Eigen::Affine3f& toolTrans, const ViewFrame& vFrame);

// main
int main(int argc, char** argv) try
{
  std::string strFile("");
  bool viewMesh = false;
  int imageCount = 3;
  double timeDuration = 1.0; // s
  int task = ParseArguments(argc, argv, strFile, viewMesh, imageCount, timeDuration);

  if (task == 1) // generate trajectory
  {
    if (strFile.empty())
    {
      std::cout << "!error! == invalid file for saving trajectory" << std::endl;
      PrintHelp();
      return 0;
    }

    Trajectory t;
    JointPositionState jpState;
    ros::AsyncSpinner spinner(1); // use 4 threads
    spinner.start();
    bool bGenerateTrajecotry = true;
    while (bGenerateTrajecotry)
    {
      bGenerateTrajecotry = UserInput();
      if (!bGenerateTrajecotry) { break; }
      iiwa_msgs::JointPosition pos = jpState.Pose();
      t.AddPosition(pos);
    }
    spinner.stop();
    std::cout << "create trajectory and save to " << strFile << std::endl;
    t.Save(strFile);
    return 0;
  }
  else if (task == 2) // scanning
  {
    // check devices
    RSScanner rscam;
    if(!rscam.Initial())
    {
      std::cout << "!error! == unable to initialize camera" << std::endl;
      return -1;
    }

    // configuration
    std::cout << "setup == " << std::to_string(imageCount) << " images will be taken at each stop" << std::endl;
    std::string config = rootPath+"config/view_frame.txt";
    ViewFrame vFrame = LoadConfiguration(config);
    std::cout << "setup == load view frame (w/h/d): ["
      << vFrame.width[0] << ", " << vFrame.width[1] << ", "
      << vFrame.height[0] << ", " << vFrame.height[1] << ", "
      << vFrame.depth[0] << ", " << vFrame.depth[1] << "]\n";

    // trajectory
    Trajectory t;
    if (!strFile.empty())
    {
      t.Load(strFile);
      std::cout << "setup == load trajectory from " << strFile << std::endl;
    }
    if (t.PositionCount() == 0)
    {
      std::cout << "setup == generate default circle trajectory" << std::endl;
      t = GenerateCirclePath(90, 15);
    }

    WSPointCloudPtr finalCloud(new WSPointCloud());
    // tool transform
    Eigen::Affine3f toolTransform = ToolTransform();
    // trajecotry for capturing images
    JointPositionState jpState;
    CartesianPoseState cpState; // CartesianPoseState reader
    JointPositionCmd jpCmd; // JointPositionCommand

    ros::AsyncSpinner spinner(1); // use 1 threads
    spinner.start();
    ros::Duration(timeDuration).sleep();

    // initialize
    iiwa_msgs::JointPosition initPos = jpState.Pose();
    if (!AtHome(initPos)) // move to home
    {
      std::cout << "kuka == move to home position." << std::endl;
      iiwa_msgs::JointPosition home = JointPose(.0,.0,.0,.0,.0,.0,.0);
      MoveIIWAtoPosition(&jpCmd, home, timeDuration);
    }

    // create image along the trajectory
    for (int i = 0; i < t.PositionCount(); ++i)
    {
      std::cout << "kuka == move to " << i << " position"<< std::endl;
      iiwa_msgs::JointPosition p = t.PositionAt(i);
      MoveIIWAtoPosition(&jpCmd, p, timeDuration);

      // get camera position
      iiwa_msgs::CartesianPose cp = cpState.Pose();
      Eigen::Affine3f trans = CartesianPoseToTransform(cp);
      trans = trans*toolTransform;

      // take image
      for (int j = 0; j < imageCount; ++j)
      {
        rs2::points points;
        rs2::frame color;
        if (rscam.CapturePoints(points, color))
        {
          WSPointCloudPtr cloud = ConvertRSPointToPCLPoint(points, color);
          cloud = CropPCLPoint(cloud, vFrame);
          cloud = TransformPCLPoint(cloud, trans);
          *finalCloud += *cloud;
        }
      }

      // std::string file = "/home/yufeng/scan3d/data/capture_" + std::to_string(i);
      // SaveTransform(file+".txt", trans);
      // SaveCaptureData(file+".pcd", cloud);
    }

    // end
    iiwa_msgs::JointPosition endPos = jpState.Pose();
    if (!AtHome(endPos)) // move to home
    {
      std::cout << "kuka == move to home position" << std::endl;
      iiwa_msgs::JointPosition home = JointPose(.0,.0,.0,.0,.0,.0,.0);
      MoveIIWAtoPosition(&jpCmd, home, timeDuration);
    }

    spinner.stop();
    rscam.Stop();

    // process final cloud
    std::cout << "pcl == remove noise points" << std::endl;
    /*
    The number of neighbors to analyze for each point is set to 50,
    and the standard deviation multiplier to 1. What this means is
    that all points who have a distance larger than 1 standard
    deviation of the mean distance to the query point will be marked
    as outliers and removed.
    */
    finalCloud = FilterPCLPointSOR(finalCloud, 50, 1);
    std::cout << "pcl == down sampling" << std::endl;
    // leaf size = 0.01
    finalCloud = FilterPCLPoint(finalCloud, 0.01);
    std::cout << "pcl == smooth point cloud" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr smoothCloud = SmoothPCLPoint(finalCloud, 0.01);

    // visualization
    if (viewMesh)
    {
      pcl::PolygonMesh mesh;
      std::cout << "pcl == convert point cloud to mesh" << std::endl;
      if(ConvertPointCloudToPolygonMesh(smoothCloud, mesh))
      {
        std::cout << "pcl == visualize mesh model" << std::endl;
        PCLViewer viewer("mesh");
        viewer.UpdateMesh(mesh);
        viewer.Spin();
      }
    }
    else
    {
      std::cout << "pcl == visualize point cloud" << std::endl;
      PCLViewer viewer("point_cloud");
      viewer.UpdateS(smoothCloud);
      viewer.Spin();
    }

    // save model
    char charSave;
    std::cout << "do you want to save the model? [y/n]";
    std::cin >> charSave;
    std::cout << std::endl;
    if (charSave == 'y' || charSave == 'Y')
    {
      std::string file = rootPath+"data/capture.ply";
      if(SaveCaptureDataPLY(file, finalCloud))
        std::cout << "model is saved to " << file << std::endl;
    }

    return 0;
  }
  else if (task == 3) // view mesh file
  {
    ViewPointCloud(strFile,timeDuration);
    return 0;
  }
  else
  {
    PrintHelp();
    return 0;
  }
}
catch (const std::exception& e)
{
  std::cout << e.what() << std::endl;
  return -1;
}


// process user input
bool UserInput()
{
  bool setLoopFlag;
  bool inputCheck = false;
  char takeFrame;
  do
  {
    std::cout << std::endl;
    std::cout << "Record Trajecotry Position ? [y/n]";
    std::cin >> takeFrame;
    std::cout << std::endl;

    if (takeFrame == 'y' || takeFrame == 'Y')
    {
      setLoopFlag = true;
      inputCheck = true;
      takeFrame = 0;
    }
    else if (takeFrame == 'n' || takeFrame == 'N')
    {
      setLoopFlag = false;
      inputCheck = true;
      takeFrame = 0;
    }
    else
    {
      inputCheck = false;
      std::cout << "Invalid input." << std::endl;
      takeFrame = 0;
    }
  } while (inputCheck == false);

  return setLoopFlag;
}

int ParseArguments(int argc, char** argv, std::string& file, bool& viewMesh, int& imageCount, double& timeDuration)
{
  // 1 for creating trajectory
  // 2 for scanning
  // other will print help.
  if (argc == 1)
      return 0;

  if (argc > 1)
  {
    if (argc > 2)
      file = std::string(argv[2]);
    if (argc > 3)
      viewMesh = (std::string(argv[3]).compare("1") == 0);
    if (argc > 4)
      imageCount = std::stoi(argv[4]);
    if (argc > 5)
      timeDuration = std::stod(argv[5]);

    std::string strArg(argv[1]);
    // generate trajectory
    if (strArg.compare("-gt") == 0)
    {
      return 1;
    }
    // scan
    if (strArg.compare("-scan") == 0)
    {
      return 2;
    }
    // visualization
    if (strArg.compare("-view") == 0)
    {
      return 3;
    }
    // // calibration
    // if (strArg.compare("-cali") == 0)
    // {
    //   return 44;
    // }
  }

  return 0;
}

void PrintHelp()
{
  std::cout << "Use this app with arguments to perform robotic scanning: \n";
  std::cout << "   Create Trajectory: cmd [-gt] [trajectory_file_path(for saving)] \n";
  std::cout << "   Scan with Trajectory: cmd [-scan] [trajectory_file_path(for loading)] [mesh/pointcloud|1/0] [image_count] [time_sleep] \n";
  std::cout << "   View Point Cloud: cmd [-view] [point_cloud_file]\n";
  std::cout << "   Print Help: cmd [-h|-help] \n";
  std::cout << "Note: \n if no trajectory supplied for scan, it will use a default path to demo the robotic scanning.\n";
}

// view point cloud in specific folder
void ViewPointCloud(const std::string& strFile, double timeDuration)
{
  if (strFile.empty())
  {
    std::cout << "!error! == invalid file for visualization" << std::endl;
    PrintHelp();
  }

  int no_new_file_count = 0;
  WSPointCloudPtr cloud(new WSPointCloud());
  std::vector<std::string> files;
  PCLViewer viewer("point_cloud");
  bool bDone = false;
  while(!viewer.IsStop())
  {
    if (bDone)
    {
      viewer.Spin(5);
      continue;
    }

    std::vector<std::string> allfiles;
    boost::filesystem::directory_iterator itr(strFile);
    for (; itr != boost::filesystem::directory_iterator(); ++itr)
    {
      if (boost::filesystem::is_regular_file(itr->status()));
        allfiles.push_back(itr->path().string());
    }

    bool bComplete = (files.size() == allfiles.size()) && !allfiles.empty();
    if (!bComplete)
      no_new_file_count=0;
    bool bNewCloud = false;
    for (const auto& file : allfiles)
    {
        std::vector<std::string>::iterator end = files.end();
        if (std::find(files.begin(),files.end(),file) == files.end())
        {
            std::cout << "pcl == load point cloud from " << file << std::endl;
            WSPointCloudPtr temp(new WSPointCloud());
            if(LoadCaptureDataPLY(file, temp))
            {
              temp = FilterPCLPointSOR(temp, 50, 1);
              temp = FilterPCLPoint(temp, 0.001);
              *cloud += *temp;
              bNewCloud = true;
              files.push_back(file);
            }
        }
     }

     if (bNewCloud)
     {
       viewer.Update(cloud);
     }
     else
     {
       if(bComplete)
       {
         if (no_new_file_count < 60)
         {
           std::this_thread::sleep_for(std::chrono::milliseconds(1000));
           no_new_file_count++;
           continue;
         }

         char toMesh;
         std::cout << "Converted to Mesh ? [y/n]";
         std::cin >> toMesh;
         if (toMesh == 'y' || toMesh == 'Y')
         {
           std::cout << "pcl == smoothing point cloud" << std::endl;
           pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr smoothCloud = SmoothPCLPoint(cloud, 0.01);
           std::cout << "pcl == converting point cloud to mesh" << std::endl;
           pcl::PolygonMesh mesh;
           if(ConvertPointCloudToPolygonMesh(smoothCloud, mesh))
           {
             std::cout << "pcl == viewing mesh model" << std::endl;
             PCLViewer viewer("mesh");
             viewer.UpdateMesh(mesh);
             viewer.Spin(100);
           }
         }
         else
         {
           bDone = true;
         }
       }
     }

     viewer.Spin(5);
  }
}

// void Calibration(const std::string& file, int count, const Eigen::Affine3f& toolTrans, const ViewFrame& vFrame)
// {
//   WSPointCloudPtr finalCloud(new WSPointCloud());
//
//   for (int i = 0; i < count; ++i)
//   {
//       std::string fileName = file + "_" + std::to_string(i);
//
//       Eigen::Affine3f trans;
//       LoadTransform(fileName + ".txt", trans);
//       trans = trans * toolTrans;
//
//       WSPointCloudPtr cloud(new WSPointCloud());
//       LoadCaptureData(fileName + ".pcd", cloud);
//       cloud = CropPCLPoint(cloud, vFrame);
//       cloud = TransformPCLPoint(cloud, trans);
//       *finalCloud += *cloud;
//   }
//
//   PCLViewer viewer("test");
//   viewer.Update(finalCloud);
//   viewer.Spin();
// }
