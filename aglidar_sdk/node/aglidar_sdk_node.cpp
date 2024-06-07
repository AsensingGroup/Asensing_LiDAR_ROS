#include "manager/node_manager.hpp"

#include <ag_driver/common/version.hpp>
#include <signal.h>

#ifdef ROS1_FOUND
#include <ros/ros.h>
#include <ros/package.h>
#elif ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#endif

using namespace asensing::lidar;

#ifdef ROS2_FOUND
std::mutex g_mtx;
std::condition_variable g_cv;
#endif

static void sigHandler(int sig)
{
  AG_MSG << "Asensing-LiDAR-Driver is stopping....." << AG_REND;

#ifdef ROS1_FOUND
  ros::shutdown();
#elif ROS2_FOUND
  rclcpp::shutdown();
  g_cv.notify_all();
#endif
}

int main(int argc, char** argv)
{
  signal(SIGINT, sigHandler);  ///< bind ctrl+c signal with the sigHandler function

  AG_TITLE << "********************************************************" << AG_REND;
  AG_TITLE << "**********                                    **********" << AG_REND;
  AG_TITLE << "**********    AGLidar_SDK Version: v" << AGLIDAR_VERSION_MAJOR 
    << "." << AGLIDAR_VERSION_MINOR 
    << "." << AGLIDAR_VERSION_PATCH << "     **********" << AG_REND;
  AG_TITLE << "**********                                    **********" << AG_REND;
  AG_TITLE << "********************************************************" << AG_REND;

#ifdef ROS1_FOUND
  ros::init(argc, argv, "aglidar_sdk_node", ros::init_options::NoSigintHandler);
#elif ROS2_FOUND
  rclcpp::init(argc, argv);
#endif

  std::string config_path;

#ifdef RUN_IN_ROS_WORKSPACE
   config_path = ros::package::getPath("aglidar_sdk");
#else
   config_path = (std::string)PROJECT_PATH;
#endif

   config_path += "/config/config.yaml";

#ifdef ROS1_FOUND
  ros::NodeHandle priv_hh("~");
  std::string path;
  priv_hh.param("config_path", path, std::string(""));
  if (!path.empty())
  {
    config_path = path;
  }
#endif

  YAML::Node config;
  try
  {
    config = YAML::LoadFile(config_path);
  }
  catch (...)
  {
    AG_ERROR << "The format of config file " << config_path 
      << " is wrong. Please check (e.g. indentation)." << AG_REND;
    return -1;
  }

  std::shared_ptr<NodeManager> node_ptr = std::make_shared<NodeManager>();
  node_ptr->init(config);
  node_ptr->start();

  AG_MSG << "Asensing-LiDAR-Driver is running....." << AG_REND;

#ifdef ROS1_FOUND
  ros::spin();
#elif ROS2_FOUND
  //msg_source: 2.  In order to subscribe to rosbag playback topics.(need to spin node)
  if(node_ptr->RosbagNum)
  {
    std::vector<Source::Ptr> AGSources_(node_ptr->GetSources());
    //just one source
    if(node_ptr->RosbagNum == 1)
    {
      while(rclcpp::ok())
      {
        AGSources_.back()->AGspinsome();
      }
    }
    //more than one sources
    else
    {
      while(rclcpp::ok())
      {
        for (auto& iter : AGSources_)
        {
          if (iter != nullptr)
          {
            iter->AGspinsome();
          }
        }
      }

    }

  }
  //msg_source: 1 or 3
  else
  {
    std::unique_lock<std::mutex> lck(g_mtx);
    g_cv.wait(lck);
  }
#endif

  return 0;
}
