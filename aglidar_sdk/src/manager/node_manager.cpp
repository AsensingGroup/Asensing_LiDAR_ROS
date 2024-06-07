#include "manager/node_manager.hpp"
#include "source/source_driver.hpp"
#include "source/source_pointcloud_ros.hpp"
#include "source/source_packet_ros.hpp"

namespace asensing
{
namespace lidar
{

void NodeManager::init(const YAML::Node& config)
{
  YAML::Node common_config = yamlSubNodeAbort(config, "common");

  int msg_source = 0;
  yamlRead<int>(common_config, "msg_source", msg_source, 0);

  bool send_packet_ros;
  yamlRead<bool>(common_config, "send_packet_ros", send_packet_ros, false);

  bool send_point_cloud_ros;
  yamlRead<bool>(common_config, "send_point_cloud_ros", send_point_cloud_ros, false);

  bool send_point_cloud_proto;
  yamlRead<bool>(common_config, "send_point_cloud_proto", send_point_cloud_proto, false);

  bool send_packet_proto;
  yamlRead<bool>(common_config, "send_packet_proto", send_packet_proto, false);

  YAML::Node lidar_config = yamlSubNodeAbort(config, "lidar");

  for (uint8_t i = 0; i < lidar_config.size(); ++i)
  {
    std::shared_ptr<Source> source;

    switch (msg_source)
    {
      case SourceType::MSG_FROM_LIDAR:  // online lidar

        AG_INFO << "------------------------------------------------------" << AG_REND;
        AG_INFO << "Receive Packets From : Online LiDAR" << AG_REND;
        AG_INFO << "Msop Port: " << lidar_config[i]["driver"]["msop_port"].as<uint16_t>() << AG_REND;
        AG_INFO << "Difop Port: " << lidar_config[i]["driver"]["difop_port"].as<uint16_t>() << AG_REND;
        AG_INFO << "------------------------------------------------------" << AG_REND;

        source = std::make_shared<SourceDriver>(SourceType::MSG_FROM_LIDAR);
        source->init(lidar_config[i]);
        break;

      case SourceType::MSG_FROM_ROS_PACKET:  // pkt from ros

        AG_INFO << "------------------------------------------------------" << AG_REND;
        AG_INFO << "Receive Packets From : ROS" << AG_REND;
        AG_INFO << "Msop Topic: " << lidar_config[i]["ros"]["ros_recv_packet_topic"].as<std::string>() << AG_REND;
        AG_INFO << "------------------------------------------------------" << AG_REND;

        source = std::make_shared<SourcePacketRos>();
        source->init(lidar_config[i]);
        RosbagNum +=1;
        break;

      case SourceType::MSG_FROM_PCAP:  // pcap

        AG_INFO << "------------------------------------------------------" << AG_REND;
        AG_INFO << "Receive Packets From : Pcap" << AG_REND;
        AG_INFO << "Msop Port: " << lidar_config[i]["driver"]["msop_port"].as<uint16_t>() << AG_REND;
        AG_INFO << "Difop Port: " << lidar_config[i]["driver"]["difop_port"].as<uint16_t>() << AG_REND;
        AG_INFO << "------------------------------------------------------" << AG_REND;

        source = std::make_shared<SourceDriver>(SourceType::MSG_FROM_PCAP);
        source->init(lidar_config[i]);
        break;

      default:
        AG_ERROR << "Unsupported LiDAR message source:" << msg_source << "." << AG_REND;
        exit(-1);
    }

    if (send_packet_ros)
    {
      AG_DEBUG << "------------------------------------------------------" << AG_REND;
      AG_DEBUG << "Send Packets To : ROS" << AG_REND;
      AG_DEBUG << "Msop Topic: " << lidar_config[i]["ros"]["ros_send_packet_topic"].as<std::string>() << AG_REND;
      AG_DEBUG << "------------------------------------------------------" << AG_REND;

      std::shared_ptr<DestinationPacket> dst = std::make_shared<DestinationPacketRos>();
      dst->init(lidar_config[i]);
      source->regPacketCallback(dst);
    }

    if (send_point_cloud_ros)
    {
      AG_DEBUG << "------------------------------------------------------" << AG_REND;
      AG_DEBUG << "Send PointCloud To : ROS" << AG_REND;
      #if defined(POINT_TYPE_XYZI)
      AG_DEBUG << "Point Type : XYZI" << AG_REND;
      #elif defined(POINT_TYPE_XYZIRT)
      AG_DEBUG << "Point Type : XYZIRT" << AG_REND;
      #else
      AG_DEBUG << "Point Type : unkonwn" << AG_REND;
      #endif
      AG_DEBUG << "PointCloud Topic: " << lidar_config[i]["ros"]["ros_send_point_cloud_topic"].as<std::string>()
               << AG_REND;
      AG_DEBUG << "------------------------------------------------------" << AG_REND;

      std::shared_ptr<DestinationPointCloud> dst = std::make_shared<DestinationPointCloudRos>();
      dst->init(lidar_config[i]);
      source->regPointCloudCallback(dst);
    }

    sources_.emplace_back(source);
  }
}

void NodeManager::start()
{
  for (auto& iter : sources_)
  {
    if (iter != nullptr)
    {
      iter->start();
    }
  }
}

void NodeManager::stop()
{
  for (auto& iter : sources_)
  {
    if (iter != nullptr)
    {
      iter->stop();
    }
  }
}

NodeManager::~NodeManager()
{
  stop();
}

}  // namespace lidar
}  // namespace asensing
