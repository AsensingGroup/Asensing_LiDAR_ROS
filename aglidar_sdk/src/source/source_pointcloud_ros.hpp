#pragma once

#include "source/source.hpp"

#ifdef ROS1_FOUND
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace asensing
{
namespace lidar
{

inline sensor_msgs::PointCloud2 toRosMsg(const LidarPointCloudMsg& ag_msg, const std::string& frame_id, bool send_by_rows)
{
  sensor_msgs::PointCloud2 ros_msg;

  int fields = 4;
#ifdef POINT_TYPE_XYZIRT
  fields = 7;
#endif
  ros_msg.fields.clear();
  ros_msg.fields.reserve(fields);

  if (send_by_rows)
  {
    ros_msg.width = ag_msg.width; 
    ros_msg.height = ag_msg.height; 
  }
  else
  {
    ros_msg.width = ag_msg.height; // exchange width and height to be compatible with pcl::PointCloud<>
    ros_msg.height = ag_msg.width; 
  }

  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::PointField::UINT8, offset);
#ifdef POINT_TYPE_XYZIRT
  offset = addPointField(ros_msg, "ring", 1, sensor_msgs::PointField::UINT8, offset);
  offset = addPointField(ros_msg, "range", 1, sensor_msgs::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::PointField::FLOAT64, offset);
  
#endif

#if 0
  std::cout << "off:" << offset << std::endl;
#endif

  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.is_dense = ag_msg.is_dense;

  size_t points_size_ = std::max((size_t)ag_msg.points.size(), (size_t)(ros_msg.width * ros_msg.height));
  ros_msg.data.resize(ros_msg.point_step * points_size_);

  sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity_(ros_msg, "intensity");
#ifdef POINT_TYPE_XYZIRT
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_ring_(ros_msg, "ring");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_range_(ros_msg, "range");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");
#endif

  if (send_by_rows)
  {
    for (size_t i = 0; i < ag_msg.height; i++)
    {
      for (size_t j = 0; j < ag_msg.width; j++)
      {
        const LidarPointCloudMsg::PointT& point = ag_msg.points[i + j * ag_msg.height];

        *iter_x_ = point.x;
        *iter_y_ = point.y;
        *iter_z_ = point.z;
        *iter_intensity_ = point.intensity;

        ++iter_x_;
        ++iter_y_;
        ++iter_z_;
        ++iter_intensity_;

#ifdef POINT_TYPE_XYZIRT
        *iter_ring_ = point.ring;
        *iter_range_ = point.range;
        *iter_timestamp_ = point.timestamp;
        
        ++iter_ring_;
        ++iter_range_;
        ++iter_timestamp_;
        
#endif
      }
    }
  }
  else
  {
    for (size_t i = 0; i < ag_msg.points.size(); i++)
    {
      const LidarPointCloudMsg::PointT& point = ag_msg.points[i];

      *iter_x_ = point.x;
      *iter_y_ = point.y;
      *iter_z_ = point.z;
      *iter_intensity_ = point.intensity;

      ++iter_x_;
      ++iter_y_;;
      ++iter_z_;
      ++iter_intensity_;

#ifdef POINT_TYPE_XYZIRT
      *iter_ring_ = point.ring;
      *iter_range_ = point.range;
      *iter_timestamp_ = point.timestamp;
      
      ++iter_ring_;
      ++iter_range_;
      ++iter_timestamp_;
#endif
    }
  }

  ros_msg.header.seq = ag_msg.seq;
  ros_msg.header.stamp = ros_msg.header.stamp.fromSec(ag_msg.timestamp);
  ros_msg.header.frame_id = frame_id;

  return ros_msg;
}

class DestinationPointCloudRos : public DestinationPointCloud
{
public:

  virtual void init(const YAML::Node& config);
  virtual void sendPointCloud(const LidarPointCloudMsg& msg);
  virtual ~DestinationPointCloudRos() = default;

private:
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher pub_;
  std::string frame_id_;
  bool send_by_rows_;
};

inline void DestinationPointCloudRos::init(const YAML::Node& config)
{
  yamlRead<bool>(config["ros"], 
      "ros_send_by_rows", send_by_rows_, false);

  bool dense_points;
  yamlRead<bool>(config["driver"], "dense_points", dense_points, false);
  if (dense_points)
    send_by_rows_ = false;

  yamlRead<std::string>(config["ros"], 
      "ros_frame_id", frame_id_, "aglidar");

  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], 
      "ros_send_point_cloud_topic", ros_send_topic, "aglidar_points");

  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  pub_ = nh_->advertise<sensor_msgs::PointCloud2>(ros_send_topic, 10);
}

inline void DestinationPointCloudRos::sendPointCloud(const LidarPointCloudMsg& msg)
{
  pub_.publish(toRosMsg(msg, frame_id_, send_by_rows_));
}

}  // namespace lidar
}  // namespace asensing

#endif  // ROS1_FOUND

#ifdef ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>

namespace asensing
{
namespace lidar
{

inline void toRosMsg(const LidarPointCloudMsg& ag_msg, const std::string& frame_id, bool send_by_rows, sensor_msgs::msg::PointCloud2& ros_msg)
{
  int fields = 4;
#ifdef POINT_TYPE_XYZIRT
  fields = 7;
#endif
  ros_msg.fields.clear();
  ros_msg.fields.reserve(fields);

  if (send_by_rows)
  {
    ros_msg.width = ag_msg.width;
    ros_msg.height = ag_msg.height;
  }
  else
  {
    ros_msg.width = ag_msg.height; // exchange width and height to be compatible with pcl::PointCloud<>
    ros_msg.height = ag_msg.width;
  }

  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::msg::PointField::UINT8, offset);
#ifdef POINT_TYPE_XYZIRT
  offset = addPointField(ros_msg, "ring", 1, sensor_msgs::msg::PointField::UINT8, offset);
  offset = addPointField(ros_msg, "range", 1, sensor_msgs::msg::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64, offset);
#endif

#if 0
  std::cout << "off:" << offset << std::endl;
#endif

  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.is_dense = ag_msg.is_dense;

  size_t points_size_ = std::max((size_t)ag_msg.points.size(), (size_t)(ros_msg.width * ros_msg.height));
  ros_msg.data.resize(ros_msg.point_step * points_size_);

  sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity_(ros_msg, "intensity");
#ifdef POINT_TYPE_XYZIRT
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_ring_(ros_msg, "ring");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_range_(ros_msg, "range");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");
#endif

  if (send_by_rows)
  {
    for (size_t i = 0; i < ag_msg.height; i++)
    {
      for (size_t j = 0; j < ag_msg.width; j++)
      {
        const LidarPointCloudMsg::PointT& point = ag_msg.points[i + j * ag_msg.height];

        *iter_x_ = point.x;
        *iter_y_ = point.y;
        *iter_z_ = point.z;
        *iter_intensity_ = point.intensity;

        ++iter_x_;
        ++iter_y_;
        ++iter_z_;
        ++iter_intensity_;

#ifdef POINT_TYPE_XYZIRT
        *iter_ring_ = point.ring;
        *iter_range_ = point.range;
        *iter_timestamp_ = point.timestamp;

        ++iter_ring_;
        ++iter_range_;
        ++iter_timestamp_;
#endif
      }
    }
  }
  else
  {
    for (size_t i = 0; i < ag_msg.points.size(); i++)
    {
      const LidarPointCloudMsg::PointT& point = ag_msg.points[i];

      *iter_x_ = point.x;
      *iter_y_ = point.y;
      *iter_z_ = point.z;
      *iter_intensity_ = point.intensity;

      ++iter_x_;
      ++iter_y_;
      ++iter_z_;
      ++iter_intensity_;

#ifdef POINT_TYPE_XYZIRT
      *iter_ring_ = point.ring;
      *iter_range_ = point.range;
      *iter_timestamp_ = point.timestamp;
      
      ++iter_ring_;
      ++iter_range_;
      ++iter_timestamp_;
#endif
    }
  }

  ros_msg.header.stamp.sec = (uint32_t)floor(ag_msg.timestamp);
  ros_msg.header.stamp.nanosec = (uint32_t)round((ag_msg.timestamp - ros_msg.header.stamp.sec) * 1e9);
  ros_msg.header.frame_id = frame_id;
}

class DestinationPointCloudRos : virtual public DestinationPointCloud
{
public:

  virtual void init(const YAML::Node& config);
  virtual void sendPointCloud(const LidarPointCloudMsg& msg);
  virtual ~DestinationPointCloudRos() = default;

private:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::string frame_id_;
  bool send_by_rows_;
  // uint32_t valid_width_;
  sensor_msgs::msg::PointCloud2 ros_msg_;
};

inline void DestinationPointCloudRos::init(const YAML::Node& config)
{
  yamlRead<bool>(config["ros"], 
      "ros_send_by_rows", send_by_rows_, false);

  bool dense_points;
  yamlRead<bool>(config["driver"], "dense_points", dense_points, false);
  if (dense_points)
    send_by_rows_ = false;

  yamlRead<std::string>(config["ros"], 
      "ros_frame_id", frame_id_, "aglidar");

  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], 
      "ros_send_point_cloud_topic", ros_send_topic, "aglidar_points");

  // yamlRead<uint32_t>(config["driver"], "valid_width", valid_width_, 1260);

  static int node_index = 0;
  std::stringstream node_name;
  node_name << "aglidar_points_destination_" << node_index++;

  node_ptr_.reset(new rclcpp::Node(node_name.str()));
  //pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(ros_send_topic, 100);
  pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(ros_send_topic, rclcpp::SensorDataQoS());
}

inline void DestinationPointCloudRos::sendPointCloud(const LidarPointCloudMsg& msg)
{
  // if (msg.width < 1260) {
  //   AG_DEBUG << "******" << AG_REND;
  // }
  // AG_DEBUG << "[" << msg.frame_id << "] Width = " << msg.width << ", Height = " << msg.height << AG_REND;
  
  // pub_->publish(toRosMsg(msg, frame_id_, send_by_rows_));

  toRosMsg(msg, frame_id_, send_by_rows_, ros_msg_);
  pub_->publish(ros_msg_);
}

}  // namespace lidar
}  // namespace asensing

#endif // ROS2_FOUND
