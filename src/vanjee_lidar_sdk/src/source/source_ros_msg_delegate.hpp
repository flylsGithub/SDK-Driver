#pragma once

#ifdef ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "vanjee_lidar_sdk/msg/device_ctrl.hpp"
#include "vanjee_lidar_sdk/msg/lidar_parameter_interface.hpp"
#include "vanjee_lidar_sdk/msg/vanjee_lidar_multi_packet.hpp"
#include "vanjee_lidar_sdk/msg/vanjeelidar_packet.hpp"

namespace vanjee {
namespace lidar {
template <typename T_RosMsg>
class VanjeeLidarSdkPublishRosMsg {
 public:
  using NodePtr = std::shared_ptr<rclcpp::Node>;
  explicit VanjeeLidarSdkPublishRosMsg(NodePtr node_ptr, std::string topic_name, const rclcpp::QoS& qos) {
    pub_ = node_ptr->create_publisher<T_RosMsg>(topic_name, qos);
  }
  void PublishMsg(T_RosMsg& msg) {
    pub_->publish(msg);
  }

 private:
  typename rclcpp::Publisher<T_RosMsg>::SharedPtr pub_;
};

template <typename T_RosMsg, typename T_SdkMsg>
class VanjeeLidarSdkSubscribeRosMsg {
 public:
  using NodePtr = std::shared_ptr<rclcpp::Node>;
  explicit VanjeeLidarSdkSubscribeRosMsg(const NodePtr node_ptr, const std::string& topic_name, const rclcpp::QoS& qos)
      : node_ptr_(node_ptr), topic_name_(topic_name), qos_(qos) {
  }

  void SubscribeTopic(std::function<void(const std::shared_ptr<T_RosMsg>)> subscribe_topic_callback) {
    sub_ = node_ptr_->create_subscription<T_RosMsg>(topic_name_, qos_, subscribe_topic_callback);
  }

 private:
  typename rclcpp::Subscription<T_RosMsg>::SharedPtr sub_;
  NodePtr node_ptr_;
  std::string topic_name_;
  rclcpp::QoS qos_;
};

class VanjeeLidarSdkNode : public rclcpp::Node {
 public:
  static std::shared_ptr<VanjeeLidarSdkNode> CreateInstance() {
    static std::shared_ptr<VanjeeLidarSdkNode> vanjee_lidar_sdk_node_ptr(new VanjeeLidarSdkNode());
    return vanjee_lidar_sdk_node_ptr;
  }

  VanjeeLidarSdkNode(const VanjeeLidarSdkNode&) = delete;
  VanjeeLidarSdkNode& operator=(const VanjeeLidarSdkNode&) = delete;

  std::shared_ptr<VanjeeLidarSdkPublishRosMsg<sensor_msgs::msg::PointCloud2>> GetPointCloud2MsgPublisher(std::string topic_name,
                                                                                                         const rclcpp::QoS& qos) {
    return std::make_shared<VanjeeLidarSdkPublishRosMsg<sensor_msgs::msg::PointCloud2>>(CreateInstance(), topic_name, qos);
  }

  std::shared_ptr<VanjeeLidarSdkPublishRosMsg<sensor_msgs::msg::LaserScan>> GetLaserScanMsgPublisher(std::string topic_name, const rclcpp::QoS& qos) {
    return std::make_shared<VanjeeLidarSdkPublishRosMsg<sensor_msgs::msg::LaserScan>>(CreateInstance(), topic_name, qos);
  }

  std::shared_ptr<VanjeeLidarSdkPublishRosMsg<sensor_msgs::msg::Imu>> GetImuMsgPublisher(std::string topic_name, const rclcpp::QoS& qos) {
    return std::make_shared<VanjeeLidarSdkPublishRosMsg<sensor_msgs::msg::Imu>>(CreateInstance(), topic_name, qos);
  }

  std::shared_ptr<VanjeeLidarSdkPublishRosMsg<vanjee_lidar_sdk::msg::DeviceCtrl>> GetDeviceCtrlMsgPublisher(std::string topic_name,
                                                                                                            const rclcpp::QoS& qos) {
    return std::make_shared<VanjeeLidarSdkPublishRosMsg<vanjee_lidar_sdk::msg::DeviceCtrl>>(CreateInstance(), topic_name, qos);
  }

  std::shared_ptr<VanjeeLidarSdkSubscribeRosMsg<vanjee_lidar_sdk::msg::DeviceCtrl, vanjee::lidar::DeviceCtrl>> GetDeviceCtrlMsgSubscriber(
      std::string topic_name, const rclcpp::QoS& qos) {
    return std::make_shared<VanjeeLidarSdkSubscribeRosMsg<vanjee_lidar_sdk::msg::DeviceCtrl, vanjee::lidar::DeviceCtrl>>(CreateInstance(), topic_name,
                                                                                                                         qos);
  }

  std::shared_ptr<VanjeeLidarSdkPublishRosMsg<vanjee_lidar_sdk::msg::VanjeelidarPacket>> GetPacketMsgPublisher(std::string topic_name,
                                                                                                               const rclcpp::QoS& qos) {
    return std::make_shared<VanjeeLidarSdkPublishRosMsg<vanjee_lidar_sdk::msg::VanjeelidarPacket>>(CreateInstance(), topic_name, qos);
  }

  std::shared_ptr<VanjeeLidarSdkPublishRosMsg<vanjee_lidar_sdk::msg::VanjeeLidarMultiPacket>> GetMultiPacketMsgPublisher(std::string topic_name,
                                                                                                                         const rclcpp::QoS& qos) {
    return std::make_shared<VanjeeLidarSdkPublishRosMsg<vanjee_lidar_sdk::msg::VanjeeLidarMultiPacket>>(CreateInstance(), topic_name, qos);
  }

  std::shared_ptr<VanjeeLidarSdkSubscribeRosMsg<vanjee_lidar_sdk::msg::VanjeelidarPacket, vanjee::lidar::Packet>> GetPacketMsgSubscriber(
      std::string topic_name, const rclcpp::QoS& qos) {
    return std::make_shared<VanjeeLidarSdkSubscribeRosMsg<vanjee_lidar_sdk::msg::VanjeelidarPacket, vanjee::lidar::Packet>>(CreateInstance(),
                                                                                                                            topic_name, qos);
  }

  std::shared_ptr<VanjeeLidarSdkSubscribeRosMsg<vanjee_lidar_sdk::msg::VanjeeLidarMultiPacket, vanjee::lidar::MultiPacket>>
  GetMultiPacketMsgSubscriber(std::string topic_name, const rclcpp::QoS& qos) {
    return std::make_shared<VanjeeLidarSdkSubscribeRosMsg<vanjee_lidar_sdk::msg::VanjeeLidarMultiPacket, vanjee::lidar::MultiPacket>>(
        CreateInstance(), topic_name, qos);
  }

  std::shared_ptr<VanjeeLidarSdkPublishRosMsg<vanjee_lidar_sdk::msg::LidarParameterInterface>> GetLidarParameterInterfaceMsgPublisher(
      std::string topic_name, const rclcpp::QoS& qos) {
    return std::make_shared<VanjeeLidarSdkPublishRosMsg<vanjee_lidar_sdk::msg::LidarParameterInterface>>(CreateInstance(), topic_name, qos);
  }

  std::shared_ptr<VanjeeLidarSdkSubscribeRosMsg<vanjee_lidar_sdk::msg::LidarParameterInterface, vanjee::lidar::LidarParameterInterface>>
  GetLidarParameterInterfaceMsgSubscriber(std::string topic_name, const rclcpp::QoS& qos) {
    return std::make_shared<VanjeeLidarSdkSubscribeRosMsg<vanjee_lidar_sdk::msg::LidarParameterInterface, vanjee::lidar::LidarParameterInterface>>(
        CreateInstance(), topic_name, qos);
  }

 private:
  VanjeeLidarSdkNode() : Node("vanjee_lidar_sdk_node") {
  }
};

inline rclcpp::QoS makeQosFromYaml(const YAML::Node& node, int default_depth = 10) {
  int depth = default_depth;
  std::string history = "keep_last";
  std::string reliability = "reliable";
  std::string durability = "volatile";
  if (node && node.IsMap()) {
    yamlRead<int>(node, "depth", depth, default_depth);
    yamlRead<std::string>(node, "history", history, "keep_last");
    yamlRead<std::string>(node, "reliability", reliability, "reliable");
    yamlRead<std::string>(node, "durability", durability, "volatile");
  }
  RCLCPP_INFO(rclcpp::get_logger("vanjee_lidar_sdk"), "makeQosFromYaml input: depth=%d, history=%s, reliability=%s, durability=%s", depth,
              history.c_str(), reliability.c_str(), durability.c_str());
  rclcpp::QoS qos(depth);
  if (history == "keep_all") {
    qos.keep_all();
  } else {
    qos.keep_last(depth);
  }
  if (reliability == "best_effort") {
    qos.best_effort();
  } else {
    qos.reliable();
  }
  if (durability == "transient_local") {
    qos.transient_local();
  } else {
    qos.durability_volatile();
  }
  return qos;
}

}  // namespace lidar
}  // namespace vanjee
#endif
