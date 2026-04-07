#include <Aria/Aria.h>
#include <ArNetworking/ArNetworking.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

class OmronLaserNode : public rclcpp::Node
{
public:
  OmronLaserNode()
  : Node("omron_laser_node"),
    primary_callback_(this, &OmronLaserNode::handlePrimaryLaser),
    secondary_callback_(this, &OmronLaserNode::handleSecondaryLaser)
  {
    host_ = declare_parameter<std::string>("host", "172.19.21.203");
    port_ = declare_parameter<int>("port", 7272);
    user_ = declare_parameter<std::string>("user", "steve");
    password_ = declare_parameter<std::string>("password", "");
    protocol_ = declare_parameter<std::string>("protocol", "6MTX");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    primary_request_ = declare_parameter<std::string>("primary_request", "Laser_1Current");
    secondary_request_ = declare_parameter<std::string>("secondary_request", "Laser_2Current");
    primary_topic_ = declare_parameter<std::string>("primary_topic", "/laser");
    secondary_topic_ = declare_parameter<std::string>("secondary_topic", "/laser_low");
    request_period_ms_ = declare_parameter<int>("request_period_ms", 200);

    primary_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(primary_topic_, 10);
    secondary_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(secondary_topic_, 10);

    connectClient();
    configureLaserRequest(primary_request_, primary_callback_);
    configureLaserRequest(secondary_request_, secondary_callback_);
    client_.runAsync();

    RCLCPP_INFO(
      get_logger(), "Connected to %s:%d using protocol %s", host_.c_str(), port_,
      protocol_.empty() ? "server-advertised" : protocol_.c_str());
  }

  ~OmronLaserNode() override
  {
    client_.disconnect();
    Aria::shutdown();
  }

private:
  void connectClient()
  {
    Aria::init();
    ArLog::init(ArLog::StdOut, ArLog::Normal);

    if (!protocol_.empty())
    {
      client_.enforceProtocolVersion(protocol_.c_str());
    }

    const char *password = password_.empty() ? NULL : password_.c_str();
    if (!client_.blockingConnect(host_.c_str(), port_, true, user_.c_str(), password))
    {
      if (client_.wasRejected())
      {
        throw std::runtime_error("Robot rejected the connection credentials or protocol.");
      }
      throw std::runtime_error("Could not connect to the Omron robot server.");
    }

    client_.setRobotName(host_.c_str());
  }

  void configureLaserRequest(const std::string &request_name, ArFunctor1<ArNetPacket *> &callback)
  {
    if (!client_.dataExists(request_name.c_str()))
    {
      RCLCPP_WARN(get_logger(), "Server does not advertise %s", request_name.c_str());
      return;
    }

    client_.addHandler(request_name.c_str(), &callback);
    client_.request(request_name.c_str(), request_period_ms_);
    RCLCPP_INFO(get_logger(), "Subscribed to %s every %d ms", request_name.c_str(), request_period_ms_);
  }

  void handlePrimaryLaser(ArNetPacket *packet)
  {
    publishPointCloud(packet, primary_publisher_);
  }

  void handleSecondaryLaser(ArNetPacket *packet)
  {
    publishPointCloud(packet, secondary_publisher_);
  }

  void publishPointCloud(
    ArNetPacket *packet,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher)
  {
    sensor_msgs::msg::PointCloud2 scan;
    scan.header.stamp = now();
    scan.header.frame_id = frame_id_;

    const int num_readings = packet->bufToByte4();
    if (num_readings <= 0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Received empty laser packet");
      return;
    }

    scan.height = 1;
    scan.width = static_cast<std::uint32_t>(num_readings);
    scan.is_bigendian = false;
    scan.is_dense = false;
    scan.point_step = 12;
    scan.row_step = scan.point_step * scan.width;
    scan.fields.resize(3);
    scan.fields[0].name = "x";
    scan.fields[0].offset = 0;
    scan.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    scan.fields[0].count = 1;
    scan.fields[1].name = "y";
    scan.fields[1].offset = 4;
    scan.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    scan.fields[1].count = 1;
    scan.fields[2].name = "z";
    scan.fields[2].offset = 8;
    scan.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    scan.fields[2].count = 1;
    scan.data.resize(scan.row_step * scan.height);

    for (int i = 0; i < num_readings; ++i)
    {
      const float x = static_cast<float>(packet->bufToByte4()) / 1000.0f;
      const float y = static_cast<float>(packet->bufToByte4()) / 1000.0f;
      const float z = 0.0f;
      std::uint8_t *point = &scan.data[static_cast<size_t>(i) * scan.point_step];
      std::memcpy(point + 0, &x, sizeof(float));
      std::memcpy(point + 4, &y, sizeof(float));
      std::memcpy(point + 8, &z, sizeof(float));
    }

    publisher->publish(scan);
  }

  ArClientBase client_;
  std::string host_;
  int port_ = 7272;
  std::string user_;
  std::string password_;
  std::string protocol_;
  std::string frame_id_;
  std::string primary_request_;
  std::string secondary_request_;
  std::string primary_topic_;
  std::string secondary_topic_;
  int request_period_ms_ = 200;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr primary_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr secondary_publisher_;
  ArFunctor1C<OmronLaserNode, ArNetPacket *> primary_callback_;
  ArFunctor1C<OmronLaserNode, ArNetPacket *> secondary_callback_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  try
  {
    auto node = std::make_shared<OmronLaserNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("omron_laser_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}