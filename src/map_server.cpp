#include <Aria/Aria.h>
#include <ArNetworking/ArNetworking.h>

#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <cstring>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void configureAriaDirectory()
{
  if (std::getenv("ARIA") != nullptr)
  {
    return;
  }

#ifdef ROS_OMRON_ARIA_DIR
  ::setenv("ARIA", ROS_OMRON_ARIA_DIR, 0);
#endif
}
}

class OmronMapNode : public rclcpp::Node
{
public:
  OmronMapNode()
  : Node("map_node"),
    map_name_callback_(this, &OmronMapNode::handleGetMapName),
    map_callback_(this, &OmronMapNode::handleGetMap)
  {
    host_ = declare_parameter<std::string>("host", "192.168.1.1");
    port_ = declare_parameter<int>("port", 7272);
    user_ = declare_parameter<std::string>("user", "admin");
    password_ = declare_parameter<std::string>("password", "admin");
    protocol_ = declare_parameter<std::string>("protocol", "6MTX");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", latched_qos);
    metadata_publisher_ = create_publisher<nav_msgs::msg::MapMetaData>("map_metadata", latched_qos);
    get_map_service_ = create_service<nav_msgs::srv::GetMap>(
      "get_map", std::bind(&OmronMapNode::handleGetMapService, this, std::placeholders::_1, std::placeholders::_2));

    connectClient();
    if (client_.dataExists("getMap"))
    {
      client_.addHandler("getMap", &map_callback_);
      map_start_time_.setToNow();
      client_.requestOnce("getMap");
    }
    if (client_.dataExists("getMapName"))
    {
      client_.addHandler("getMapName", &map_name_callback_);
      client_.requestOnce("getMapName");
    }
    client_.runAsync();

    RCLCPP_INFO(
      get_logger(), "Connected to %s:%d using protocol %s", host_.c_str(), port_,
      protocol_.empty() ? "server-advertised" : protocol_.c_str());
  }

  ~OmronMapNode() override
  {
    client_.disconnect();
    Aria::shutdown();
  }

private:
  void connectClient()
  {
    configureAriaDirectory();
    Aria::init();
    ArLog::init(ArLog::StdOut, ArLog::Normal);

    if (!protocol_.empty())
    {
      client_.enforceProtocolVersion(protocol_.c_str());
    }

    const char *password = password_.empty() ? nullptr : password_.c_str();
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

  void handleGetMapName(ArNetPacket *packet)
  {
    char buffer[512];
    packet->bufToStr(buffer, sizeof(buffer));
    RCLCPP_INFO(get_logger(), "MapFile: %s", buffer);
  }

  void handleGetMap(ArNetPacket *packet)
  {
    char buffer[65536];

    if (packet->getDataReadLength() == packet->getDataLength())
    {
      return;
    }

    packet->bufToStr(buffer, sizeof(buffer));
    if (buffer[0] == '\0')
    {
      finalizeMap();
      return;
    }

    char *header_location = std::strstr(buffer, "2D-Map-Ex4");
    if (header_location != nullptr)
    {
      buffer[6] = '\0';
    }

    ar_map_.parseLine(buffer);
  }

  void finalizeMap()
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    ar_map_.parsingComplete();

    const int resolution_mm = ar_map_.getResolution();
    if (resolution_mm <= 0)
    {
      RCLCPP_WARN(get_logger(), "Received an invalid map resolution");
      return;
    }

    const ArPose min_pose = ar_map_.getMinPose();
    const ArPose max_pose = ar_map_.getMaxPose();
    const int grid_x = static_cast<int>((max_pose.getX() - min_pose.getX()) / resolution_mm);
    const int grid_y = static_cast<int>((max_pose.getY() - min_pose.getY()) / resolution_mm);
    if (grid_x <= 0 || grid_y <= 0)
    {
      RCLCPP_WARN(get_logger(), "Received an empty map");
      return;
    }

    map_message_.header.frame_id = frame_id_;
    map_message_.header.stamp = now();
    map_message_.info.map_load_time = now();
    map_message_.info.resolution = static_cast<float>(resolution_mm) / 1000.0f;
    map_message_.info.width = static_cast<std::uint32_t>(grid_x);
    map_message_.info.height = static_cast<std::uint32_t>(grid_y);
    map_message_.info.origin.position.x = min_pose.getX() / 1000.0;
    map_message_.info.origin.position.y = min_pose.getY() / 1000.0;
    map_message_.info.origin.position.z = 0.0;
    map_message_.data.assign(static_cast<size_t>(grid_x * grid_y), 0);

    const std::vector<ArPose> *point_list = ar_map_.getPoints();
    for (size_t index = 0; index < point_list->size(); ++index)
    {
      const int coord = grid_x * static_cast<int>((point_list->at(index).getY() - min_pose.getY()) / resolution_mm) +
        static_cast<int>((point_list->at(index).getX() - min_pose.getX()) / resolution_mm);
      if (coord >= 0 && coord < static_cast<int>(map_message_.data.size()))
      {
        map_message_.data[static_cast<size_t>(coord)] = 100;
      }
    }

    map_ready_ = true;
    metadata_publisher_->publish(map_message_.info);
    map_publisher_->publish(map_message_);
    RCLCPP_INFO(
      get_logger(), "Read a %u x %u map @ %.3f m/cell in %.2f s", map_message_.info.width,
      map_message_.info.height, map_message_.info.resolution, map_start_time_.mSecSince() / 1000.0);
  }

  void handleGetMapService(
    const std::shared_ptr<nav_msgs::srv::GetMap::Request>,
    std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    response->map = map_message_;
  }

  ArMap ar_map_;
  ArClientBase client_;
  ArTime map_start_time_;
  std::string host_;
  int port_ = 7272;
  std::string user_;
  std::string password_;
  std::string protocol_;
  std::string frame_id_;
  bool map_ready_ = false;
  std::mutex map_mutex_;
  nav_msgs::msg::OccupancyGrid map_message_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr metadata_publisher_;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr get_map_service_;
  ArFunctor1C<OmronMapNode, ArNetPacket *> map_name_callback_;
  ArFunctor1C<OmronMapNode, ArNetPacket *> map_callback_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  try
  {
    auto node = std::make_shared<OmronMapNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("map_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
