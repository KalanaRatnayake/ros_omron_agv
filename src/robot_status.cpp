#include <Aria/Aria.h>
#include <ArNetworking/ArClientRatioDrive.h>
#include <ArNetworking/ArNetworking.h>

#include <action_msgs/msg/goal_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <ros_omron_agv/msg/omron.hpp>
#include <ros_omron_agv/srv/dock_request.hpp>

#include <cmath>
#include <cstring>
#include <memory>
#include <algorithm>
#include <stdexcept>
#include <string>

class OmronStatusNode : public rclcpp::Node
{
public:
  OmronStatusNode()
      : Node("robot_status_node"),
        ratio_drive_(&client_),
        update_numbers_callback_(this, &OmronStatusNode::handleUpdateNumbers),
        update_strings_callback_(this, &OmronStatusNode::handleUpdateStrings),
        dock_info_callback_(this, &OmronStatusNode::handleDockInfo)
  {
    host_ = declare_parameter<std::string>("host", "192.168.1.1");
    port_ = declare_parameter<int>("port", 7272);
    user_ = declare_parameter<std::string>("user", "admin");
    password_ = declare_parameter<std::string>("password", "admin");
    protocol_ = declare_parameter<std::string>("protocol", "6MTX");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "current_pose");
    status_topic_ = declare_parameter<std::string>("status_topic", "robot_status");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "goal_pose");
    initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "initialpose");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    cmd_vel_timeout_sec_ = declare_parameter<double>("cmd_vel_timeout_sec", 0.2);
    max_linear_speed_mps_ = declare_parameter<double>("max_linear_speed_mps", 0.5);
    max_angular_speed_rad_s_ = declare_parameter<double>("max_angular_speed_rad_s", 1.0);
    drive_throttle_pct_ = declare_parameter<double>("drive_throttle_pct", 100.0);
    unsafe_drive_ = declare_parameter<bool>("unsafe_drive", true);

    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    status_publisher_ = create_publisher<ros_omron_agv::msg::Omron>(status_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_, 10, std::bind(&OmronStatusNode::handleGoalPose, this, std::placeholders::_1));
    initial_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initial_pose_topic_, 10, std::bind(&OmronStatusNode::handleInitialPose, this, std::placeholders::_1));
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10, std::bind(&OmronStatusNode::handleCmdVel, this, std::placeholders::_1));

    dock_service_ = create_service<ros_omron_agv::srv::DockRequest>(
        "dock", std::bind(&OmronStatusNode::handleDockRequest, this, std::placeholders::_1, std::placeholders::_2));

    watchdog_timer_ = create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&OmronStatusNode::handleCmdVelWatchdog, this));

    connectClient();
    configureHandlers();
    ratio_drive_.setThrottle(drive_throttle_pct_);
    configureDriveMode();
    client_.runAsync();

    RCLCPP_INFO(
        get_logger(), "Connected to %s:%d using protocol %s", host_.c_str(), port_,
        protocol_.empty() ? "server-advertised" : protocol_.c_str());
  }

  ~OmronStatusNode() override
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

  void configureHandlers()
  {
    if (client_.dataExists("updateNumbers"))
    {
      client_.addHandler("updateNumbers", &update_numbers_callback_);
      client_.request("updateNumbers", 50);
    }
    if (client_.dataExists("updateStrings"))
    {
      client_.addHandler("updateStrings", &update_strings_callback_);
      client_.request("updateStrings", -1);
    }
    if (client_.dataExists("dockInfoChanged"))
    {
      client_.addHandler("dockInfoChanged", &dock_info_callback_);
      client_.requestOnce("dockInfoChanged");
      client_.request("dockInfoChanged", -1);
    }
  }

  void configureDriveMode()
  {
    if (!client_.dataExists("setSafeDrive"))
    {
      RCLCPP_WARN(get_logger(), "Server does not advertise setSafeDrive; leaving drive mode unchanged");
      return;
    }

    if (unsafe_drive_)
    {
      ratio_drive_.unsafeDrive();
      RCLCPP_WARN(get_logger(), "Unsafe drive enabled for teleop");
    }
    else
    {
      ratio_drive_.safeDrive();
      RCLCPP_INFO(get_logger(), "Safe drive enabled for teleop");
    }
  }

  void handleUpdateNumbers(ArNetPacket *packet)
  {
    const double battery_voltage = static_cast<double>(packet->bufToByte2()) / 10.0;
    const double x = static_cast<double>(packet->bufToByte4()) / 1000.0;
    const double y = static_cast<double>(packet->bufToByte4()) / 1000.0;
    const double theta_deg = static_cast<double>(packet->bufToByte2());
    packet->bufToByte2();
    packet->bufToByte2();
    packet->bufToByte2();
    packet->bufToByte();

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, theta_deg * M_PI / 180.0);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = tf2::toMsg(orientation);
    tf_broadcaster_->sendTransform(transform);

    current_pose_.header = transform.header;
    current_pose_.pose.position.x = x;
    current_pose_.pose.position.y = y;
    current_pose_.pose.position.z = 0.0;
    current_pose_.pose.orientation = transform.transform.rotation;
    pose_publisher_->publish(current_pose_);

    ros_omron_agv::msg::Omron status;
    status.battery_percentage = static_cast<std::uint8_t>(std::round(battery_voltage));
    status.dock_status = static_cast<std::uint8_t>(dock_status_);
    status.robot_status = static_cast<std::uint8_t>(robot_status_);
    status_publisher_->publish(status);
  }

  void handleUpdateStrings(ArNetPacket *packet)
  {
    char status_buffer[256];
    char mode_buffer[256];
    std::memset(status_buffer, 0, sizeof(status_buffer));
    std::memset(mode_buffer, 0, sizeof(mode_buffer));
    packet->bufToStr(status_buffer, sizeof(status_buffer));
    packet->bufToStr(mode_buffer, sizeof(mode_buffer));

    const std::string status = status_buffer;
    if (status.find("Parking") != std::string::npos)
    {
      robot_status_ = action_msgs::msg::GoalStatus::STATUS_ACCEPTED;
    }
    if (status.find("Undocking") != std::string::npos || status.find("Going") != std::string::npos)
    {
      robot_status_ = action_msgs::msg::GoalStatus::STATUS_EXECUTING;
    }
    if (status.find("Arrived") != std::string::npos)
    {
      robot_status_ = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
    }
    if (status.find("Failed: Failed going to goal") != std::string::npos)
    {
      robot_status_ = action_msgs::msg::GoalStatus::STATUS_ABORTED;
    }
    if (status.find("Failed: Cannot find path") != std::string::npos)
    {
      robot_status_ = action_msgs::msg::GoalStatus::STATUS_CANCELED;
    }
  }

  void handleDockInfo(ArNetPacket *packet)
  {
    dock_status_ = packet->bufToUByte();
    packet->bufToUByte();
    packet->bufToUByte2();
  }

  void handleGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    sendPoseRequest("gotoPose", message->pose.position.x, message->pose.position.y, tf2::getYaw(message->pose.orientation));
  }

  void handleInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    sendPoseRequest(
        "localizeToPose", message->pose.pose.position.x, message->pose.pose.position.y,
        tf2::getYaw(message->pose.pose.orientation));
  }

  void sendPoseRequest(const char *request_name, double x_meters, double y_meters, double yaw_radians)
  {
    if (!client_.dataExists(request_name))
    {
      RCLCPP_WARN(get_logger(), "Server does not advertise %s", request_name);
      return;
    }

    ArNetPacket packet;
    packet.byte4ToBuf(static_cast<int>(std::lround(x_meters * 1000.0)));
    packet.byte4ToBuf(static_cast<int>(std::lround(y_meters * 1000.0)));
    packet.byte4ToBuf(static_cast<int>(std::lround(yaw_radians * 180.0 / M_PI)));
    client_.requestOnce(request_name, &packet);
  }

  void handleCmdVel(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    last_cmd_vel_time_ = now();
    stop_sent_ = false;
    cmd_vel_active_ = std::fabs(message->linear.x) > 1e-3 || std::fabs(message->angular.z) > 1e-3;
    if (!cmd_vel_active_)
    {
      ratio_drive_.stop();
      return;
    }

    if (!client_.dataExists("ratioDrive"))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Server does not advertise ratioDrive");
      return;
    }

    const double trans_ratio = toPercent(message->linear.x, max_linear_speed_mps_);
    const double rot_ratio = toPercent(message->angular.z, max_angular_speed_rad_s_);

    ratio_drive_.setThrottle(drive_throttle_pct_);
    ratio_drive_.setLatVelRatio(0.0);
    ratio_drive_.setTransVelRatio(trans_ratio);
    ratio_drive_.setRotVelRatio(rot_ratio);
  }

  void handleCmdVelWatchdog()
  {
    if (!cmd_vel_active_)
    {
      return;
    }

    if ((now() - last_cmd_vel_time_).seconds() > cmd_vel_timeout_sec_)
    {
      cmd_vel_active_ = false;
      if (!stop_sent_)
      {
        ratio_drive_.stop();
        stop_sent_ = true;
        RCLCPP_WARN(get_logger(), "cmd_vel timeout; stop requested");
      }
    }
  }

  double toPercent(double value, double max_abs_value) const
  {
    if (max_abs_value <= 0.0)
    {
      return 0.0;
    }

    const double percent = (value / max_abs_value) * 100.0;
    return std::clamp(percent, -100.0, 100.0);
  }

  void handleDockRequest(
      const std::shared_ptr<ros_omron_agv::srv::DockRequest::Request>,
      std::shared_ptr<ros_omron_agv::srv::DockRequest::Response> response)
  {
    response->result = false;
    if (!client_.dataExists("dock"))
    {
      RCLCPP_WARN(get_logger(), "Server does not advertise dock");
      return;
    }
    client_.requestOnce("dock");
    response->result = true;
  }

  ArClientBase client_;
  std::string host_;
  int port_ = 7272;
  std::string user_;
  std::string password_;
  std::string protocol_;
  std::string map_frame_;
  std::string base_frame_;
  std::string pose_topic_;
  std::string status_topic_;
  std::string goal_topic_;
  std::string initial_pose_topic_;
  std::string cmd_vel_topic_;
  double cmd_vel_timeout_sec_ = 0.2;
  double max_linear_speed_mps_ = 0.5;
  double max_angular_speed_rad_s_ = 1.0;
  double drive_throttle_pct_ = 100.0;
  bool unsafe_drive_ = true;
  int robot_status_ = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
  int dock_status_ = ros_omron_agv::msg::Omron::UNDOCKED;
  bool cmd_vel_active_ = false;
  bool stop_sent_ = false;
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  ArClientRatioDrive ratio_drive_;
  geometry_msgs::msg::PoseStamped current_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<ros_omron_agv::msg::Omron>::SharedPtr status_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Service<ros_omron_agv::srv::DockRequest>::SharedPtr dock_service_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  ArFunctor1C<OmronStatusNode, ArNetPacket *> update_numbers_callback_;
  ArFunctor1C<OmronStatusNode, ArNetPacket *> update_strings_callback_;
  ArFunctor1C<OmronStatusNode, ArNetPacket *> dock_info_callback_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  try
  {
    auto node = std::make_shared<OmronStatusNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("robot_status_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}