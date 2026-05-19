#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

using namespace dynamixel;

// Protocol 2.0 MX
#define ADDR_OPERATING_MODE_P2    11
#define ADDR_TORQUE_ENABLE_P2     64
#define ADDR_GOAL_VELOCITY_P2     104
#define ADDR_PRESENT_VELOCITY_P2  128

// Protocol 1.0 AX
#define ADDR_TORQUE_ENABLE_P1     24
#define ADDR_GOAL_POSITION_P1     30
#define ADDR_PRESENT_POSITION_P1  36

#define PROTOCOL_VERSION1         1.0
#define PROTOCOL_VERSION2         2.0

#define BAUDRATE                  1000000

#define VELOCITY_MODE             1

class AxMxJoyNode : public rclcpp::Node
{
public:
  AxMxJoyNode()
  : Node("ax_mx_joy")
  {
    dev_name_ = this->declare_parameter<std::string>("dev", "/dev/ttyUSB0");

    ax_id_ = this->declare_parameter<int>("ax_id", 1);
    mx_id_ = this->declare_parameter<int>("mx_id", 10);

    ax_axis_ = this->declare_parameter<int>("ax_axis", 0);
    mx_axis_ = this->declare_parameter<int>("mx_axis", 1);

    scale_ax_ = this->declare_parameter<double>("scale_ax", 4.0);
    scale_mx_ = this->declare_parameter<double>("scale_mx", 300.0);

    ax_min_ = this->declare_parameter<int>("ax_min", 300);
    ax_max_ = this->declare_parameter<int>("ax_max", 700);
    ax_initial_ = this->declare_parameter<int>("ax_initial", 512);

    mx_vel_limit_ = this->declare_parameter<int>("mx_vel_limit", 250);
    loop_hz_ = this->declare_parameter<double>("loop_hz", 200.0);

    position_ax_write_ = ax_initial_;

    port_handler_ = PortHandler::getPortHandler(dev_name_.c_str());
    packet_handler1_ = PacketHandler::getPacketHandler(PROTOCOL_VERSION1);
    packet_handler2_ = PacketHandler::getPacketHandler(PROTOCOL_VERSION2);

    if (!port_handler_->openPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", dev_name_.c_str());
      throw std::runtime_error("Failed to open port");
    }

    if (!port_handler_->setBaudRate(BAUDRATE)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate");
      throw std::runtime_error("Failed to set baudrate");
    }

    torqueAx(true);

    torqueMx(false);
    setMxOperatingMode(VELOCITY_MODE);
    torqueMx(true);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy",
      10,
      std::bind(&AxMxJoyNode::joyCallback, this, std::placeholders::_1)
    );

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / loop_hz_),
      std::bind(&AxMxJoyNode::loop, this)
    );

    RCLCPP_INFO(this->get_logger(), "ax_mx_joy started");
  }

  ~AxMxJoyNode()
  {
    writeMxVelocity(0);
    torqueAx(false);
    torqueMx(false);

    if (port_handler_) {
      port_handler_->closePort();
    }
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (ax_axis_ >= 0 && static_cast<size_t>(ax_axis_) < msg->axes.size()) {
      vel_ax_ = msg->axes[ax_axis_] * scale_ax_;
    }

    if (mx_axis_ >= 0 && static_cast<size_t>(mx_axis_) < msg->axes.size()) {
      double v = msg->axes[mx_axis_] * scale_mx_;
      v = std::clamp(v, -static_cast<double>(mx_vel_limit_), static_cast<double>(mx_vel_limit_));
      vel_mx_write_ = static_cast<int32_t>(v);
    }
  }

  void torqueAx(bool on)
  {
    uint8_t dxl_error = 0;

    int result = packet_handler1_->write1ByteTxRx(
      port_handler_,
      ax_id_,
      ADDR_TORQUE_ENABLE_P1,
      on ? 1 : 0,
      &dxl_error
    );

    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set AX torque. ID: %d result: %d", ax_id_, result);
    }
  }

  void torqueMx(bool on)
  {
    uint8_t dxl_error = 0;

    int result = packet_handler2_->write1ByteTxRx(
      port_handler_,
      mx_id_,
      ADDR_TORQUE_ENABLE_P2,
      on ? 1 : 0,
      &dxl_error
    );

    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set MX torque. ID: %d result: %d", mx_id_, result);
    }
  }

  void setMxOperatingMode(uint8_t mode)
  {
    uint8_t dxl_error = 0;

    int result = packet_handler2_->write1ByteTxRx(
      port_handler_,
      mx_id_,
      ADDR_OPERATING_MODE_P2,
      mode,
      &dxl_error
    );

    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set MX operating mode. ID: %d result: %d", mx_id_, result);
    }
  }

  void writeAxPosition(uint16_t position)
  {
    uint8_t dxl_error = 0;

    int result = packet_handler1_->write2ByteTxRx(
      port_handler_,
      ax_id_,
      ADDR_GOAL_POSITION_P1,
      position,
      &dxl_error
    );

    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write AX position. result: %d", result);
    }
  }

  void writeMxVelocity(int32_t velocity)
  {
    uint8_t dxl_error = 0;

    int result = packet_handler2_->write4ByteTxRx(
      port_handler_,
      mx_id_,
      ADDR_GOAL_VELOCITY_P2,
      static_cast<uint32_t>(velocity),
      &dxl_error
    );

    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write MX velocity. result: %d", result);
    }
  }

  void readDebug()
  {
    uint8_t dxl_error = 0;

    uint16_t ax_pos = 0;
    int result_ax = packet_handler1_->read2ByteTxRx(
      port_handler_,
      ax_id_,
      ADDR_PRESENT_POSITION_P1,
      &ax_pos,
      &dxl_error
    );

    uint32_t mx_vel_raw = 0;
    int result_mx = packet_handler2_->read4ByteTxRx(
      port_handler_,
      mx_id_,
      ADDR_PRESENT_VELOCITY_P2,
      &mx_vel_raw,
      &dxl_error
    );

    if (result_ax == COMM_SUCCESS && result_mx == COMM_SUCCESS) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "AX pos: %u, MX goal_vel: %d, MX present_vel_raw: %u",
        ax_pos,
        vel_mx_write_,
        mx_vel_raw
      );
    }
  }

  void loop()
  {
    writeAxPosition(static_cast<uint16_t>(position_ax_write_));
    writeMxVelocity(vel_mx_write_);

    position_ax_write_ += static_cast<int>(vel_ax_);
    position_ax_write_ = std::clamp(position_ax_write_, ax_min_, ax_max_);

    readDebug();
  }

  std::string dev_name_;

  int ax_id_;
  int mx_id_;
  int ax_axis_;
  int mx_axis_;

  double scale_ax_;
  double scale_mx_;
  int ax_min_;
  int ax_max_;
  int ax_initial_;
  int mx_vel_limit_;
  double loop_hz_;

  double vel_ax_{0.0};
  int32_t vel_mx_write_{0};
  int position_ax_write_{512};

  PortHandler * port_handler_{nullptr};
  PacketHandler * packet_handler1_{nullptr};
  PacketHandler * packet_handler2_{nullptr};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<AxMxJoyNode>());
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}