#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <algorithm>
#include <iostream>

using namespace dynamixel;

// AX-12A Protocol 1.0
#define AX_ID_1                  1
#define AX_ID_2                  2
#define AX_ID_3                  3
#define AX_ID_4                  4
#define AX_ID_5                  5

#define ADDR_TORQUE_ENABLE_P1    24
#define ADDR_GOAL_POSITION_P1    30
#define ADDR_MOVING_SPEED_P1     32

#define LEN_GOAL_POSITION_P1     2

#define PROTOCOL_VERSION1        1.0
#define BAUDRATE                 1000000

#define AX_POS_UPPER_LIMIT       1023
#define AX_POS_LOWER_LIMIT       0

// MX Protocol 2.0
#define MX_ID                    10

#define PROTOCOL_VERSION2        2.0

#define ADDR_OPERATING_MODE_P2   11
#define ADDR_TORQUE_ENABLE_P2    64
#define ADDR_GOAL_VELOCITY_P2    104
#define ADDR_PRESENT_VELOCITY_P2 128

#define LEN_GOAL_VELOCITY_P2     4

#define MX_VELOCITY_MODE         1

class CraneControlNode : public rclcpp::Node
{
public:
  CraneControlNode()
  : Node("crane_control_mx")
  {
    dev_name_ = this->declare_parameter<std::string>("dev", "/dev/ttyUSB0");

    // AXサーボの動作速度
    moving_speed_ = this->declare_parameter<int>("moving_speed", 180);

    // 1周期の最大角度変化量
    max_delta_rad_ = this->declare_parameter<double>("max_delta_rad", 0.015);

    // MX速度指令の最大値
    mx_max_velocity_ = this->declare_parameter<int32_t>("mx_max_velocity", 200);

    port_handler_ = PortHandler::getPortHandler(dev_name_.c_str());

    packet_handler_p1_ = PacketHandler::getPacketHandler(PROTOCOL_VERSION1);
    packet_handler_p2_ = PacketHandler::getPacketHandler(PROTOCOL_VERSION2);

    if (!port_handler_->openPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", dev_name_.c_str());
      throw std::runtime_error("Failed to open port");
    }

    if (!port_handler_->setBaudRate(BAUDRATE)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate");
      throw std::runtime_error("Failed to set baudrate");
    }

    group_sync_write_ax_ = std::make_unique<GroupSyncWrite>(
      port_handler_,
      packet_handler_p1_,
      ADDR_GOAL_POSITION_P1,
      LEN_GOAL_POSITION_P1
    );

    group_sync_write_mx_ = std::make_unique<GroupSyncWrite>(
      port_handler_,
      packet_handler_p2_,
      ADDR_GOAL_VELOCITY_P2,
      LEN_GOAL_VELOCITY_P2
    );

    for (auto id : ax_ids_) {
      writeAxSpeed(id, static_cast<uint16_t>(moving_speed_));
      torqueAx(id, true);
    }

    setupMxVelocityMode(MX_ID);
    torqueMx(MX_ID, true);

    prev_rad_.fill(0.0);
    initialized_ = false;

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",
      10,
      std::bind(&CraneControlNode::jointStateCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "crane_control started");
  }

  ~CraneControlNode()
  {
    sendMxVelocity(MX_ID, 0);

    for (auto id : ax_ids_) {
      torqueAx(id, false);
    }

    torqueMx(MX_ID, false);

    if (port_handler_) {
      port_handler_->closePort();
    }
  }

private:
  void torqueAx(uint8_t dxl_id, bool on)
  {
    uint8_t dxl_error = 0;

    int result = packet_handler_p1_->write1ByteTxRx(
      port_handler_,
      dxl_id,
      ADDR_TORQUE_ENABLE_P1,
      on ? 1 : 0,
      &dxl_error
    );

    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set AX torque. ID: %d", dxl_id);
    }
  }

  void writeAxSpeed(uint8_t dxl_id, uint16_t speed)
  {
    uint8_t dxl_error = 0;

    int result = packet_handler_p1_->write2ByteTxRx(
      port_handler_,
      dxl_id,
      ADDR_MOVING_SPEED_P1,
      speed,
      &dxl_error
    );

    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set AX moving speed. ID: %d", dxl_id);
    }
  }

  void torqueMx(uint8_t dxl_id, bool on)
  {
    uint8_t dxl_error = 0;

    int result = packet_handler_p2_->write1ByteTxRx(
      port_handler_,
      dxl_id,
      ADDR_TORQUE_ENABLE_P2,
      on ? 1 : 0,
      &dxl_error
    );

    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set MX torque. ID: %d", dxl_id);
    }
  }

  void setupMxVelocityMode(uint8_t dxl_id)
  {
    uint8_t dxl_error = 0;

    torqueMx(dxl_id, false);

    int result = packet_handler_p2_->write1ByteTxRx(
      port_handler_,
      dxl_id,
      ADDR_OPERATING_MODE_P2,
      MX_VELOCITY_MODE,
      &dxl_error
    );

    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set MX velocity mode. ID: %d", dxl_id);
    }
  }

  void sendMxVelocity(uint8_t dxl_id, int32_t velocity)
  {
    velocity = std::clamp(
      velocity,
      -mx_max_velocity_,
      mx_max_velocity_
    );

    group_sync_write_mx_->clearParam();

    uint8_t param_goal_velocity[4];
    param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(velocity));
    param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(velocity));
    param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(velocity));
    param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(velocity));

    bool addparam_result = group_sync_write_mx_->addParam(
      dxl_id,
      param_goal_velocity
    );

    if (!addparam_result) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to add MX sync write param. ID: %d",
        dxl_id
      );
      return;
    }

    int result = group_sync_write_mx_->txPacket();

    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "MX GroupSyncWrite failed. result: %d",
        result
      );
    }
  }

  uint16_t radToAxPosition(double rad, size_t index)
  {
    double signed_rad = joint_sign_[index] * rad;

    double pos = signed_rad / (2.0 * M_PI) * 1024.0 + joint_center_[index];

    int pos_int = static_cast<int>(std::round(pos));
    pos_int = std::clamp(pos_int, AX_POS_LOWER_LIMIT, AX_POS_UPPER_LIMIT);

    return static_cast<uint16_t>(pos_int);
  }

  double limitDelta(double target, double current, double max_delta)
  {
    double diff = target - current;

    if (diff > max_delta) {
      diff = max_delta;
    } else if (diff < -max_delta) {
      diff = -max_delta;
    }

    return current + diff;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() < ax_ids_.size()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "joint_states position size is too small"
      );
      return;
    }

    if (!initialized_) {
      for (size_t i = 0; i < ax_ids_.size(); ++i) {
        prev_rad_[i] = msg->position[i];
      }
      initialized_ = true;
    }

    group_sync_write_ax_->clearParam();

    for (size_t i = 0; i < ax_ids_.size(); ++i) {
      double target_rad = msg->position[i];

      double limited_rad = limitDelta(
        target_rad,
        prev_rad_[i],
        max_delta_rad_
      );

      prev_rad_[i] = limited_rad;

      uint16_t target_position = radToAxPosition(limited_rad, i);

      uint8_t param_goal_position[2];
      param_goal_position[0] = DXL_LOBYTE(target_position);
      param_goal_position[1] = DXL_HIBYTE(target_position);

      bool addparam_result = group_sync_write_ax_->addParam(
        ax_ids_[i],
        param_goal_position
      );

      if (!addparam_result) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to add AX sync write param. ID: %d",
          ax_ids_[i]
        );
      }

      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "AX joint%zu rad: %.3f -> dxl: %d",
        i + 1,
        limited_rad,
        target_position
      );
    }

    int ax_result = group_sync_write_ax_->txPacket();

    if (ax_result != COMM_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "AX GroupSyncWrite failed. result: %d",
        ax_result
      );
    }

    int32_t mx_velocity = 0;

    if (!msg->velocity.empty()) {
      mx_velocity = static_cast<int32_t>(std::round(msg->velocity[0]));
    }

    sendMxVelocity(MX_ID, mx_velocity);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "MX velocity command: %d",
      mx_velocity
    );
  }

  std::string dev_name_;
  int moving_speed_;
  double max_delta_rad_;
  int32_t mx_max_velocity_;

  PortHandler * port_handler_{nullptr};
  PacketHandler * packet_handler_p1_{nullptr};
  PacketHandler * packet_handler_p2_{nullptr};

  std::unique_ptr<GroupSyncWrite> group_sync_write_ax_;
  std::unique_ptr<GroupSyncWrite> group_sync_write_mx_;

  std::array<uint8_t, 5> ax_ids_ = {AX_ID_1, AX_ID_2, AX_ID_3, AX_ID_4, AX_ID_5};

  std::array<double, 5> joint_sign_ = {
    1.0,
    1.0,
    1.0,
    1.0,
    1.0
  };

  std::array<double, 5> joint_center_ = {
    512.0,
    512.0,
    512.0,
    512.0,
    512.0
  };

  std::array<double, 5> prev_rad_;
  bool initialized_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<CraneControlNode>());
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}