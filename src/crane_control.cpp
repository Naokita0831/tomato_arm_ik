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
#define ADDR_TORQUE_ENABLE_P1    24
#define ADDR_GOAL_POSITION_P1    30
#define ADDR_MOVING_SPEED_P1     32

#define LEN_GOAL_POSITION_P1     2

#define PROTOCOL_VERSION1        1.0
#define BAUDRATE                 1000000

#define AX_POS_UPPER_LIMIT       1023
#define AX_POS_LOWER_LIMIT       0

class CraneControlNode : public rclcpp::Node
{
public:
  CraneControlNode()
  : Node("crane_control")
  {
    dev_name_ = this->declare_parameter<std::string>("dev", "/dev/ttyUSB0");
    moving_speed_ = this->declare_parameter<int>("moving_speed", 180);
    max_delta_rad_ = this->declare_parameter<double>("max_delta_rad", 0.015);
    deadband_rad_ = this->declare_parameter<double>("deadband_rad", 0.02);

    port_handler_ = PortHandler::getPortHandler(dev_name_.c_str());
    packet_handler_ = PacketHandler::getPacketHandler(PROTOCOL_VERSION1);

    if (!port_handler_->openPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", dev_name_.c_str());
      throw std::runtime_error("Failed to open port");
    }

    if (!port_handler_->setBaudRate(BAUDRATE)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate");
      throw std::runtime_error("Failed to set baudrate");
    }

    group_sync_write_ = std::make_unique<GroupSyncWrite>(
      port_handler_,
      packet_handler_,
      ADDR_GOAL_POSITION_P1,
      LEN_GOAL_POSITION_P1
    );

    for (auto id : dxl_ids_) {
      writeSpeed(id, static_cast<uint16_t>(moving_speed_));
      torque(id, true);
    }

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
    for (auto id : dxl_ids_) {
      torque(id, false);
    }

    if (port_handler_) {
      port_handler_->closePort();
    }
  }

private:
  void torque(uint8_t dxl_id, bool on)
  {
    uint8_t dxl_error = 0;

    int result = packet_handler_->write1ByteTxRx(
      port_handler_,
      dxl_id,
      ADDR_TORQUE_ENABLE_P1,
      on ? 1 : 0,
      &dxl_error
    );

    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set torque. ID: %d", dxl_id);
    }
  }

  void writeSpeed(uint8_t dxl_id, uint16_t speed)
  {
    uint8_t dxl_error = 0;

    int result = packet_handler_->write2ByteTxRx(
      port_handler_,
      dxl_id,
      ADDR_MOVING_SPEED_P1,
      speed,
      &dxl_error
    );

    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set moving speed. ID: %d", dxl_id);
    }
  }

  double applyDeadband(double rad, size_t index)
  {
    // joint1だけ0rad付近の微小振動を消す
    if (index == 0 && std::fabs(rad) < deadband_rad_) {
      return 0.0;
    }

    return rad;
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
    if (msg->position.size() < dxl_ids_.size()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "joint_states position size is too small"
      );
      return;
    }

    if (!initialized_) {
      for (size_t i = 0; i < dxl_ids_.size(); ++i) {
        prev_rad_[i] = msg->position[i];
      }
      initialized_ = true;
    }

    group_sync_write_->clearParam();

    for (size_t i = 0; i < dxl_ids_.size(); ++i) {
      double target_rad = msg->position[i];

      target_rad = applyDeadband(target_rad, i);

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

      bool addparam_result = group_sync_write_->addParam(
        dxl_ids_[i],
        param_goal_position
      );

      if (!addparam_result) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to add sync write param. ID: %d",
          dxl_ids_[i]
        );
      }

      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "joint%zu rad: %.3f -> dxl: %d",
        i + 1,
        limited_rad,
        target_position
      );
    }

    int result = group_sync_write_->txPacket();

    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "GroupSyncWrite failed. result: %d",
        result
      );
    }
  }

  std::string dev_name_;
  int moving_speed_;
  double max_delta_rad_;
  double deadband_rad_;

  PortHandler * port_handler_{nullptr};
  PacketHandler * packet_handler_{nullptr};
  std::unique_ptr<GroupSyncWrite> group_sync_write_;

  std::array<uint8_t, 5> dxl_ids_ = {1, 2, 3, 4, 5};

  // joint1だけ逆方向にしている．違えば 1.0 に戻す
  std::array<double, 5> joint_sign_ = {
    1.0,
    1.0,
    1.0,
    1.0,
    1.0
  };

  // joint1の0rad位置がずれている場合は512.0を微調整する
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