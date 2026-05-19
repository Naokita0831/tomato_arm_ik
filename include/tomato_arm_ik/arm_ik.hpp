#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <limits>
#include <string>

constexpr double j_det_min = 0.003;

struct Angle4D
{
  float angle1{0.0f};
  float angle2{0.0f};
  float angle3{0.0f};
  float angle4{0.0f};
  float angle5{0.0f};
};

class ArmMock
{
public:
  ArmMock(float l1, float l2, float l3, float l4, float l5)
  : length1_(l1), length2_(l2), length3_(l3), length4_(l4), length5_(l5)
  {
  }

  void setAngle(float angle1, float angle2, float angle3, float angle4, float angle5)
  {
    arm1_angle_ = angle1;
    arm2_angle_ = angle2;
    arm3_angle_ = angle3;
    arm4_angle_ = angle4;
    arm5_angle_ = angle5;
  }

  void setAngle(const Angle4D & angles)
  {
    setAngle(angles.angle1, angles.angle2, angles.angle3, angles.angle4, angles.angle5);
  }

  sensor_msgs::msg::JointState getJointState() const
  {
    sensor_msgs::msg::JointState output;
    output.name = {
      arm1_joint_name_,
      arm2_joint_name_,
      arm3_joint_name_,
      arm4_joint_name_,
      arm5_joint_name_
    };
    output.position = {
      arm1_angle_,
      arm2_angle_,
      arm3_angle_,
      arm4_angle_,
      arm5_angle_
    };
    return output;
  }

  geometry_msgs::msg::Point getTargetPoint() const
  {
    geometry_msgs::msg::Point output;
    const float l = length3_ * std::sin(arm2_angle_) +
      length4_ * std::sin(arm2_angle_ + arm3_angle_) +
      length5_ * std::sin(arm2_angle_ + arm3_angle_ + arm4_angle_);
    const float z = length1_ + length2_ +
      length3_ * std::cos(arm2_angle_) +
      length4_ * std::cos(arm2_angle_ + arm3_angle_) +
      length5_ * std::cos(arm2_angle_ + arm3_angle_ + arm4_angle_);
    output.x = l * std::cos(arm1_angle_);
    output.y = l * std::sin(arm1_angle_);
    output.z = z;
    return output;
  }

private:
  float length1_{0.1f};
  float length2_{0.1f};
  float length3_{0.1f};
  float length4_{0.1f};
  float length5_{0.1f};

  float arm1_angle_{0.0f};
  float arm2_angle_{0.0f};
  float arm3_angle_{0.0f};
  float arm4_angle_{0.0f};
  float arm5_angle_{0.0f};

  std::string arm1_joint_name_{"joint1"};
  std::string arm2_joint_name_{"joint2"};
  std::string arm3_joint_name_{"joint3"};
  std::string arm4_joint_name_{"joint4"};
  std::string arm5_joint_name_{"joint5"};
};

class ArmSolver
{
public:
  ArmSolver(float l1, float l2, float l3, float l4, float l5)
  : length1_(l1), length2_(l2), length3_(l3), length4_(l4), length5_(l5)
  {
  }

  bool solve(const geometry_msgs::msg::Point & point_msg, float target_angle, Angle4D & output) const
  {
    output.angle1 = getDirection(point_msg);
    float ik_l = 0.0f;
    float ik_z = 0.0f;
    getLZ(point_msg, target_angle, ik_l, ik_z);
    output.angle2 = getAngle1(ik_l, ik_z);
    output.angle3 = getAngle2(ik_l, ik_z);
    output.angle4 = target_angle - (output.angle2 + output.angle3);
    output.angle5 = 0.0f;
    return checkAngleValid(output);
  }

  float jacobiDet(const Angle4D & angles) const
  {
    float J[4][4];
    const float c1 = std::cos(angles.angle1);
    const float c2 = std::cos(angles.angle2);
    const float c23 = std::cos(angles.angle2 + angles.angle3);
    const float c234 = std::cos(angles.angle2 + angles.angle3 + angles.angle4);
    const float s1 = std::sin(angles.angle1);
    const float s2 = std::sin(angles.angle2);
    const float s23 = std::sin(angles.angle2 + angles.angle3);
    const float s234 = std::sin(angles.angle2 + angles.angle3 + angles.angle4);

    J[0][0] = -(length3_ * s2 + length4_ * s23 + length5_ * s234) * s1;
    J[0][1] = (length3_ * c2 + length4_ * c23 + length5_ * c234) * c1;
    J[0][2] = (length4_ * c23 + length5_ * c234) * c1;
    J[0][3] = length5_ * c234 * c1;

    J[1][0] = (length3_ * s2 + length4_ * s23 + length5_ * s234) * c1;
    J[1][1] = (length3_ * c2 + length4_ * c23 + length5_ * c234) * s1;
    J[1][2] = (length4_ * c23 + length5_ * c234) * s1;
    J[1][3] = length5_ * c234 * s1;

    J[2][0] = 0.0f;
    J[2][1] = -(length3_ * s2 + length4_ * s23 + length5_ * s234);
    J[2][2] = -(length4_ * s23 + length5_ * s234);
    J[2][3] = -length5_ * s234;

    J[3][0] = 0.0f;
    J[3][1] = 1.0f;
    J[3][2] = 1.0f;
    J[3][3] = 1.0f;

    const float Jm_00 = J[1][1] * J[2][2] * J[3][3] +
      J[1][2] * J[2][3] * J[3][1] +
      J[1][3] * J[2][1] * J[3][2] -
      J[1][3] * J[2][2] * J[3][1] -
      J[1][2] * J[2][1] * J[3][3] -
      J[1][1] * J[2][3] * J[3][2];

    const float Jm_10 = J[0][1] * J[2][2] * J[3][3] +
      J[0][2] * J[2][3] * J[3][1] +
      J[0][3] * J[2][1] * J[3][2] -
      J[0][3] * J[2][2] * J[3][1] -
      J[0][2] * J[2][1] * J[3][3] -
      J[0][1] * J[2][3] * J[3][2];

    return J[0][0] * Jm_00 - J[1][0] * Jm_10;
  }

private:
  float getDirection(const geometry_msgs::msg::Point & point_msg) const
  {
    return std::atan2(point_msg.y, point_msg.x);
  }

  void getLZ(const geometry_msgs::msg::Point & point_msg, float target_angle, float & ik_l, float & ik_z) const
  {
    const float raw_l = std::sqrt(point_msg.x * point_msg.x + point_msg.y * point_msg.y);
    const float raw_z = point_msg.z;
    ik_l = raw_l - length5_ * std::sin(target_angle);
    ik_z = raw_z - length1_ - length2_ - length5_ * std::cos(target_angle);
  }

  float getAngle1(float ik_l, float ik_z) const
  {
    const float v = std::sqrt(ik_l * ik_l + ik_z * ik_z);
    return static_cast<float>(M_PI / 2.0) - getAngle(length3_, v, length4_) - std::atan2(ik_z, ik_l);
  }

  float getAngle2(float ik_l, float ik_z) const
  {
    const float v = std::sqrt(ik_l * ik_l + ik_z * ik_z);
    return static_cast<float>(M_PI) - getAngle(length3_, length4_, v);
  }

  float getAngle(float a, float b, float c) const
  {
    const float cos_n = a * a + b * b - c * c;
    const float cos_d = 2.0f * a * b;
    if (std::fabs(cos_d) < std::numeric_limits<float>::epsilon()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    const float value = std::max(-1.0f, std::min(1.0f, cos_n / cos_d));
    return std::acos(value);
  }

  bool checkAngleValid(const Angle4D & angles) const
  {
    return !std::isnan(angles.angle1) &&
      !std::isnan(angles.angle2) &&
      !std::isnan(angles.angle3) &&
      !std::isnan(angles.angle4) &&
      !std::isnan(angles.angle5);
  }

  float length1_{0.1f};
  float length2_{0.1f};
  float length3_{0.1f};
  float length4_{0.1f};
  float length5_{0.1f};
};

class ArmSmooth
{
public:
  void setTargetAngles(const Angle4D & angles)
  {
    start_angles_ = current_angles_;
    end_angles_ = angles;
  }

  void setCurrentAngles(const Angle4D & angles)
  {
    current_angles_ = angles;
  }

  Angle4D output(float value)
  {
    value = std::max(0.0f, std::min(1.0f, value));
    Angle4D output;
    output.angle1 = (1.0f - value) * start_angles_.angle1 + value * end_angles_.angle1;
    output.angle2 = (1.0f - value) * start_angles_.angle2 + value * end_angles_.angle2;
    output.angle3 = (1.0f - value) * start_angles_.angle3 + value * end_angles_.angle3;
    output.angle4 = (1.0f - value) * start_angles_.angle4 + value * end_angles_.angle4;
    output.angle5 = (1.0f - value) * start_angles_.angle5 + value * end_angles_.angle5;
    current_angles_ = output;
    return output;
  }

private:
  Angle4D start_angles_;
  Angle4D end_angles_;
  Angle4D current_angles_;
};
