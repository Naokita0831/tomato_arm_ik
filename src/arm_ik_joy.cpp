#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <cmath>
#include <memory>
#include <mutex>

#include "tomato_arm_ik/arm_ik.hpp"

class ArmIkJoyNode : public rclcpp::Node
{
public:
  ArmIkJoyNode()
  : Node("arm_ik_joy"),
    arm_mock_(0.0454f, 0.026f, 0.083f, 0.0935f, 0.0473f),
    arm_solver_(0.0454f, 0.026f, 0.083f, 0.0935f, 0.0473f)//linkの長さをここに入力
  {
    publish_joint_states_ = this->declare_parameter<bool>("publish_joint_states", true);
    trajectory_topic_ = this->declare_parameter<std::string>("trajectory_topic", "/crane_plus_arm_controller/joint_trajectory");
    timer_period_ms_ = this->declare_parameter<int>("timer_period_ms", 50);
    trajectory_time_sec_ = this->declare_parameter<double>("trajectory_time_sec", 0.05);

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("target_point", 1);
    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(trajectory_topic_, 1);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&ArmIkJoyNode::joyCallback, this, std::placeholders::_1));

    target_point_.header.frame_id = "base_link";

    //初期姿勢
    current_angles_.angle1 = 0.0f;
    current_angles_.angle2 = 0.6f;
    current_angles_.angle3 = -0.3f;
    current_angles_.angle4 = -0.6f;
    current_angles_.angle5 = 0.0f;
    
    //初期姿勢をもとに手先位置を算出
    arm_mock_.setAngle(current_angles_);
    target_point_.point = arm_mock_.getTargetPoint();
    target_angle_ = current_angles_.angle2 + current_angles_.angle3 + current_angles_.angle4;


    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms_), std::bind(&ArmIkJoyNode::loop, this));

    RCLCPP_INFO(this->get_logger(), "arm_ik_joy started. publish to: %s", trajectory_topic_.c_str());
  }

private:
  //joyより値を受け取って手先目標位置更新料を作成
  void joyCallback(const sensor_msgs::msg::Joy & msg)
  {
    received_joy_ = true;

    const float gain_x = 0.001f;
    const float gain_y = 0.001f;
    const float gain_z = 0.001f;
    const float gain_rot = 0.01f;

    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (msg.axes.size() >= 5) {
      cmd_x_ = msg.axes[1] * gain_x;
      cmd_y_ = msg.axes[0] * gain_y;
      cmd_z_ = msg.axes[3] * gain_z;
      cmd_rot_ = msg.axes[4] * gain_rot;
    }
  }

  void loop()
  {
    if (!received_joy_) {
      publishCurrentState();
      return;
    }

    float cmd_x = 0.0f;
    float cmd_y = 0.0f;
    float cmd_z = 0.0f;
    float cmd_rot = 0.0f;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      cmd_x = cmd_x_;
      cmd_y = cmd_y_;
      cmd_z = cmd_z_;
      cmd_rot = cmd_rot_;
    }

    //目標手先位置姿勢を作成
    target_point_.header.stamp = this->now();
    target_point_.point.x += cmd_x;
    target_point_.point.y += cmd_y;
    target_point_.point.z += cmd_z;
    target_angle_ += cmd_rot;

    Angle4D angles;
    if (!arm_solver_.solve(target_point_.point, target_angle_, angles)) {
      rollbackTarget(cmd_x, cmd_y, cmd_z, cmd_rot);
      publishCurrentState();
      return;
    }

    //角度上限化どうか判断
    if (std::fabs(angles.angle1) > limit_upper_ || std::fabs(angles.angle1) < limit_lower_ ||
        std::fabs(angles.angle2) > limit_upper_ || std::fabs(angles.angle2) < limit_lower_ ||
        std::fabs(angles.angle3) > limit_upper_ || std::fabs(angles.angle3) < limit_lower_ ||
        std::fabs(angles.angle4) > limit_upper_ || std::fabs(angles.angle4) < limit_lower_)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Too close to joint limit");
      rollbackTarget(cmd_x, cmd_y, cmd_z, cmd_rot);
      publishCurrentState();
      return;
    }

    current_angles_ = angles;
    publishCurrentState();
    publishTrajectory(current_angles_);
  }

  void rollbackTarget(float cmd_x, float cmd_y, float cmd_z, float cmd_rot)
  {
    target_point_.point.x -= cmd_x;
    target_point_.point.y -= cmd_y;
    target_point_.point.z -= cmd_z;
    target_angle_ -= cmd_rot;
  }

  void publishCurrentState()
  {
    target_point_.header.stamp = this->now();
    point_pub_->publish(target_point_);

    if (!publish_joint_states_) {
      return;
    }

    arm_mock_.setAngle(current_angles_);
    auto js = arm_mock_.getJointState();
    js.header.stamp = this->now();
    joint_state_pub_->publish(js);
  }

  void publishTrajectory(const Angle4D & angles)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = this->now();
    traj.joint_names = {
      "crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3", "crane_plus_joint4", "crane_plus_joint_hand"
    };

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {
      static_cast<double>(angles.angle1),
      static_cast<double>(angles.angle2),
      static_cast<double>(angles.angle3),
      static_cast<double>(angles.angle4),
      static_cast<double>(angles.angle5)
    };
    pt.time_from_start = rclcpp::Duration::from_seconds(trajectory_time_sec_);
    traj.points.push_back(pt);
    trajectory_pub_->publish(traj);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  ArmMock arm_mock_;
  ArmSolver arm_solver_;

  geometry_msgs::msg::PointStamped target_point_;
  float target_angle_{0.0f};
  float limit_upper_{1.5f};
  float limit_lower_{-1.5f};
  Angle4D current_angles_;

  bool publish_joint_states_{true};
  bool received_joy_{false};
  int timer_period_ms_{50};
  double trajectory_time_sec_{0.05};
  std::string trajectory_topic_;

  std::mutex cmd_mutex_;
  float cmd_x_{0.0f};
  float cmd_y_{0.0f};
  float cmd_z_{0.0f};
  float cmd_rot_{0.0f};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmIkJoyNode>());
  rclcpp::shutdown();
  return 0;
}
