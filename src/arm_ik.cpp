#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "tomato_arm_ik/arm_ik.hpp"

class ArmIkDemoNode : public rclcpp::Node
{
public:
  ArmIkDemoNode()
  : Node("arm_ik_node"),
    arm_mock_(0.05f, 0.05f, 0.15f, 0.15f, 0.1f),
    arm_solver_(0.05f, 0.05f, 0.15f, 0.15f, 0.1f)
  {
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("target_point", 1);

    Angle4D init_angles;
    arm_smooth_.setCurrentAngles(init_angles);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ArmIkDemoNode::loop, this));
  }

private:
  void loop()
  {
    geometry_msgs::msg::PointStamped target_point;
    target_point.header.stamp = this->now();
    target_point.header.frame_id = "base_link";

    if (phase_ == 0) {
      target_point.point.x = 0.3;
      target_point.point.y = 0.1;
      target_point.point.z = 0.2;
    } else {
      target_point.point.x = 0.2;
      target_point.point.y = -0.1;
      target_point.point.z = 0.3;
    }

    Angle4D angles;
    if (!arm_solver_.solve(target_point.point, 1.5707f, angles)) {
      RCLCPP_WARN(this->get_logger(), "can not solve");
      return;
    }

    if (count_ == 0) {
      arm_smooth_.setTargetAngles(angles);
    }

    arm_mock_.setAngle(arm_smooth_.output(static_cast<float>(count_) / 20.0f));
    auto js = arm_mock_.getJointState();
    js.header.stamp = this->now();

    point_pub_->publish(target_point);
    joint_state_pub_->publish(js);

    count_++;
    if (count_ > 20) {
      count_ = 0;
      phase_ = 1 - phase_;
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  ArmMock arm_mock_;
  ArmSolver arm_solver_;
  ArmSmooth arm_smooth_;
  int phase_{0};
  int count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmIkDemoNode>());
  rclcpp::shutdown();
  return 0;
}
