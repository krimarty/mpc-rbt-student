#include "Localization.hpp"

#include "mpc_rbt_simulator/RobotConfig.hpp"

LocalizationNode::LocalizationNode()
: rclcpp::Node("localization_node"), last_time_(this->get_clock()->now())
{
  // Odometry message initialization
  odometry_.header.frame_id = "map";
  odometry_.child_frame_id = "base_link";
  odometry_.pose.pose.position.x = -0.5;
  odometry_.pose.pose.position.y = 0.0;
  odometry_.pose.pose.orientation.w = 1.0;  // identity quaternion (theta=0)
  // add code here

  // Subscriber for joint_states
  joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1));

  // Publisher for odometry
  odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

  // tf_briadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg)
{
  RCLCPP_INFO(
    get_logger(), "jointCallback: left=%.3f right=%.3f", msg.velocity[0], msg.velocity[1]);

  auto current_time = this->get_clock()->now();
  double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;

  updateOdometry(msg.velocity[1], msg.velocity[0], dt);  // [0]=right, [1]=left
  publishOdometry();
  publishTransform();
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt)
{
  double linear = robot_config::WHEEL_RADIUS * (left_wheel_vel + right_wheel_vel) / 2.0;  //m/s
  double angular = robot_config::WHEEL_RADIUS * (right_wheel_vel - left_wheel_vel) /      //rad/s
                   (2.0 * robot_config::HALF_DISTANCE_BETWEEN_WHEELS);

  tf2::Quaternion tf_quat;
  tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
  double roll, pitch, theta;
  tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

  odometry_.pose.pose.position.x += linear * std::cos(theta + angular / 2) * dt;
  odometry_.pose.pose.position.y += linear * std::sin(theta + angular / 2) * dt;
  theta += angular * dt;
  theta = std::atan2(std::sin(theta), std::cos(theta));  // Normalize angle to [-pi, pi]

  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  q.normalize();
  odometry_.pose.pose.orientation = tf2::toMsg(q);

  odometry_.twist.twist.linear.x = linear;
  odometry_.twist.twist.angular.z = angular;

  // ********
  // * Help *
  // ********
  /*
    double linear =  ;
    double angular = ;  //robot_config::HALF_DISTANCE_BETWEEN_WHEELS

    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    theta = std::atan2(std::sin(theta), std::cos(theta));

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    */
}

void LocalizationNode::publishOdometry()
{
  odometry_.header.stamp = this->get_clock()->now();
  odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform()
{
  // add code here
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  t.transform.translation.x = odometry_.pose.pose.position.x;
  t.transform.translation.y = odometry_.pose.pose.position.y;
  t.transform.translation.z = 0.0;
  t.transform.rotation = odometry_.pose.pose.orientation;

  // ********
  // * Help *
  // ********
  tf_broadcaster_->sendTransform(t);
}
