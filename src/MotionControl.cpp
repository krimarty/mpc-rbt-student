#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"
#include "tf2/utils.h"

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {

        // Subscribers for odometry and laser scans
        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));

        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/tiago_base/Hokuyo_URG_04LX_UG01", rclcpp::SensorDataQoS(),
            std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));

        // Publisher for robot control
        twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Client for path planning
        plan_client_ = create_client<nav_msgs::srv::GetPlan>("plan_path");

        // Action server
        nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
            this, "/go_to_goal",
            std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
            std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Motion control node started.");

        // Connect to path planning service server
        while (!plan_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "Waiting for planning service...");
        }
        RCLCPP_INFO(get_logger(), "Planning service connected.");
    }

void MotionControlNode::checkCollision() {
    if (laser_scan_.ranges.empty()) return;
    if (!goal_handle_ || !goal_handle_->is_executing()) return;

    const double thresh     = 0.4;   // stop distance [m]
    const double half_arc   = 40.0 * M_PI / 180.0;  // ±40° forward arc

    int n = static_cast<int>(laser_scan_.ranges.size());
    double angle = laser_scan_.angle_min;
    double angle_inc = laser_scan_.angle_increment;

    for (int i = 0; i < n; i++, angle += angle_inc) {
        if (std::abs(angle) > half_arc) continue;

        float r = laser_scan_.ranges[i];
        if (std::isfinite(r) && r < thresh) {
            RCLCPP_WARN(get_logger(), "Collision risk at %.2f m (beam %.1f deg), aborting.", r, angle * 180.0 / M_PI);

            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);

            auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
            goal_handle_->abort(result);
            return;
        }
    }
}

geometry_msgs::msg::Point MotionControlNode::getTarget(double L) {
    auto & poses = path_.poses;
    int n = static_cast<int>(poses.size());

    // default: last waypoint
    geometry_msgs::msg::Point target;
    target.x = poses.back().pose.position.x;
    target.y = poses.back().pose.position.y;

    double rx = current_pose_.pose.position.x;
    double ry = current_pose_.pose.position.y;

    for (int i = 0; i < n - 1; i++) {
        double w1x = poses[i].pose.position.x;
        double w1y = poses[i].pose.position.y;
        double w2x = poses[i + 1].pose.position.x;
        double w2y = poses[i + 1].pose.position.y;

        double dx = w2x - w1x;
        double dy = w2y - w1y;
        double fx = w1x - rx;
        double fy = w1y - ry;

        double a = dx * dx + dy * dy;
        double b = 2.0 * (fx * dx + fy * dy);
        double c = fx * fx + fy * fy - L * L;

        double disc = b * b - 4.0 * a * c;

        if (disc >= 0.0) {
            double t2 = (-b + std::sqrt(disc)) / (2.0 * a);
            if (t2 >= 0.0 && t2 <= 1.0) {
                target.x = w1x + t2 * dx;
                target.y = w1y + t2 * dy;
            }
        }
    }

    return target;
}

void MotionControlNode::updateTwist() {
    const double L          = 0.3;   // lookahead distance [m]
    const double v_max      = 0.3;   // max linear velocity [m/s]
    const double interwheel = 2.0 * robot_config::HALF_DISTANCE_BETWEEN_WHEELS;
    const double w_max      = 2.0 * v_max / interwheel;
    const double w_max_turn = w_max;

    geometry_msgs::msg::Point target = getTarget(L);

    // Transform target from map frame to robot frame
    double rx    = current_pose_.pose.position.x;
    double ry    = current_pose_.pose.position.y;
    double theta = tf2::getYaw(current_pose_.pose.orientation);

    double v, w;
    if (!heading_aligned_) {
        double angle_path = std::atan2(
            path_.poses[1].pose.position.y - path_.poses[0].pose.position.y,
            path_.poses[1].pose.position.x - path_.poses[0].pose.position.x);
        double heading_error = std::remainder(angle_path - theta, 2.0 * M_PI);

        if (std::abs(heading_error) > 20.0 * M_PI / 180.0) {
            v = 0.0;
            w = w_max_turn * (heading_error >= 0.0 ? 1.0 : -1.0);
        } else {
            heading_aligned_ = true;
        }
    }

    if (heading_aligned_) {
        double dx = target.x - rx;
        double dy = target.y - ry;

        // y-coordinate in robot frame (lateral offset) — matches MATLAB G_robot(2)
        double y_local = -std::sin(theta) * dx + std::cos(theta) * dy;

        // Pure pursuit: radius of curvature
        double R = (L * L) / (2.0 * y_local);
        double R_min = v_max / w_max;

        if (std::abs(R) >= R_min) {
            v = v_max;
            w = v / R;
        } else {
            w = w_max * (R >= 0.0 ? 1.0 : -1.0);
            v = R_min * w_max;
        }
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x  = v;
    twist.angular.z = w;
    twist_publisher_->publish(twist);
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(get_logger(), "Received goal: (%.2f, %.2f)",
        goal->pose.pose.position.x, goal->pose.pose.position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Goal cancel requested.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlNode::navHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    goal_handle_ = goal_handle;
    goal_pose_ = goal_handle->get_goal()->pose;
    heading_aligned_ = false;

    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start = current_pose_;
    request->goal = goal_pose_;

    auto future = plan_client_->async_send_request(request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
}

void MotionControlNode::execute() {
    RCLCPP_INFO(get_logger(), "Execute started.");
    rclcpp::Rate loop_rate(10.0);

    auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();

    while (rclcpp::ok() && goal_handle_->is_executing()) {
        if (goal_handle_->is_canceling()) {
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            goal_handle_->canceled(result);
            RCLCPP_INFO(get_logger(), "Goal canceled.");
            return;
        }

        double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
        double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        feedback->distance_remaining = static_cast<float>(dist);
        feedback->current_pose = current_pose_;
        goal_handle_->publish_feedback(feedback);

        if (dist < 0.15) {
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            goal_handle_->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal reached!");
            return;
        }

        loop_rate.sleep();
    }
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    auto response = future.get();
    if (response && response->plan.poses.size() > 0) {
        path_ = response->plan;
        RCLCPP_INFO(get_logger(), "Path received: %zu waypoints.", path_.poses.size());
        goal_handle_->execute();
        std::thread(&MotionControlNode::execute, this).detach();
    } else {
        RCLCPP_ERROR(get_logger(), "No path found, aborting goal.");
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        goal_handle_->execute();
        goal_handle_->abort(result);
    }
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    current_pose_.header = msg.header;
    current_pose_.pose = msg.pose.pose;

    if (goal_handle_ && goal_handle_->is_executing()) {
        updateTwist();
    }
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    laser_scan_ = msg;
    checkCollision();
}
