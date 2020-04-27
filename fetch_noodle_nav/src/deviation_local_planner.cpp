#include <fetch_noodle_nav/deviation_local_planner.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using geometry_msgs::Twist;
using std::nullopt;
using std::optional;
using std::vector;

namespace deviation_local_planner {
  bool DeviationLocalPlanner::computeVelocityCommands(Twist& cmd_vel) {
    if (!is_running_) {
      is_running_ = true;
      start_time_ = ros::Time::now();
    }
    auto p = get_pose();
    if (!p) {
      return false;
    }
    auto pose = p->pose;
    double d = norm(pose, plan_[current_target_].pose);
    while (d < look_ahead_ && current_target_ < plan_.size() - 1) {
      current_target_++;
      d = norm(pose, plan_[current_target_].pose);
    }
    auto target = Pose(plan_[current_target_].pose);
    if (d < look_ahead_ && current_target_ == plan_.size()-1) {
      // create a false look ahead point
      auto bearing = tf2::getYaw(target.orientation);
      target.position.x += look_ahead_ * std::cos(bearing);
      target.position.y += look_ahead_ * std::sin(bearing);
    }

    auto remaining = norm(pose, target);
    double scaling = std::max(0.1, std::min(1.0, remaining/stopping_distance_));
    auto now = ros::Time::now();
    double t = (start_time_ - now).toSec();
    cmd_vel.linear.x = (speed_ - amplitude_ * sin(2.0*3.14159*t/period_)) * scaling;

    auto yaw = tf2::getYaw(pose.orientation);
    auto bearing = std::atan2(target.position.y - pose.position.y,
        target.position.x - pose.position.x);
    auto delta = bearing - yaw;
    delta = std::atan2(std::sin(delta), std::cos(delta));
    cmd_vel.angular.z = delta / (look_ahead_/cmd_vel.linear.x);

    return true;
  }

  bool DeviationLocalPlanner::isGoalReached() {
    auto p = get_pose();
    if (p) {
      auto pose = p->pose;
      auto goal = plan_[plan_.size()-1].pose;
      double d = norm(pose, goal);
      return d < goal_tolerance_;
    }
    return true;
  }

  bool DeviationLocalPlanner::setPlan(const vector<PoseStamped>& plan) {
    plan_ = plan;
    is_running_ = false;
    current_target_ = 0;
    private_nh_.param("speed/amplitude", amplitude_, 0.5);
    private_nh_.param("speed/period", period_, 5.0);
    private_nh_.param("speed/base_speed", speed_, 2.0);
    private_nh_.param("stopping_distance", stopping_distance_, 1.0);
    private_nh_.param("look_ahead", look_ahead_, 0.5);
    private_nh_.param("goal_tolerance", goal_tolerance_, 0.2);
    return true;
  }

  void DeviationLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    tf_ = tf;
    private_nh_ = ros::NodeHandle("~/" + name);
  }

  optional<PoseStamped> DeviationLocalPlanner::get_pose() {
    PoseStamped global_pose, robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    global_pose.header.frame_id = plan_[0].header.frame_id;
    robot_pose.header.frame_id = "base_link";
    robot_pose.header.stamp = ros::Time();

    // get the global pose of the robot
    try
    {
      tf_->transform(robot_pose, global_pose, plan_[0].header.frame_id);
    }
    catch (tf2::LookupException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return nullopt;
    }
    catch (tf2::ConnectivityException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return nullopt;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return nullopt;
    }

    return global_pose;
  }

  double norm(const Pose &p1, const Pose &p2) {
    return std::hypot(p1.position.x - p2.position.x,
        p1.position.y - p2.position.y);
  }
}