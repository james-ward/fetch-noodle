#ifndef deviation_local_planner_h
#define deviation_local_planner_h

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>

#include <pluginlib/class_list_macros.h>

namespace deviation_local_planner {

double norm(const geometry_msgs::Pose&, const geometry_msgs::Pose&);

class DeviationLocalPlanner : public nav_core::BaseLocalPlanner {
public:
  DeviationLocalPlanner() = default;
  ~DeviationLocalPlanner() = default;

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
  bool isGoalReached();
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
  void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

private:
  geometry_msgs::PoseStamped get_pose();

  tf::TransformListener* tf_;
  std::vector<geometry_msgs::PoseStamped> plan_;
  ros::NodeHandle private_nh_;

  double goal_tolerance_ = 0.4;
  double look_ahead_ = 0.5;
  double stopping_distance_ = 1.0;
  int current_target_ = 0;
  double period_ = 5.0;
  double amplitude_ = 0.5;
  double speed_ = 2.0;
  ros::Time start_time_;
  bool is_running_;
  bool aligning_;
};
}

PLUGINLIB_EXPORT_CLASS(deviation_local_planner::DeviationLocalPlanner,
                       nav_core::BaseLocalPlanner)

#endif

