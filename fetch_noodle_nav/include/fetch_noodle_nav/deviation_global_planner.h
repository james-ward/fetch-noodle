#ifndef deviation_global_planner_h
#define deviation_global_planner_h

#include <angles/angles.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>

#include <Eigen/Core>

using Point = Eigen::Vector2d;

namespace deviation_global_planner {
class DeviationGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
  DeviationGlobalPlanner() = default;
  DeviationGlobalPlanner(std::string name,
                         costmap_2d::Costmap2DROS *costmap_ros);

  void initialize(std::string name,
                  costmap_2d::Costmap2DROS *costmap_ros) override;
  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan) override;

private:
  static Point quadraticBezier(const Point &p0, const Point &p1,
                               const Point &p2, const double t);
  static std::pair<Point, Point> cubicBezier(const Point &p0, const Point &p1,
                                             const Point &p2, const Point &p3,
                                             const double t);
  void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path);

  ros::NodeHandle private_nh_;
  ros::Publisher plan_pub_;
};
} // namespace deviation_global_planner

PLUGINLIB_EXPORT_CLASS(deviation_global_planner::DeviationGlobalPlanner,
                       nav_core::BaseGlobalPlanner)

#endif
