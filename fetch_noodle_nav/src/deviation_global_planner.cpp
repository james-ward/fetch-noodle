#include "fetch_noodle_nav/deviation_global_planner.h"

#include <nav_msgs/Path.h>
#include <tf2/utils.h>

namespace deviation_global_planner {

DeviationGlobalPlanner::DeviationGlobalPlanner(
    std::string name, costmap_2d::Costmap2DROS *costmap_ros) {
  initialize(name, costmap_ros);
}

void DeviationGlobalPlanner::initialize(std::string name,
                                        costmap_2d::Costmap2DROS *costmap_ros) {
  private_nh_ = ros::NodeHandle("~" + name);

  plan_pub_ = private_nh_.advertise<nav_msgs::Path>("plan", 1);
}

bool DeviationGlobalPlanner::makePlan(
    const geometry_msgs::PoseStamped &start,
    const geometry_msgs::PoseStamped &goal,
    std::vector<geometry_msgs::PoseStamped> &plan) {

  const double start_yaw = tf2::getYaw(start.pose.orientation);
  const double goal_yaw = tf2::getYaw(goal.pose.orientation);

  // TODO Read deviation parameters from the parameter server
  double amplitude = 0;
  int half_cycles = 0;
  private_nh_.param("position/amplitude", amplitude, 0.5);
  private_nh_.param("position/half_cycles", half_cycles, 5);

  // Make a cubic Bezier curve for the path
  plan.push_back(start);

  double start_offset = 3;
  double goal_offset = 2;
  private_nh_.param("bezier_start_offset", start_offset, 3.0);
  private_nh_.param("bezier_goal_offset", goal_offset, 2.0);

  const Point p0(start.pose.position.x, start.pose.position.y);
  const Point p3(goal.pose.position.x, goal.pose.position.y);
  const Point p1 =
      p0 + start_offset * Point(std::cos(start_yaw), std::sin(start_yaw));
  const Point p2 =
      p3 - goal_offset * Point(std::cos(goal_yaw), std::sin(goal_yaw));

  double dist = 0;
  Point prev(p0);

  std::vector<std::pair<Point, Point>> curve;
  const double steps = std::max(20, half_cycles * 10);
  for (double i = 1; i < steps; i++) {
    const double t = i / steps;
    auto [p, gradient] = cubicBezier(p0, p1, p2, p3, t);
    gradient.normalize();
    curve.push_back(std::make_pair(p, gradient));
    dist += (p - prev).norm();
    prev = p;
  }

  double l = 0;
  prev = p0;
  for (auto [p, gradient] : curve) {
    l += (p - prev).norm();
    double deviation = amplitude * std::sin(3.14159 * l / dist * half_cycles);
    geometry_msgs::PoseStamped ps(start);
    ps.pose.position.x = p[0] - deviation * gradient[1];
    ps.pose.position.y = p[1] + deviation * gradient[0];
    // TODO set the correct heading
    plan.push_back(ps);
    prev = p;
  }

  plan.push_back(goal);

  // TODO Deviate the path

  // Publish the path for visualisation
  publishPlan(plan);

  return true;
}

void DeviationGlobalPlanner::publishPlan(
    const std::vector<geometry_msgs::PoseStamped> &path) {

  // create a message for the plan
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());

  gui_path.header.frame_id = "map";
  gui_path.header.stamp = ros::Time::now();

  // Extract the plan in world co-ordinates, we assume the path is all in the
  // same frame
  for (unsigned int i = 0; i < path.size(); i++) {
    gui_path.poses[i] = path[i];
  }

  plan_pub_.publish(gui_path);
}

Point DeviationGlobalPlanner::quadraticBezier(const Point &p0, const Point &p1,
                                              const Point &p2, const double t) {
  return p1 + std::pow(1 - t, 2) * (p0 - p1) + std::pow(t, 2) * (p2 - p1);
}

std::pair<Point, Point> DeviationGlobalPlanner::cubicBezier(const Point &p0,
                                                            const Point &p1,
                                                            const Point &p2,
                                                            const Point &p3,
                                                            const double t) {
  auto p = (1 - t) * quadraticBezier(p0, p1, p2, t) +
           t * quadraticBezier(p1, p2, p3, t);
  auto grad = 3 * std::pow(1 - t, 2) * (p1 - p0) + 6 * (1 - t) * t * (p2 - p1) +
              3 * std::pow(t, 2) * (p3 - p2);
  return std::make_pair(p, grad);
}

} // namespace deviation_global_planner
