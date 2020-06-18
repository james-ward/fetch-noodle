#include <fetch_noodle_nav/deviation_global_planner.h>

int main(void) {
  const Point p0(8.3, 2.5);
  const Point p1(7.4, -0.4);
  const Point p2(3.5, 3.3);
  const Point p3(1.5, 3.3);
  ROS_INFO_STREAM("Bezier points:");
  ROS_INFO_STREAM(p0);
  ROS_INFO_STREAM(p1);
  ROS_INFO_STREAM(p2);
  ROS_INFO_STREAM(p3);

  double dist = 0;
  Point prev(p0);

  std::vector<std::pair<Point, Point>> curve;
  const double steps = 20;
  ROS_INFO_STREAM("Bezier points");
  for (double i = 1; i < steps; i++) {
    const double t = i / steps;
    auto cb = deviation_global_planner::DeviationGlobalPlanner::cubicBezier(p0, p1, p2, p3, t);
    auto p = cb.first;
    ROS_INFO_STREAM(p);
    auto gradient = cb.second;
  }
}
