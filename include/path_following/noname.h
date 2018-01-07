#include <vector>
#include <cstddef>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

#include "Robotstate.h"

namespace Send_path_segment {

extern Path path;

class Send_path_segment {
 public:
  Send_path_segment(ros::NodeHandle& n);
  ~Send_path_segment(){ };
  std::vector<size_t> find_closest_points(double x, 
                                          double y, 
                                          const Path& path);
  double find_distance(int x1, int x2, const Path& path);
  std::vector<size_t> find_closest_index(const std::vector<size_t>& des, 
                                         const std::vector<size_t>& cur);
  void callback_estop(const std_msgs::Bool::ConstPtr& data);
  void callback_goal_index(const geometry_msgs::PoseStamped::ConstPtr& goal);
  void callback_send_path(const nav_msgs::Odometry::ConstPtr& data);

 private:
  ros::Publisher pub_test_;
  ros::Publisher pub_plan_;
  size_t closest_goal_index_;
  bool get_goal_;
  size_t current_location_index_;
  float pre_time_;
  float time_step_;
  bool e_stop_;
  std::vector<size_t> closest_goal_list_;
  std::vector<size_t> current_location_list_;
};  // class SendPathSegment

}  // namespace SendPathSegment
