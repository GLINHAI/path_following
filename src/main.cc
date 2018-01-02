#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include "path_following/robot_state.h"
#include "path_following/TestMsg.h"

using path_following::RobotState;
using path_following::Path;
using path_following::Pose3d;

bool ReadPath(Path* path, std::string filename) {
  std::fstream fins(filename, std::fstream::in);
  if (fins.is_open()) {
    std::string line;
    while (getline(fins, line)) {
      std::stringstream ss(line);
      double x, y, z, pitch, yaw, linear_speed, curvature;
      ss >> x >> y >> z >> pitch >> yaw >> linear_speed >> curvature;
      Eigen::Vector3d position(x, y, z);
      RobotState state(Pose3d(position, pitch, yaw),
                       Eigen::Vector3d(linear_speed, 0, 0),
                       curvature);
      path->emplace_back(state);
    }
  } else {
    std::cerr << "Error: Can not open file " << filename << std::endl;
    return false;
  }

  fins.close();
  return true;
}

int main(int argc, char** argv) {
  Path path;
  if (ReadPath(&path, "test_input.txt")) {
    for (auto state : path) { };
  } else {
    std::cerr << "Error: ReadPath Error" << std::endl;
  }

  ros::init(argc, argv, "path_planning");
  ros::NodeHandle n;

  // ros::Subscriber sub_goal = n.subscribe("goal_point", 1, callback_goal_index);
  // ros::Subscriber sub_position = n.subscribe("global_position", 1, callback_send_path);
  // ros::Subscriber sub_stop = n.subscribe("e_stop", 1, callback_estop);

  ros::spin();

  return 0;
}
