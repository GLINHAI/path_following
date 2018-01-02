#include <vector>
#include <string>
#include <sstream>
#include <Eigen/Eigen>

namespace path_following {

class Pose3d {
 public:
  Pose3d(const Eigen::Vector3d& position, double pitch, double yaw) :
    position_(position), pitch_(pitch), yaw_(yaw) { }

  double x() const { return position_.x(); }
  double y() const { return position_.y(); }
  double z() const { return position_.z(); }
  double pitch() const { return pitch_; }
  double yaw() const { return yaw_; }

  std::string toString() const {
    std::stringstream ss;
    ss << position_.x() << " " << position_.y() << " " << position_.z() << " ";
    ss << pitch_ << " " << yaw_;
    return ss.str();
  }

 private:
  Eigen::Vector3d position_;
  double pitch_;
  double yaw_;
};  // class Pose3d

struct RobotState {
  Pose3d pose;
  Eigen::Vector3d linear_speed;
  double curvature;

  RobotState(const Pose3d& p,
             const Eigen::Vector3d v,
             double c) : pose(p), linear_speed(v), curvature(c) { }
  std::string toString() const {
    std::stringstream ss;
    ss << pose.toString() << " ";
    ss << linear_speed.x() << " "
       << linear_speed.y() << " "
       << linear_speed.z() << " "
       << curvature;
    return ss.str();
  }
};  // struct RobotState

typedef std::vector<RobotState> Path;

}  // namespace path_following
