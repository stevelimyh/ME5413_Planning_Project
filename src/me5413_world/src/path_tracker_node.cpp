/** path_tracker_node.cpp
 *
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 *
 * MIT License
 *
 * ROS Node for robot to track a given path
 */

#include "me5413_world/math_utils.hpp"
#include "me5413_world/path_tracker_node.hpp"

namespace me5413_world
{

// Dynamic Parameters
double SPEED_TARGET;

double LOOKAHEAD_DISTANCE, STEERING_GAIN, MAX_STEERING_RATE;

// double PID_Kp, PID_Ki, PID_Kd;
// double STANLEY_K;
bool PARAMS_UPDATED;

void dynamicParamCallback(const me5413_world::path_trackerConfig& config, uint32_t level)
{
  // Common Params
  SPEED_TARGET = config.speed_target;
  // // PID
  // PID_Kp = config.PID_Kp;
  // PID_Ki = config.PID_Ki;
  // PID_Kd = config.PID_Kd;
  // // Stanley
  // STANLEY_K = config.stanley_K;

  // Pure Pursuit
  LOOKAHEAD_DISTANCE = config.lookahead_distance;
  STEERING_GAIN = config.steering_gain;
  MAX_STEERING_RATE = config.max_steering_rate;


  ;

  PARAMS_UPDATED = true;
}

PathTrackerNode::PathTrackerNode() : tf2_listener_(tf2_buffer_)
{
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
  this->sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
  this->pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);

  // Initialization
  this->robot_frame_ = "base_link";
  this->world_frame_ = "world";

  // this->pid_ = control::PID(0.1, 1.0, -1.0, PID_Kp, PID_Ki, PID_Kd);
}

void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
{
  // Calculate absolute errors (wrt to world frame)
  current_path_ = *path;
  this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_));

  // this->pose_world_goal_ = path->poses[11].pose;
  // this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_, this->pose_world_goal_));

  return;
}

void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->odom_world_robot_ = *odom.get();

  return;
}

double PathTrackerNode::getYawFromQuaternion(const geometry_msgs::Quaternion& quat) {
    tf2::Quaternion tfQuat;
    tf2::fromMsg(quat, tfQuat);
    tf2::Matrix3x3 m(tfQuat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

// Utility function to calculate distance between two points
double PathTrackerNode::distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

// Finds and returns the lookahead point on the path
geometry_msgs::Point PathTrackerNode::findLookaheadPoint(const geometry_msgs::Point& robot_position, const nav_msgs::Path& path, double lookahead_distance) {
    for (size_t i = 0; i < path.poses.size() - 1; ++i) {
        auto& start = path.poses[i].pose.position;
        auto& end = path.poses[i + 1].pose.position;

        // Project robot position onto the path segment
        double r2 = std::pow(distance(start, end), 2);
        double t = std::max(0.0, std::min(1.0, ((robot_position.x - start.x) * (end.x - start.x) + (robot_position.y - start.y) * (end.y - start.y)) / r2));
        
        geometry_msgs::Point projection;
        projection.x = start.x + t * (end.x - start.x);
        projection.y = start.y + t * (end.y - start.y);
        projection.z = 0.0;
        double d = distance(robot_position, projection);

        // If the projected point to the robot distance plus a small epsilon for numerical stability is greater than the lookahead distance,
        // use this segment to calculate the exact lookahead point.
        if (d + 1e-6 < lookahead_distance) {
            double remaining_distance = lookahead_distance - d;
            double segment_length = distance(start, end);

            if (segment_length > 0.0 && remaining_distance < segment_length) {
                double ratio = remaining_distance / segment_length;
                geometry_msgs::Point lookahead_point;
                lookahead_point.x = start.x + ratio * (end.x - start.x);
                lookahead_point.y = start.y + ratio * (end.y - start.y);
                lookahead_point.z = 0.0; 
                return lookahead_point;
            }
        }
    }

    // Fallback: return the last point if no valid lookahead point is found
    return path.poses.back().pose.position;
}


// double PathTrackerNode::computeStanelyControl(const double heading_error, const double cross_track_error, const double velocity)
// {
//   const double stanley_output = -1.0*(heading_error + std::atan2(STANLEY_K*cross_track_error, std::max(velocity, 0.3)));

//   return std::min(std::max(stanley_output, -2.2), 2.2);
// }

geometry_msgs::Twist PathTrackerNode::computeControlOutputs(const nav_msgs::Odometry& odom_robot) {
    // Ensure the path is not empty
    if (current_path_.poses.empty()) {
        ROS_WARN("Current path is empty.");
        return geometry_msgs::Twist();
    }

    // Find the robot's current position and orientation
    geometry_msgs::Point robot_position = odom_robot.pose.pose.position;
    double robot_yaw = getYawFromQuaternion(odom_robot.pose.pose.orientation);

    // Find the lookahead point on the path
    geometry_msgs::Point lookahead_point = findLookaheadPoint(robot_position, current_path_, LOOKAHEAD_DISTANCE);

    // Calculate the angle to the lookahead point from the robot's current position
    double angle_to_lookahead = atan2(lookahead_point.y - robot_position.y, lookahead_point.x - robot_position.x);

    double heading_error = angle_to_lookahead - robot_yaw;

    heading_error = atan2(sin(heading_error), cos(heading_error));

    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = SPEED_TARGET;

    double steering_command = STEERING_GAIN * heading_error;

    if (std::abs(steering_command) > MAX_STEERING_RATE) {
        steering_command = std::copysign(MAX_STEERING_RATE, steering_command); // Retains the sign of the original command
    }

    cmd_vel.angular.z = steering_command;

    return cmd_vel;
}
// geometry_msgs::Twist PathTrackerNode::computeControlOutputs(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal)
// {
//   // Heading Error
//   tf2::Quaternion q_robot, q_goal;
//   tf2::fromMsg(odom_robot.pose.pose.orientation, q_robot);
//   tf2::fromMsg(pose_goal.orientation, q_goal);
//   const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
//   const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_goal);

//   double roll, pitch, yaw_robot, yaw_goal;
//   m_robot.getRPY(roll, pitch, yaw_robot);
//   m_goal.getRPY(roll, pitch, yaw_goal);

//   const double heading_error = unifyAngleRange(yaw_robot - yaw_goal);

//   // Lateral Error
//   tf2::Vector3 point_robot, point_goal;
//   tf2::fromMsg(odom_robot.pose.pose.position, point_robot);
//   tf2::fromMsg(pose_goal.position, point_goal);
//   const tf2::Vector3 V_goal_robot = point_robot - point_goal;
//   const double angle_goal_robot = std::atan2(V_goal_robot.getY(), V_goal_robot.getX());
//   const double angle_diff = angle_goal_robot - yaw_goal;
//   const double lat_error = V_goal_robot.length()*std::sin(angle_diff);

//   // Velocity
//   tf2::Vector3 robot_vel;
//   tf2::fromMsg(this->odom_world_robot_.twist.twist.linear, robot_vel);
//   const double velocity = robot_vel.length();

//   geometry_msgs::Twist cmd_vel;
//   if (PARAMS_UPDATED)
//   {
//     this->pid_.updateSettings(PID_Kp, PID_Ki, PID_Kd);
//     PARAMS_UPDATED = false;
//   }
//   cmd_vel.linear.x = this->pid_.calculate(SPEED_TARGET, velocity);
//   cmd_vel.angular.z = computeStanelyControl(heading_error, lat_error, velocity);

//   // std::cout << "robot velocity is " << velocity << " throttle is " << cmd_vel.linear.x << std::endl;
//   // std::cout << "lateral error is " << lat_error << " heading_error is " << heading_error << " steering is " << cmd_vel.angular.z << std::endl;

//   return cmd_vel;
// }

} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_tracker_node");
  me5413_world::PathTrackerNode path_tracker_node;
  ros::spin();  // spin the ros node.
  return 0;
}
