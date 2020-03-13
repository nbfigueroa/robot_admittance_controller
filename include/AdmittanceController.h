#ifndef ADMITTANCECONTROLLER_H
#define ADMITTANCECONTROLLER_H

#include "ros/ros.h"

// #include "cartesian_state_msgs/PoseTwist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include "std_msgs/Float32.h"


// AdmittanceController class //
// A simple class implementing an admittance
// controller for the ridgeback + UR5 platform at the LASA lab. An
// AdmittanceController is a while loop that:
// 1) Reads the robot state (arm+platform)
// 1) Sends a desired twist to the robot platform
// 2) Sends a desired twist to the robot arm
//
// Communication interfaces:
// - The desired twists are both sent through ROS topics.
// The low level controllers of both the arm and the platform are
// implemented in ROS control.
// NOTE: This node should be ideally a ROS
// controller, but the current implementation of ROS control in
// indigo requires writing a specific interface for this matter.
// The kinetic version of ROS control enables controllers with
// multiple interfaces (FT sensor, platform and arm), but a proper
// ROS control implementation of this controller remains future work.
//
// USAGE EXAMPLE;
// ros::NodeHandle nh;
// double frequency = 1000.0;
// std::string state_topic_arm, cmd_topic_arm, topic_arm_twist_world,
//   topic_wrench_u_e, topic_wrench_u_c, cmd_topic_platform,
//   state_topic_platform, wrench_topic, wrench_control_topic,
//   laser_front_topic, laser_rear_topic;
// std::vector<double> M_p, M_a, D, D_p, D_a, K, d_e;
// double wrench_filter_factor, force_dead_zone_thres,
//    torque_dead_zone_thres, obs_distance_thres, self_detect_thres;
//
//
// // Fill in values
// ...
//
//
// AdmittanceController admittance_controller(nh, frequency,
//                                           cmd_topic_platform,
//                                           state_topic_platform,
//                                           cmd_topic_arm,
//                                           topic_arm_twist_world,
//                                           topic_wrench_u_e,
//                                           topic_wrench_u_c,
//                                           state_topic_arm,
//                                           wrench_topic,
//                                           wrench_control_topic,
//                                           laser_front_topic,
//                                           laser_rear_topic,
//                                           M_p, M_a, D, D_p, D_a, K, d_e,
//                                           wrench_filter_factor,
//                                           force_dead_zone_thres,
//                                           torque_dead_zone_thres,
//                                           obs_distance_thres,
//                                           self_detect_thres);
// admittance_controller.run();

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

class AdmittanceController
{
protected:
  // ---  ROS VARIABLES --- //
  // A handle to the node in ros
  ros::NodeHandle nh_;
  // Rate of the run loop
  ros::Rate loop_rate_;


  // --- Subscribers --- //

  // Subscriber for the arm state
  ros::Subscriber sub_arm_pose_;
  ros::Subscriber sub_arm_twist_;
  // Subscriber for the ft sensor at the end-effector
  ros::Subscriber sub_wrench_external_;
  // Subscriber for the control input (wrench)
  ros::Subscriber sub_wrench_control_;


  // --- Publishers --- //
  // Publisher for the twist of arm endeffector
  ros::Publisher pub_arm_cmd_;
  // Publisher for the external wrench specified in the world frame
  // ros::Publisher pub_wrench_external_;
  // Publisher for the control wrench specified in the world frame
  // ros::Publisher pub_wrench_control_;


  // --- INPUT SIGNAL --- //
  // external wrench (force/torque sensor) in "robotiq_force_torque_frame_id" frame
  Vector6d wrench_external_;
  // control wrench (from any controller) expected to be in "ur5_arm_base_link" frame
  Vector6d wrench_control_;


  // --- ADMITTANCE PARAMETERS --- //
  // M_a_ -> Desired mass of arm
  // D_a_ -> Desired damping of arm
  Matrix6d M_a_, D_a_;

  // --- OUTPUT COMMANDS --- //
  // final arm desired velocity 
  Vector6d arm_desired_twist_;

  // limiting the workspace of the arm
  Vector6d workspace_limits_;
  double arm_max_vel_;
  double arm_max_acc_;

  // --- STATE VARIABLES -- //
  // Arm state: position, orientation, and twist (in "arm_base_link")
  Vector3d     arm_real_position_;
  Quaterniond  arm_real_orientation_;
  Vector6d     arm_real_twist_;

  // End-effector state: pose and twist (in "world" frame)
  Vector7d     ee_pose_world_;
  Vector6d     ee_twist_world_;

  // Transform from base_link to world
  Matrix6d rotation_base_;

  // TF:
  // Listeners
  tf::TransformListener listener_ft_;
  tf::TransformListener listener_control_;
  tf::TransformListener listener_arm_;

  // Guards
  bool ft_arm_ready_;
  bool arm_world_ready_;
  bool world_arm_ready_;

  // Initialization
  void wait_for_transformations();

  // Control
  void compute_admittance();

  // Callbacks
  void pose_arm_callback(const geometry_msgs::PoseConstPtr msg);
  void twist_arm_callback(const geometry_msgs::TwistConstPtr msg);
  void wrench_external_callback(const geometry_msgs::WrenchStampedConstPtr msg);
  void wrench_control_callback(const geometry_msgs::WrenchStampedConstPtr msg);


  // Util
  bool get_rotation_matrix(Matrix6d & rotation_matrix,
                           tf::TransformListener & listener,
                           std::string from_frame,  std::string to_frame);

  void limit_to_workspace();

  // void publish_debuggings_signals();

  void send_commands_to_robot();


public:
  AdmittanceController(ros::NodeHandle &n, double frequency,                       
                       std::string topic_arm_pose,
                       std::string topic_arm_twist,
                       std::string wrench_external_topic,
                       std::string wrench_control_topic,
                       std::string cmd_topic_arm,
                       std::vector<double> M_a,
                       std::vector<double> D_a,
                       std::vector<double> workspace_limits,
                       double arm_max_vel,
                       double arm_max_acc);
  void run();
};

#endif // ADMITTANCECONTROLLER_H

