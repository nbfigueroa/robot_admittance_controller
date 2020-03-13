#include "ros/ros.h"
#include "AdmittanceController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "admittance_controller_node");

  ros::NodeHandle nh;
  double frequency = 150.0;

  // Parameters
  std::string topic_arm_pose;
  std::string topic_arm_twist;
  std::string topic_arm_command;
  std::string topic_external_wrench;
  std::string topic_control_wrench;
  std::string topic_ds_velocity;
  std::string topic_external_wrench_arm_frame;
  std::string topic_control_external_arm_frame;
  std::string topic_admittance_ratio;

  std::vector<double> M_a;
  std::vector<double> D_a;
  std::vector<double> workspace_limits;

  double arm_max_vel;
  double arm_max_acc;


  // LOADING PARAMETERS FROM THE ROS SERVER

  // Topic names
  if (!nh.getParam("topic_arm_pose", topic_arm_pose)) {
    ROS_ERROR("Couldn't retrieve the topic name for the EE pose in the world frame.");
    return -1;
  }

  if (!nh.getParam("topic_arm_twist", topic_arm_twist)) {
    ROS_ERROR("Couldn't retrieve the topic name for the EE twist in the world frame.");
    return -1;
  }

  if (!nh.getParam("topic_external_wrench", topic_external_wrench)) {
    ROS_ERROR("Couldn't retrieve the topic name for the force/torque sensor.");
    return -1;
  }

  if (!nh.getParam("topic_control_wrench", topic_control_wrench)) {
    ROS_ERROR("Couldn't retrieve the topic name for the control wrench.");
    return -1;
  }

  if (!nh.getParam("topic_arm_command", topic_arm_command)) {
    ROS_ERROR("Couldn't retrieve the topic name for commanding the arm.");
    return -1;
  }

  // ADMITTANCE PARAMETERS
  if (!nh.getParam("mass_arm", M_a)) {
    ROS_ERROR("Couldn't retrieve the desired mass of the arm.");
    return -1;
  }

  if (!nh.getParam("damping_arm", D_a)) {
    ROS_ERROR("Couldn't retrieve the desired damping of the arm.");
    return -1;
  }


  // SAFETY PARAMETERS
  if (!nh.getParam("workspace_limits", workspace_limits)) {
    ROS_ERROR("Couldn't retrieve the limits of the workspace.");
    return -1;
  }

  if (!nh.getParam("arm_max_vel", arm_max_vel)) {
    ROS_ERROR("Couldn't retrieve the max velocity for the arm.");
    return -1;
  }

  if (!nh.getParam("arm_max_acc", arm_max_acc)) {
    ROS_ERROR("Couldn't retrieve the max acceleration for the arm.");
    return -1;
  }


  // Constructing the controller
  AdmittanceController admittance_controller(
    nh,
    frequency,
    topic_arm_pose,
    topic_arm_twist,
    topic_external_wrench,
    topic_control_wrench,
    topic_arm_command,
    M_a, D_a,
    workspace_limits,
    arm_max_vel, arm_max_acc);

  // Running the controller
  admittance_controller.run();

  return 0;
}
