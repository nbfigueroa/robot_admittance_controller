#include "AdmittanceController.h"

AdmittanceController::AdmittanceController(ros::NodeHandle &n,
    double frequency,
    std::string topic_arm_command,
    std::string topic_arm_pose,
    std::string topic_arm_twist,
    std::string topic_external_wrench_arm_frame,
    std::string topic_control_wrench_arm_frame,
    std::string topic_external_wrench,
    std::string topic_control_wrench,
    std::string topic_admittance_ratio,
    std::string topic_ds_velocity,
    std::vector<double> M_a,
    std::vector<double> D_a,
    std::vector<double> workspace_limits,
    double arm_max_vel,
    double arm_max_acc) :
  nh_(n), loop_rate_(frequency),
  M_a_(M_a.data()),
  D_a_(D_a.data()),
  workspace_limits_(workspace_limits.data()),
  arm_max_vel_(arm_max_vel),
  arm_max_acc_(arm_max_acc){


  // Subscribers
  sub_arm_pose_           = nh_.subscribe(topic_arm_pose, 10,
                                 &AdmittanceController::pose_arm_callback, this,
                                 ros::TransportHints().reliable().tcpNoDelay());

  sub_arm_twist_          = nh_.subscribe(topic_arm_twist, 10,
                                 &AdmittanceController::twist_arm_callback, this,
                                 ros::TransportHints().reliable().tcpNoDelay());

  sub_wrench_external_    = nh_.subscribe(topic_external_wrench, 5,
                                       &AdmittanceController::wrench_callback, this,
                                       ros::TransportHints().reliable().tcpNoDelay());
  sub_wrench_control_     = nh_.subscribe(topic_control_wrench, 5,
                                      &AdmittanceController::wrench_control_callback, this,
                                      ros::TransportHints().reliable().tcpNoDelay());

  sub_ds_velocity_        = nh_.subscribe(topic_ds_velocity, 10,
                              &AdmittanceController::ds_velocity_callback , this,
                              ros::TransportHints().reliable().tcpNoDelay());

  sub_admittance_ratio_   = nh_.subscribe(topic_admittance_ratio, 10,
                                        &AdmittanceController::admittance_ratio_callback, this,
                                        ros::TransportHints().reliable().tcpNoDelay());


  // Publishers
  pub_arm_cmd_            = nh_.advertise<geometry_msgs::Twist>(topic_arm_command, 5);

  pub_wrench_external_    = nh_.advertise<geometry_msgs::WrenchStamped>(
                             topic_external_wrench_arm_frame, 5);
  pub_wrench_control_     = nh_.advertise<geometry_msgs::WrenchStamped>(
                             topic_control_wrench_arm_frame, 5);

  ROS_INFO_STREAM("Arm max vel:" << arm_max_vel_ << " max acc:" << arm_max_acc_);


  // initializing the class variables
  wrench_external_.setZero();
  wrench_control_.setZero();

  ee_pose_world_.setZero();
  ee_twist_world_.setZero();

  // setting the robot state to zero and wait for data
  arm_real_position_.setZero();

  while (nh_.ok() && !arm_real_position_(0)) {
    ROS_WARN_THROTTLE(1, "Waiting for the state of the arm...");
    ros::spinOnce();
    loop_rate_.sleep();
  }

  // Init integrator
  arm_desired_twist_adm_.setZero();
  // platform_desired_twist_.setZero();

  arm_desired_twist_ds_.setZero();
  arm_desired_twist_final_.setZero();


  ft_arm_ready_ = false;
  arm_world_ready_ = false;
  base_world_ready_ = false;
  world_arm_ready_ = false;
  admittance_ratio_ = 1;

  wait_for_transformations();
}

///////////////////////////////////////////////////////////////
///////////////////// Control Loop ////////////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::run() {

  ROS_INFO("Running the admittance control loop .................");

  while (nh_.ok()) {
    // Admittance Dynamics computation
    compute_admittance();

    // sum the vel from admittance to DS in this function
    // limit the the movement of the arm to the permitted workspace
    limit_to_workspace();

    // Copy commands to messages
    send_commands_to_robot();

    // publishing visualization/debugging info
    publish_debuggings_signals();

    ros::spinOnce();
    loop_rate_.sleep();
  }


}



///////////////////////////////////////////////////////////////
///////////////////// Admittance Dynamics /////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::compute_admittance() {

  // Vector6d platform_desired_acceleration;
  Vector6d arm_desired_accelaration;

  arm_desired_accelaration = M_a_.inverse() * ( - D_a_ * arm_desired_twist_adm_
                             + admittance_ratio_ * wrench_external_ + wrench_control_);

  // limiting the accelaration for better stability and safety
  double a_acc_norm = (arm_desired_accelaration.segment(0, 3)).norm();
  if (a_acc_norm > arm_max_acc_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high arm accelaration!"
                             << " norm: " << a_acc_norm);
    arm_desired_accelaration.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
  }

  // Integrate for velocity based interface
  ros::Duration duration = loop_rate_.expectedCycleTime();
  arm_desired_twist_adm_  += arm_desired_accelaration * duration.toSec();

}

///////////////////////////////////////////////////////////////
////////////////////////// Callbacks //////////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::pose_arm_callback(
  const geometry_msgs::PoseConstPtr msg) {
  arm_real_position_ << msg->position.x, msg->position.y,
                     msg->position.z;

  arm_real_orientation_.coeffs() << msg->orientation.x,
                               msg->orientation.y,
                               msg->orientation.z,
                               msg->orientation.w;
}

void AdmittanceController::twist_arm_callback(
  const geometry_msgs::TwistConstPtr msg) {

  arm_real_twist_ << msg->linear.x, msg->linear.y,
                  msg->linear.z, msg->angular.x, msg->angular.y,
                  msg->angular.z;
}

void AdmittanceController::wrench_callback(
  const geometry_msgs::WrenchStampedConstPtr msg) {
  Vector6d wrench_ft_frame;
  Matrix6d rotation_ft_base;
  if (ft_arm_ready_) {

    // Reading the FT-sensor in its own frame (robotiq_force_torque_frame_id)
    wrench_ft_frame << msg->wrench.force.x, msg->wrench.force.y,
                    msg->wrench.force.z, msg->wrench.torque.x,
                    msg->wrench.torque.y, msg->wrench.torque.z;

    // Get transform from arm base link to platform base link
    get_rotation_matrix(rotation_ft_base, listener_ft_,
                        "ur5_arm_base_link", "robotiq_force_torque_frame_id");

    wrench_external_ <<  rotation_ft_base * wrench_ft_frame;
  }
}

void AdmittanceController::wrench_control_callback(
  const geometry_msgs::WrenchStampedConstPtr msg) {

  if (msg->header.frame_id == "ur5_arm_base_link") {
    wrench_control_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                    msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  }
  else  {
    ROS_WARN_THROTTLE(5, "wrench_control_callback: The frame_id is not specified as ur5_arm_base_link");
  }
}

void AdmittanceController::ds_velocity_callback(const geometry_msgs::TwistStampedPtr msg) {

  arm_desired_twist_ds_ << msg->twist.linear.x , msg->twist.linear.y , msg->twist.linear.z ;
  // ROS_INFO_STREAM_THROTTLE(1,"received velocity, z:" << arm_desired_twist_ds_(2));

}

void AdmittanceController::admittance_ratio_callback(const std_msgs::Float32Ptr msg) {

  double h = msg->data;

  if (h > 1) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance ration higher than one is recieved " <<  h);
    h = 1;
  }
  else if (h < 0 ) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance ratio lower than zero is recieved " << h);
    h = 0;
  }
  else {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance ratio between 0 and 1 recieved " << h);
  }

  admittance_ratio_ = h;

}

///////////////////////////////////////////////////////////////
//////////////////// COMMANDING THE ROBOT /////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::send_commands_to_robot() {

  // for the arm
  geometry_msgs::Twist arm_twist_cmd;

  arm_twist_cmd.linear.x  = arm_desired_twist_final_(0);
  arm_twist_cmd.linear.y  = arm_desired_twist_final_(1);
  arm_twist_cmd.linear.z  = arm_desired_twist_final_(2);
  arm_twist_cmd.angular.x = arm_desired_twist_final_(3);
  arm_twist_cmd.angular.y = arm_desired_twist_final_(4);
  arm_twist_cmd.angular.z = arm_desired_twist_final_(5);

  // ROS_WARN_STREAM_THROTTLE(1,"sending z vel: " << arm_twist_cmd.linear.z);

  pub_arm_cmd_.publish(arm_twist_cmd);
}


void AdmittanceController::limit_to_workspace() {

  if (arm_real_position_(0) < workspace_limits_(0) || arm_real_position_(0) > workspace_limits_(1)) {
    ROS_WARN_STREAM_THROTTLE (1, "Out of permitted workspace.  x = "
                              << arm_real_position_(0) << " not in [" << workspace_limits_(0) << " , "
                              << workspace_limits_(1) << "]");
  }

  if (arm_real_position_(1) < workspace_limits_(2) || arm_real_position_(1) > workspace_limits_(3)) {
    ROS_WARN_STREAM_THROTTLE (1, "Out of permitted workspace.  y = "
                              << arm_real_position_(1) << " not in [" << workspace_limits_(2) << " , "
                              << workspace_limits_(3) << "]");
  }

  if (arm_real_position_(2) < workspace_limits_(4) || arm_real_position_(2) > workspace_limits_(5)) {
    ROS_WARN_STREAM_THROTTLE (1, "Out of permitted workspace.  z = "
                              << arm_real_position_(2) << " not in [" << workspace_limits_(4) << " , "
                              << workspace_limits_(5) << "]");
  }


  arm_desired_twist_final_ = arm_desired_twist_adm_;
  /* Why? */
  // arm_desired_twist_final_.segment(0,3) += (1- admittance_ratio_) * arm_desired_twist_ds_;
  arm_desired_twist_final_.segment(0,3) += arm_desired_twist_ds_;

  if (arm_desired_twist_final_(0) < 0 && arm_real_position_(0) < workspace_limits_(0)) {
    arm_desired_twist_final_(0) = 0;
  }

  if (arm_desired_twist_final_(0) > 0 && arm_real_position_(0) > workspace_limits_(1)) {
    arm_desired_twist_final_(0) = 0;
  }

  if (arm_desired_twist_final_(1) < 0 && arm_real_position_(1) < workspace_limits_(2)) {
    arm_desired_twist_final_(1) = 0;
  }

  if (arm_desired_twist_final_(1) > 0 && arm_real_position_(1) > workspace_limits_(3)) {
    arm_desired_twist_final_(1) = 0;
  }

  if (arm_desired_twist_final_(2) < 0 && arm_real_position_(2) < workspace_limits_(4)) {
    arm_desired_twist_final_(2) = 0;
  }

  if (arm_desired_twist_final_(2) > 0 && arm_real_position_(2) > workspace_limits_(5)) {
    arm_desired_twist_final_(2) = 0;
  }

  // velocity of the arm along x, y, and z axis
  double norm_vel_des = (arm_desired_twist_final_.segment(0, 3)).norm();

  if (norm_vel_des > arm_max_vel_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generate fast arm movements! velocity norm: " << norm_vel_des);

    arm_desired_twist_final_.segment(0, 3) *= (arm_max_vel_ / norm_vel_des);

  }

}


//////////////////////
/// INITIALIZATION ///
//////////////////////
void AdmittanceController::wait_for_transformations() {
  tf::TransformListener listener;
  Matrix6d rot_matrix;
  rotation_base_.setZero();

  // Makes sure all TFs exists before enabling all transformations in the callbacks
  while (!get_rotation_matrix(rotation_base_, listener,
                              "base_link", "ur5_arm_base_link")) {
    sleep(1);
  }

  while (!get_rotation_matrix(rot_matrix, listener,
                              "world", "base_link")) {
    sleep(1);
  }
  base_world_ready_ = true;

  while (!get_rotation_matrix(rot_matrix, listener,
                              "world", "ur5_arm_base_link")) {
    sleep(1);
  }
  arm_world_ready_ = true;
  while (!get_rotation_matrix(rot_matrix, listener,
                              "ur5_arm_base_link", "world")) {
    sleep(1);
  }
  world_arm_ready_ = true;

  while (!get_rotation_matrix(rot_matrix, listener,
                              "ur5_arm_base_link", "FT300_link")) {
    sleep(1);
  }

  ft_arm_ready_ = true;
  ROS_INFO("The Force/Torque sensor is ready to use.");
}




////////////
/// UTIL ///
////////////

bool AdmittanceController::get_rotation_matrix(Matrix6d & rotation_matrix,
    tf::TransformListener & listener,
    std::string from_frame,
    std::string to_frame) {
  tf::StampedTransform transform;
  Matrix3d rotation_from_to;
  try {
    listener.lookupTransform(from_frame, to_frame,
                             ros::Time(0), transform);
    tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  }
  catch (tf::TransformException ex) {
    rotation_matrix.setZero();
    ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame );
    return false;
  }

  return true;
}



void AdmittanceController::publish_debuggings_signals() {

  geometry_msgs::WrenchStamped msg_wrench;

  msg_wrench.header.stamp    = ros::Time::now();
  msg_wrench.header.frame_id = "ur5_arm_base_link";
  msg_wrench.wrench.force.x  = wrench_external_(0);
  msg_wrench.wrench.force.y  = wrench_external_(1);
  msg_wrench.wrench.force.z  = wrench_external_(2);
  msg_wrench.wrench.torque.x = wrench_external_(3);
  msg_wrench.wrench.torque.y = wrench_external_(4);
  msg_wrench.wrench.torque.z = wrench_external_(5);
  pub_wrench_external_.publish(msg_wrench);

  msg_wrench.header.stamp    = ros::Time::now();
  msg_wrench.header.frame_id = "ur5_arm_base_link";
  msg_wrench.wrench.force.x  = wrench_control_(0);
  msg_wrench.wrench.force.y  = wrench_control_(1);
  msg_wrench.wrench.force.z  = wrench_control_(2);
  msg_wrench.wrench.torque.x = wrench_control_(3);
  msg_wrench.wrench.torque.y = wrench_control_(4);
  msg_wrench.wrench.torque.z = wrench_control_(5);
  pub_wrench_control_.publish(msg_wrench);

}

