#include "AdmittanceController.h"

AdmittanceController::AdmittanceController(ros::NodeHandle &n,
    double frequency,    
    std::string topic_arm_pose,
    std::string topic_arm_twist,
    std::string topic_external_wrench,
    std::string topic_control_wrench,
    std::string topic_arm_command,
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


  // --- Subscribers --- //
  sub_arm_pose_           = nh_.subscribe(topic_arm_pose, 10,
                                 &AdmittanceController::pose_arm_callback, this,
                                 ros::TransportHints().reliable().tcpNoDelay());
  std::string topic_arm_pose_arm = "/UR10arm/ee_pose_arm";
  sub_arm_pose_arm_       = nh_.subscribe(topic_arm_pose_arm, 10,
                                 &AdmittanceController::pose_arm_arm_callback, this,
                                 ros::TransportHints().reliable().tcpNoDelay());

  sub_arm_twist_          = nh_.subscribe(topic_arm_twist, 10,
                                 &AdmittanceController::twist_arm_callback, this,
                                 ros::TransportHints().reliable().tcpNoDelay());

  sub_wrench_external_    = nh_.subscribe(topic_external_wrench, 5,
                                       &AdmittanceController::wrench_external_callback, this,
                                       ros::TransportHints().reliable().tcpNoDelay());
  sub_wrench_control_     = nh_.subscribe(topic_control_wrench, 5,
                                      &AdmittanceController::wrench_control_callback, this,
                                      ros::TransportHints().reliable().tcpNoDelay());

  // --- Publishers --- //
  pub_arm_cmd_            = nh_.advertise<geometry_msgs::Twist>(topic_arm_command, 5);

  // pub_wrench_external_    = nh_.advertise<geometry_msgs::WrenchStamped>(
  //                            topic_external_wrench_arm_frame, 5);
  // pub_wrench_control_     = nh_.advertise<geometry_msgs::WrenchStamped>(
  //                            topic_control_wrench_arm_frame, 5);

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
  arm_desired_twist_.setZero();
  ft_arm_ready_ = false;
  arm_world_ready_ = false;
  world_arm_ready_ = false;

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
    limit_to_workspace();
    // Here I can do the "workspace-modulation" idea

    // Copy commands to messages
    send_commands_to_robot();

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

  arm_desired_accelaration = M_a_.inverse() * ( - D_a_ * arm_desired_twist_
                             + wrench_external_ + wrench_control_);

  // limiting the accelaration for better stability and safety
  double a_acc_norm = (arm_desired_accelaration.segment(0, 3)).norm();
  if (a_acc_norm > arm_max_acc_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high arm accelaration!"
                             << " norm: " << a_acc_norm);
    arm_desired_accelaration.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
  }

  // Integrate for velocity based interface
  ros::Duration duration = loop_rate_.expectedCycleTime();
  arm_desired_twist_  += arm_desired_accelaration * duration.toSec();

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

void AdmittanceController::pose_arm_arm_callback(
  const geometry_msgs::PoseConstPtr msg) {
  arm_real_position_arm_ << msg->position.x, msg->position.y,
                     msg->position.z;

  arm_real_orientation_arm_.coeffs() << msg->orientation.x,
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

void AdmittanceController::wrench_external_callback(
  const geometry_msgs::WrenchStampedConstPtr msg) {
  Vector6d wrench_ft_frame;
  if (ft_arm_ready_) {

    // Reading the FT-sensor in its own frame (robotiq_force_torque_frame_id)
    wrench_ft_frame << msg->wrench.force.x, msg->wrench.force.y,
                    msg->wrench.force.z, msg->wrench.torque.x,
                    msg->wrench.torque.y, msg->wrench.torque.z;


    // --- This can be a callback! --- //                
    get_rotation_matrix(rotation_tool_, listener_ft_, "base_link", "robotiq_force_torque_frame_id", 0 );    
    wrench_external_ <<  rotation_tool_  * wrench_ft_frame;
  }
}

void AdmittanceController::wrench_control_callback(
  const geometry_msgs::WrenchStampedConstPtr msg) {

  if (msg->header.frame_id == "arm_base_link") {
    wrench_control_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                    msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  }
  else  {
    ROS_WARN_THROTTLE(5, "wrench_control_callback: The frame_id is not specified as ur5_arm_base_link");
  }
}

///////////////////////////////////////////////////////////////
//////////////////// COMMANDING THE ROBOT /////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::send_commands_to_robot() {

  // for the arm
  geometry_msgs::Twist arm_twist_cmd;

  arm_twist_cmd.linear.x  = arm_desired_twist_(0);
  arm_twist_cmd.linear.y  = arm_desired_twist_(1);
  arm_twist_cmd.linear.z  = arm_desired_twist_(2);
  arm_twist_cmd.angular.x = arm_desired_twist_(3);
  arm_twist_cmd.angular.y = arm_desired_twist_(4);
  arm_twist_cmd.angular.z = arm_desired_twist_(5);

  ROS_WARN_STREAM_THROTTLE(0.1,"Desired linear velocity: "  << arm_twist_cmd.linear.x << " " << arm_twist_cmd.linear.y << " " <<  arm_twist_cmd.linear.z);
  ROS_WARN_STREAM_THROTTLE(0.1,"Desired angular velocity: " << arm_twist_cmd.angular.x << " " << arm_twist_cmd.angular.y << " " <<  arm_twist_cmd.angular.z);

  pub_arm_cmd_.publish(arm_twist_cmd);
}


void AdmittanceController::limit_to_workspace() {

  // velocity of the arm along x, y, and z axis
  double norm_vel_des = (arm_desired_twist_.segment(0, 3)).norm();

  if (norm_vel_des > arm_max_vel_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generate fast arm movements! velocity norm: " << norm_vel_des);

    arm_desired_twist_.segment(0, 3) *= (arm_max_vel_ / norm_vel_des);

  }

  if (norm_vel_des < 1e-5)
    arm_desired_twist_.segment(0,3).setZero();

  if (arm_desired_twist_(3) < 1e-5)
      arm_desired_twist_(3) = 0;
  if (arm_desired_twist_(4)< 1e-5)
      arm_desired_twist_(4) = 0;
  if (arm_desired_twist_(5) < 1e-5)
      arm_desired_twist_(5) = 0;    

  // velocity of the arm along x, y, and z angles
  if (arm_desired_twist_(3) > 0.3)
      arm_desired_twist_(3) = 0.3;
  if (arm_desired_twist_(4) > 0.3)
      arm_desired_twist_(4) = 0.3;
  if (arm_desired_twist_(5) > 0.3)
      arm_desired_twist_(5) = 0.3;    


  // Impose workspace constraints on desired velocities
  double ee_base_norm   = (arm_real_position_arm_).norm();
  double rec_operating_limit = 1.15; // simulated robot
  // double rec_operating_limit = 1; // real robot
  double dist_limit = rec_operating_limit - ee_base_norm; 
  ROS_WARN_STREAM_THROTTLE(0.1, "||x_ee-w_limit||: " << dist_limit) ;
  if (ee_base_norm >= rec_operating_limit){
    ROS_WARN_STREAM_THROTTLE(0.1, "Out of operational workspace limit!") ;
    base_position_ << 0.33000, 0.00000, 0.48600;
    Vector3d repulsive_field = -(arm_real_position_- base_position_);
    arm_desired_twist_.segment(0,3) = 0.05*(repulsive_field/ee_base_norm);
    ROS_WARN_STREAM_THROTTLE(0.1, "Bringing robot back slowly with uniform repulsive velocity field!") ;
  }

  if (arm_real_position_(2) < workspace_limits_(4))
      arm_desired_twist_(2) += 0.3;

  
  // Continuous modulation of velocity to follow the surface (try walid/sina's modulation)
  double workspace_fct = dist_limit;
  if (dist_limit > 0.05)
      workspace_fct = 1;
  
  // workspace_fct  *= norm_vel_des;  
  // repulsive_field = workspace_fct * (repulsive_field/ee_base_norm);
  // ROS_WARN_STREAM_THROTTLE(0.1,"Repulsive velocity field: "  << repulsive_field(0) << " " << repulsive_field(1) << " " <<  repulsive_field(2));  
  ROS_WARN_STREAM_THROTTLE(0.1,"Workspace Scaling function: "  << workspace_fct);  
  // arm_desired_twist_.segment(0,3) = workspace_fct*arm_desired_twist_.segment(0,3);

}


//////////////////////
/// INITIALIZATION ///
//////////////////////
void AdmittanceController::wait_for_transformations() {
  tf::TransformListener listener;
  Matrix6d rot_matrix;
  rotation_base_.setZero();
  rotation_tool_.setZero();

  // Makes sure all TFs exists before enabling all transformations in the callbacks
  while (!get_rotation_matrix(rotation_base_, listener,
                              "base_link", "arm_base_link", 1)) {
    sleep(1);
  }
  arm_world_ready_ = true;
  
  while (!get_rotation_matrix(rot_matrix, listener,
                              "arm_base_link", "base_link", 0)) {
    sleep(1);
  }
  world_arm_ready_ = true;

  while (!get_rotation_matrix(rotation_tool_, listener,
                              "base_link", "robotiq_force_torque_frame_id", 0)) {
    sleep(1);
  }
  ft_arm_ready_ = true;
  ROS_INFO("The Force/Torque sensor is ready to use.");
}




////////////
/// UTIL ///
////////////

// Add a "get transformation"
bool AdmittanceController::get_rotation_matrix(Matrix6d & rotation_matrix,
    tf::TransformListener & listener,
    std::string from_frame,
    std::string to_frame, bool getT) {
  tf::StampedTransform transform;
  Matrix3d rotation_from_to;
  try {
    listener.lookupTransform(from_frame, to_frame,
                             ros::Time(0), transform);
    tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);    
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;

    if (getT==1)
      base_position_ << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z() ; 
  }
  catch (tf::TransformException ex) {
    rotation_matrix.setZero();
    ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame );
    return false;
  }

  return true;
}
