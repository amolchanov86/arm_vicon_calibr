/*
 * estimate_hand_load.cpp
 *
 *  Created on: July 21, 2011
 *      Author: righetti, artem
 */

#include <arm_estimate_hand_load/load_estimator.h>
#include <cmath>


namespace arm_estimate_hand_load{
	
const double LoadEstimator::GRAVITY_CONSTANT_ = 9.81;


LoadEstimator::LoadEstimator(ros::NodeHandle& node_handle) : node_handle_(node_handle)
  , estimate_update_count(0)
  , calib_wait_period_(0.1)
  , hand_mass(0)
  , hand_cog(0, 0, 0)
  , obj_mass_min(0.01)
  , sim(false)
{
  //first read the parameters
  ROS_VERIFY(readParams());
  going_ = true;
  num_input_data_ = 6;
  reset();

  inertial_estimation_pub_ = node_handle_.advertise<arm_msgs::InertialParameters>("load_in_hand", 100);

  //Initialisation of markers representing centers of masses of hand+obj, hand only, object only
  initCogMarker(1, 0, 0, "cog_hand_obj", 0, cog_marker_);
  initCogMarker(0, 1, 0, "cog_hand", 1, cog_hand_marker_);
  initCogMarker(0, 0, 1, "cog_obj" , 2, cog_obj_marker_);

  cog_marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("cog_marker", 10);
  reset_srv = node_handle_.advertiseService("reset", &LoadEstimator::resetSrv, this);

  //The service reestimates empty hand mass and cog. Must be called when the hand is empty and static
  calib_srv = node_handle_.advertiseService("calibrate", &LoadEstimator::calibSrv, this);

  publisher_thread_ = boost::thread(boost::bind(&LoadEstimator::publish_inertial_data, this));
  wrench_sub_ = node_handle_.subscribe(end_effector_state_topic_, 1, &LoadEstimator::endEffectorStateCallback, this);

  //Calling calibration thread (the thread executes procedure only once until calibration service is called)
  calibrate_thread_ = boost::thread(boost::bind(&LoadEstimator::calibrate, this));
}

LoadEstimator::~LoadEstimator() {
  // TODO Auto-generated destructor stub
}


bool LoadEstimator::reset()
{
  ROS_WARN("WARNING: Estimation reset !!!");

  regressATA_.setZero(num_estimated_params_, num_estimated_params_);
  regressATb_.setZero(num_estimated_params_);

  P_.setIdentity(num_estimated_params_, num_estimated_params_);
  P_ *= 100.0;
  K_.setZero(num_estimated_params_, num_input_data_);
  regressed_parameters_.setZero(num_estimated_params_);
  return true;
}

bool LoadEstimator::calibrate()
{
    ROS_WARN("WARNING: Calibration started !!!");
    //Reset to get rid of previous estimates
    reset();

    //Wait till new data are accumulate (till we have enough estimates)
    estimate_update_count = 0;
    while( estimate_update_count <= calib_min_updates_ )
    {
      ros::Duration(calib_wait_period_).sleep();
    }

    //Save new estimates
    hand_mass   = inertial_parameters_msg_.mass;
    hand_cog[0] = inertial_parameters_msg_.cog.x;
    hand_cog[1] = inertial_parameters_msg_.cog.y;
    hand_cog[2] = inertial_parameters_msg_.cog.z;

    ROS_WARN("WARNING: Calibration of empty hand mass and COG done !!!");

  return true;
}

bool LoadEstimator::resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    return reset();
}

bool LoadEstimator::calibSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    return calibrate();
}

void LoadEstimator::publish_inertial_data()
{
  while(going_)
  {
    mutex_thread_.lock();

    double norm_of_P = P_.norm();
    std::cout << "norm of P_ \t" << norm_of_P << std::endl << std::endl;

    computeInertialParameters();

    mutex_thread_.unlock();
    //we don;t need the lock here
    inertial_estimation_pub_.publish(inertial_parameters_msg_);

    //Publish markers
    publishMarkers();

    ros::Duration(pub_time_).sleep();
  }
}

void LoadEstimator::initCogMarker( const float color_r_in, const float color_g_in, const float color_b_in, const std::string& ns_in, const int id_in, visualization_msgs::Marker& cog_marker_out )
{
    cog_marker_out.header.frame_id = tf_wrench_name_;
    cog_marker_out.header.stamp = ros::Time::now();
    cog_marker_out.ns = ns_in;
    cog_marker_out.id = id_in;

    cog_marker_out.type = visualization_msgs::Marker::SPHERE;
    cog_marker_out.action = visualization_msgs::Marker::ADD;

    cog_marker_out.pose.position.x = 0;
    cog_marker_out.pose.position.y = 0;
    cog_marker_out.pose.position.z = 0;

    cog_marker_out.scale.x = cog_marker_size_;
    cog_marker_out.scale.y = cog_marker_size_;
    cog_marker_out.scale.z = cog_marker_size_;

    cog_marker_out.color.r = color_r_in;
    cog_marker_out.color.g = color_g_in;
    cog_marker_out.color.b = color_b_in;
    cog_marker_out.color.a = 1;

    cog_marker_out.lifetime = ros::Duration();
}

void LoadEstimator::setMarker(float x_in, float y_in, float z_in,  visualization_msgs::Marker& cog_marker_out)
{
   cog_marker_out.header.stamp = ros::Time::now();
   cog_marker_out.pose.position.x = x_in;
   cog_marker_out.pose.position.y = y_in;
   cog_marker_out.pose.position.z = z_in;
}

void LoadEstimator::publishMarkers()
{
    setMarker(inertial_parameters_msg_.cog.x, inertial_parameters_msg_.cog.y, inertial_parameters_msg_.cog.z, cog_marker_);
    cog_marker_pub_.publish(cog_marker_);

    setMarker(inertial_parameters_msg_.hand_cog.x, inertial_parameters_msg_.hand_cog.y, inertial_parameters_msg_.hand_cog.z, cog_hand_marker_);
    cog_marker_pub_.publish(cog_hand_marker_);

    setMarker(inertial_parameters_msg_.obj_cog.x, inertial_parameters_msg_.obj_cog.y, inertial_parameters_msg_.obj_cog.z, cog_obj_marker_);
    cog_marker_pub_.publish(cog_obj_marker_);
}

bool LoadEstimator::updateRegression()
{
//    std::cout << "updateRegression()" << std::endl;
	
  Eigen::MatrixXd regress_matrix(num_input_data_, num_estimated_params_);
  Eigen::VectorXd regress_vector(num_input_data_);

  regress_matrix.setZero(num_input_data_, num_estimated_params_);
  regress_vector.setZero(num_input_data_);

  tf::StampedTransform wrench_transform;

  tf::Vector3 wrench_torque;
  tf::vector3MsgToTF(end_effector_state_msg_.wrench.torque, wrench_torque);

  regress_vector(0) = - end_effector_state_msg_.wrench.force.x;
  regress_vector(1) = - end_effector_state_msg_.wrench.force.y;
  regress_vector(2) = - end_effector_state_msg_.wrench.force.z;

  if(sim)
  {
    regress_vector(3) =  end_effector_state_msg_.wrench.torque.x;
    regress_vector(4) =  end_effector_state_msg_.wrench.torque.y;
    regress_vector(5) =  end_effector_state_msg_.wrench.torque.z;
  }
  else
  {
      regress_vector(3) =  - end_effector_state_msg_.wrench.torque.x;
      regress_vector(4) =  - end_effector_state_msg_.wrench.torque.y;
      regress_vector(5) =  - end_effector_state_msg_.wrench.torque.z;
  }

  mass_naive = regress_vector.block(0,0,3,1).norm() / GRAVITY_CONSTANT_;
  
  //get the transform for the FT 


//Options:
//    ros::Time(0); //Just the latest message
//    end_effector_state_msg_.header.stamp; //Time from a header
//    ros::Time::now(); //Current time from a clock
  bool success = false;
  while (!success)
  {
    try
    {
      tf_listener_.waitForTransform(tf_base_name_, tf_wrench_name_, ros::Time(0), ros::Duration(0.5));
      tf_listener_.lookupTransform(tf_base_name_, tf_wrench_name_, ros::Time(0), wrench_transform);
      success = true;
    }
    catch (tf::ExtrapolationException ex)
    {
      ROS_ERROR("%s",ex.what());
    }
    sleep(0.1);
  }
  
  
  wrench_transform.setOrigin(tf::Vector3(0,0,0));

  //get an eigen transform
  //we need to do everything in FT local frame to get the CoM invariant
  // (will not be the case in world frame since the CoM vector will be moving)
  
  Eigen::Affine3d world_to_wrench_transform;
  tf::transformTFToEigen(wrench_transform.inverse(), world_to_wrench_transform); //previously it was TransformTFToEigen


  //create the total vector from acceleration and gravity
  Eigen::Vector3d acceleration_vector;
  acceleration_vector(0) = end_effector_state_msg_.linear_acceleration.x;
  acceleration_vector(1) = end_effector_state_msg_.linear_acceleration.y;
  acceleration_vector(2) = end_effector_state_msg_.linear_acceleration.z;

  //get the angular acc and vel in FT frame
  Eigen::Vector3d angular_velocity = Eigen::Vector3d(end_effector_state_msg_.angular_velocity.x,
                                                     end_effector_state_msg_.angular_velocity.y,
                                                     end_effector_state_msg_.angular_velocity.z);

  Eigen::Vector3d angular_acceleration = Eigen::Vector3d(end_effector_state_msg_.angular_acceleration.x,
                                                         end_effector_state_msg_.angular_acceleration.y,
                                                         end_effector_state_msg_.angular_acceleration.z);  

  //If we assume static case - let's get rid of noises introduced by angular vel and accelerations (just test option)
  if(static_assumption_)
  {
//    acceleration_vector.setZero();
    angular_velocity.setZero();
    angular_acceleration.setZero();
  }
  //create the gravity vector
  Eigen::Vector3d gravity_vector = Eigen::Vector3d::Zero();
  gravity_vector(2) = -GRAVITY_CONSTANT_;

  //total contribution
  Eigen::Vector3d total_contribution = acceleration_vector - gravity_vector;

  //create the moment matrix in local frame
  Eigen::Matrix3d moment_matrix;
  getCrossMatrix(-total_contribution, moment_matrix);
  moment_matrix = world_to_wrench_transform.rotation() * moment_matrix * world_to_wrench_transform.rotation().transpose();

  //contribution of angular accelerations to the force
  Eigen::Matrix3d angular_vel_cross;
  getCrossMatrix(angular_velocity, angular_vel_cross);
  
  Eigen::Matrix3d angular_acc_cross;
  getCrossMatrix(angular_acceleration, angular_acc_cross);

  Eigen::Matrix3d angular_acc_contrib = angular_acc_cross + angular_vel_cross * angular_vel_cross;
  angular_acc_contrib = world_to_wrench_transform.rotation() * angular_acc_contrib * world_to_wrench_transform.rotation().transpose();

  //fill the regression matrix
  regress_matrix.block(0, 0, 3, 1) = world_to_wrench_transform.rotation() * total_contribution;
  regress_matrix.block(3, 1, 3, 3) = moment_matrix;
  regress_matrix.block(0, 1, 3, 3) = angular_acc_contrib;

  //fill with ones for the offset (i.e. unknown biases of sensors move to the regression matrix <=> add identity matrix)
  if(estimate_offset_)
  {
	regress_matrix.block(0, 4, 3, 3) = Eigen::Matrix3d::Identity(); 
	regress_matrix.block(3, 4 + 3, 3, 3) = Eigen::Matrix3d::Identity();	  
  }

  //add optional regession for inertial parameters Iii
  if(estimate_inertia_)
  {
    Eigen::MatrixXd angular_vel_dot = Eigen::MatrixXd(3, 6);
    getDotMatrix(world_to_wrench_transform.rotation() * angular_velocity, angular_vel_dot);

    Eigen::MatrixXd angular_acc_dot = Eigen::MatrixXd(3, 6);
    getDotMatrix(world_to_wrench_transform.rotation() * angular_acceleration, angular_acc_dot);

    Eigen::MatrixXd inertial_contrib = angular_acc_dot + world_to_wrench_transform.rotation() * angular_vel_cross * world_to_wrench_transform.rotation().transpose() * angular_vel_dot;

    regress_matrix.block(3, 4 + num_offset_params_, 3, 6) = inertial_contrib;
// 	regress_matrix.block(3, 4, 3, 6) = inertial_contrib;
// 	std::cout << "Inertial_contrib = " << std::endl << inertial_contrib << std::endl << std::endl;
  }

//       std::cout << "the regress matrix looks like" << std::endl << std::endl << regress_matrix << std::endl;
//       std::cout << "Regressed vector looks like: " << regress_vector << std::endl;

  if(use_recursive_least_square_)
  {
    K_ = P_ * regress_matrix.transpose() * (lambda_ * Eigen::MatrixXd::Identity(num_input_data_, num_input_data_) + regress_matrix * P_ * regress_matrix.transpose()).inverse();

    P_ = (1/lambda_) * (Eigen::MatrixXd::Identity(num_estimated_params_, num_estimated_params_) - K_ * regress_matrix) * P_;

    regressed_parameters_ = regressed_parameters_ + K_ * (regress_vector - regress_matrix * regressed_parameters_);
//  std::cout << regressed_parameters_<< std::endl << std::endl;
//  std::cout << "K matrix = " << K_<< std::endl << std::endl;
//  std::cout << "P_ matrix = " << P_ << std::endl << std::endl;
  }
  else
  {
    regressATA_ += regress_matrix.transpose() * regress_matrix;
    regressATb_ += regress_matrix.transpose() * regress_vector;
	
// 	std::cout << "A.transpose()*A:" << std::endl << regressATA_ << std::endl << std::endl << std::endl;
// 	std::cout << "A.transpose()*b:" << std::endl << regressATb_ << std::endl << std::endl << std::endl;
  }

  return true;
}

void LoadEstimator::getCrossMatrix(const Eigen::Vector3d& input_vector, Eigen::Matrix3d& crossed_matrix)
{
  crossed_matrix.setZero(3,3);

  crossed_matrix(0,1) = - input_vector(2);
  crossed_matrix(1,0) = input_vector(2);
  crossed_matrix(0,2) = input_vector(1);
  crossed_matrix(2,0) = - input_vector(1);
  crossed_matrix(1,2) = - input_vector(0);
  crossed_matrix(2,1) = input_vector(0);
}

void LoadEstimator::getDotMatrix(const Eigen::Vector3d& input_vector, Eigen::MatrixXd& dotted_matrix)
{
  dotted_matrix.setZero(3, 6);

  dotted_matrix(0,0) = input_vector(0);
  dotted_matrix(0,1) = input_vector(1);
  dotted_matrix(0,2) = input_vector(2);

  dotted_matrix(1,1) = input_vector(0);
  dotted_matrix(1,3) = input_vector(1);
  dotted_matrix(1,4) = input_vector(2);

  dotted_matrix(2,2) = input_vector(0);
  dotted_matrix(2,4) = input_vector(1);
  dotted_matrix(2,5) = input_vector(2);
}

bool LoadEstimator::computeInertialParameters()
{
double error = 0;	
  if(!use_recursive_least_square_)
  {
//  regressed_parameters_ = regressATA_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(regressATb_); 
//  regressed_parameters_ = regressATA_.colPivHouseholderQr().solve(regressATb_);
    regressed_parameters_ = regressATA_.fullPivHouseholderQr().solve(regressATb_);

    error = (regressATA_*regressed_parameters_ - regressATb_).norm() / regressATb_.norm();
    std::cout << "Estimation error = " << error << std::endl;
  }

  //Inertial parameters for obj + hand
  inertial_parameters_msg_.mass = regressed_parameters_(0);
  inertial_parameters_msg_.cog.x = regressed_parameters_(1)/regressed_parameters_(0);
  inertial_parameters_msg_.cog.y = regressed_parameters_(2)/regressed_parameters_(0);
  inertial_parameters_msg_.cog.z = regressed_parameters_(3)/regressed_parameters_(0);

  //Inertial parameters for hand only
  inertial_parameters_msg_.hand_mass = hand_mass;
  inertial_parameters_msg_.hand_cog.x = hand_cog[0];
  inertial_parameters_msg_.hand_cog.y = hand_cog[1];
  inertial_parameters_msg_.hand_cog.z = hand_cog[2];

  //Inertial parameters for obj only
  inertial_parameters_msg_.obj_mass = inertial_parameters_msg_.mass - inertial_parameters_msg_.hand_mass;

//  ROS_INFO("Obj mass = %f Min mass = %f", inertial_parameters_msg_.obj_mass, obj_mass_min );
  if( fabs( inertial_parameters_msg_.obj_mass ) >= obj_mass_min)
  {
    inertial_parameters_msg_.obj_cog.x
            = (inertial_parameters_msg_.mass * inertial_parameters_msg_.cog.x -  inertial_parameters_msg_.hand_mass * hand_cog[0]) / inertial_parameters_msg_.obj_mass;
    inertial_parameters_msg_.obj_cog.y
            = (inertial_parameters_msg_.mass * inertial_parameters_msg_.cog.y -  inertial_parameters_msg_.hand_mass * hand_cog[1]) / inertial_parameters_msg_.obj_mass;
    inertial_parameters_msg_.obj_cog.z
            = (inertial_parameters_msg_.mass * inertial_parameters_msg_.cog.z -  inertial_parameters_msg_.hand_mass * hand_cog[2]) / inertial_parameters_msg_.obj_mass;
  }
  else
  {
    inertial_parameters_msg_.obj_cog.x = 0;
    inertial_parameters_msg_.obj_cog.y = 0;
    inertial_parameters_msg_.obj_cog.z = 0;
  }

  if(estimate_inertia_)
  {
    inertial_parameters_msg_.inertia_tensor[0] = regressed_parameters_(4 + num_offset_params_); //prev: 10
    inertial_parameters_msg_.inertia_tensor[1] = regressed_parameters_(5 + num_offset_params_); //prev: 11
    inertial_parameters_msg_.inertia_tensor[2] = regressed_parameters_(6 + num_offset_params_); //prev: 12
    inertial_parameters_msg_.inertia_tensor[3] = regressed_parameters_(7 + num_offset_params_); //prev: 13
    inertial_parameters_msg_.inertia_tensor[4] = regressed_parameters_(8 + num_offset_params_); //prev: 14
    inertial_parameters_msg_.inertia_tensor[5] = regressed_parameters_(9 + num_offset_params_); //prev: 15 
  }
  std::cout << "Mass = " << inertial_parameters_msg_.mass << " Mass_naive = " << mass_naive;
  printf(" Fx=%f Fy=%f Fz=%f\n", -end_effector_state_msg_.wrench.force.x, -end_effector_state_msg_.wrench.force.y, -end_effector_state_msg_.wrench.force.z);
//   std::cout << "Regressed parameters = " << regressed_parameters_ << std::endl << std::endl;
    
  std::cout<< "updates = "<<estimate_update_count<<std::endl;

  estimate_update_count++;

  return true;
}

void LoadEstimator::endEffectorStateCallback(const sl_controller_msgs::EndEffectorState& msg)
{
//    std::cout << "endEffectorStateCallback()" << std::endl;
  boost::mutex::scoped_lock lock(mutex_thread_);
  end_effector_state_msg_ = msg;
  updateRegression();
}

bool LoadEstimator::readParams()
{
  ROS_VERIFY(usc_utilities::read(node_handle_, "end_effector_state_topic", end_effector_state_topic_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "tf_wrench_name", tf_wrench_name_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "tf_base_name", tf_base_name_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "pub_time", pub_time_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "lambda", lambda_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "use_recursive_least_square", use_recursive_least_square_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "estimate_inertia", estimate_inertia_)); 
  ROS_VERIFY(usc_utilities::read(node_handle_, "estimate_offset", estimate_offset_)); //Add offsets for FT(force-torque) sensors to the regression parameters
  ROS_VERIFY(usc_utilities::read(node_handle_, "static_assumption", static_assumption_)); // Forces angular velocity/accelerations readings to be 0
  ROS_VERIFY(usc_utilities::read(node_handle_, "cog_marker_size", cog_marker_size_)); // Size of the markers representing the center of mass estimates
  ROS_VERIFY(usc_utilities::read(node_handle_, "calib_min_updates", calib_min_updates_)); //Min number of updates required to estimate mass (parameter is used for calibration only)
  ROS_VERIFY(usc_utilities::read(node_handle_, "obj_mass_min", obj_mass_min)); //Min mass estimate required to re-estimate object cog, otherwise cog is set to [0,0,0]
  ROS_VERIFY(usc_utilities::read(node_handle_, "sim", sim)); //Simulator or a real robot

  //4 params (mass and mass*cog) + 6 for the FT offset (previously the ARM FT sensors had large biases = offsets);
  //we assume that the inertia I is = 0 (i.e. negligible)
  //num_estimated_params_ = 4 + 6; //previously
  
  num_offset_params_ = ( estimate_offset_ ?  6 : 0);
  num_estimated_params_ = 4 + num_offset_params_;
  if(estimate_inertia_)
  {
          num_estimated_params_ += 6;
  }

  if(static_assumption_) estimate_inertia_ = false;
	
//   end_effector_state_topic_ = "/SL/r_hand_state";
//   std::cout << "End effector state topic: " << end_effector_state_topic_ << std::endl;

  return true;
}

}
