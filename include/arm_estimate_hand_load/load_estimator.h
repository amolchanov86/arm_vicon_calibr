/*
 * estimate_hand_load.h
 *
 *  Created on: July 21, 2011
 *      Author: righetti
 */

#ifndef LOAD_ESTIMATOR_H
#define LOAD_ESTIMATOR_H

// system includes
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>


#include <sl_controller_msgs/EndEffectorState.h>
#include <arm_msgs/InertialParameters.h>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <visualization_msgs/Marker.h>

#include <sl_controller_interface/robot_monitor.h>

#include <Eigen/Eigen>



namespace arm_estimate_hand_load{

class LoadEstimator {


public:
  LoadEstimator(ros::NodeHandle& node_handle);
  virtual ~LoadEstimator();


private:

  static const double GRAVITY_CONSTANT_;

  bool reset();
  bool calibrate();
  bool resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool calibSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool readParams();
  void endEffectorStateCallback(const sl_controller_msgs::EndEffectorState& msg);

  bool updateRegression();
  bool computeInertialParameters();

  void publish_inertial_data();
  void publishMarkers();
  void initCogMarker( const float col_r_in, const float col_g_in, const float col_b_in, const std::string& ns_in, const int id_in, visualization_msgs::Marker& cog_marker_out );
  void setMarker(float x_in, float y_in, float z_in,  visualization_msgs::Marker& cog_marker_out);

  void getCrossMatrix(const Eigen::Vector3d& input_vector, Eigen::Matrix3d& crossed_matrix);
  void getDotMatrix(const Eigen::Vector3d& input_vector, Eigen::MatrixXd& dotted_matrix);


  boost::thread publisher_thread_;
  boost::thread calibrate_thread_;
  boost::mutex mutex_thread_;

  ros::NodeHandle node_handle_;

  //SL state of the robot
  ros::Subscriber wrench_sub_;
  sl_controller_msgs::EndEffectorState end_effector_state_msg_;
  std::string end_effector_state_topic_;

  //Services & clients
  ros::ServiceServer reset_srv;
  ros::ServiceServer calib_srv;

  //publishing result
  ros::Publisher inertial_estimation_pub_;
  ros::Publisher cog_marker_pub_;
  arm_msgs::InertialParameters inertial_parameters_msg_;
  visualization_msgs::Marker cog_marker_; //Marker for a center of mass of object + hand
  visualization_msgs::Marker cog_obj_marker_; //Marker for a center of mass of the object only
  visualization_msgs::Marker cog_hand_marker_; //Marker for a center of mass of the object only

  //Other stuff
  bool sim; //Simulator or a real robot

  //estimation parameters
  int num_estimated_params_;
  int num_offset_params_;
  int num_input_data_;
  bool use_recursive_least_square_;
  bool estimate_inertia_;
  bool estimate_offset_;
  bool static_assumption_;
  float cog_marker_size_;
  int estimate_update_count;
  int calib_min_updates_;
  int calib_wait_period_;
  double hand_mass;
  Eigen::Vector3d hand_cog;

  //for least squares
  Eigen::MatrixXd regressATA_;
  Eigen::VectorXd regressATb_;

  //for recursive least squares
  Eigen::MatrixXd P_, K_;
  Eigen::VectorXd regressed_parameters_;
  double lambda_;
  double mass_naive;
  double obj_mass_min;
  
  //tf stuff
  tf::TransformListener tf_listener_;
  std::string tf_wrench_name_;
  std::string tf_base_name_;

  bool going_;
  double pub_time_;
};

}

#endif /* LOAD_ESTIMATOR_H */
