#ifndef _NMHE_FXFYFZLEARNING_MAIN_H
#define _NMHE_FXFYFZLEARNING_MAIN_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>

#include <nmhe_fxfyfzlearning.h>

// Subscribers
ros::Subscriber state_sub;
ros::Subscriber estimation_on_sub;
ros::Subscriber vel_sub;
ros::Subscriber nmpc_cmd_rpy_sub;
ros::Subscriber nmpc_cmd_Fz_sub;

// Publishers
ros::Publisher nmhe_predInit_pub;
ros::Publisher nmhe_vel_pub;
ros::Publisher nmhe_dist_Fx_pub;
ros::Publisher nmhe_dist_Fy_pub;
ros::Publisher nmhe_dist_Fz_pub;
ros::Publisher nmhe_exeTime_pub;
ros::Publisher nmhe_kkt_pub;
ros::Publisher nmhe_obj_pub;

nmhe_struct_ nmhe_struct;
std::string mocap_topic_part;

mavros_msgs::State current_state_msg;
std_msgs::Bool estimation_on_msg;

bool estimator_stop;

double m_in, g_in;

int print_flag_offboard = 1, print_flag_arm = 1, 
print_flag_altctl = 1, print_flag_estimation_on = 0;

double t, t_est_loop;

Eigen::VectorXd current_vel_rate;
Eigen::Vector3d nmpc_cmd_ryp;
Eigen::Vector2d nmpc_cmd_Fz;
 
#endif

