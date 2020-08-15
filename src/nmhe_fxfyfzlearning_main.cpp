/**
 * @file   nmhe_fxfyfzlearning_main.cpp
 * @author Mohit Mehndiratta
 * @date   September 2019
 *
 * @copyright
 * Copyright (C) 2019.
 */

#include <nmhe_fxfyfzlearning_main.h>

using namespace Eigen;
using namespace ros;

double sampleTime = 0.03;
const double NMPC_N = 30;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_msg = *msg;
}
void estimation_on_cb(const std_msgs::Bool::ConstPtr& msg)
{
    estimation_on_msg = *msg;
}
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_vel_rate << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
                        msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
}
void nmpc_cmd_rpy_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    int i = 0;
    for(std::vector<double>::const_iterator itr = msg->data.begin(); itr != msg->data.end(); ++itr)
    {
        nmpc_cmd_ryp(i) = *itr;
        i++;
    }
}
void nmpc_cmd_Fz_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    int i = 0;
    for(std::vector<double>::const_iterator itr = msg->data.begin(); itr != msg->data.end(); ++itr)
    {
        nmpc_cmd_Fz(i) = *itr;
        i++;
    }
}

void NMHE_FXFYFZ::publish_uvw_FxFyFz(struct estimation_struct &estimationstruct)
{
    std_msgs::Bool predInit_msg;
    predInit_msg.data = is_prediction_init;
    nmhe_predInit_pub.publish(predInit_msg);

    std::vector<double> uvw_vec = {estimationstruct.u_est, estimationstruct.v_est, estimationstruct.w_est};
    std_msgs::Float64MultiArray uvw_vec_msg;
    uvw_vec_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    uvw_vec_msg.layout.dim[0].size = uvw_vec.size();
    uvw_vec_msg.layout.dim[0].stride = 1;
    uvw_vec_msg.layout.dim[0].label = "u_vel, v_vel, w_vel (m/sec)";
    uvw_vec_msg.data.clear();
    uvw_vec_msg.data.insert(uvw_vec_msg.data.end(), uvw_vec.begin(), uvw_vec.end());
    nmhe_vel_pub.publish(uvw_vec_msg);

    std::vector<double> dist_Fx_data, dist_Fy_data, dist_Fz_data;
    for (int i=0; i<NMPC_N+1; i++)
    {
        dist_Fx_data.push_back(estimationstruct.Fx_dist_est);
        dist_Fy_data.push_back(estimationstruct.Fy_dist_est);
        dist_Fz_data.push_back(estimationstruct.Fz_dist_est);
    }
    std_msgs::Float64MultiArray dist_data_msg;
    dist_data_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dist_data_msg.layout.dim[0].size = dist_Fx_data.size();
    dist_data_msg.layout.dim[0].stride = 1;
    dist_data_msg.layout.dim[0].label = "Fx_dist (N)";
    dist_data_msg.data.clear();
    dist_data_msg.data.insert(dist_data_msg.data.end(), dist_Fx_data.begin(), dist_Fx_data.end());
    nmhe_dist_Fx_pub.publish(dist_data_msg);
    dist_data_msg.layout.dim[0].label = "Fy_dist (N)";
    dist_data_msg.data.clear();
    dist_data_msg.data.insert(dist_data_msg.data.end(), dist_Fy_data.begin(), dist_Fy_data.end());
    nmhe_dist_Fy_pub.publish(dist_data_msg);
    dist_data_msg.layout.dim[0].label = "Fz_dist (N)";
    dist_data_msg.data.clear();
    dist_data_msg.data.insert(dist_data_msg.data.end(), dist_Fz_data.begin(), dist_Fz_data.end());
    nmhe_dist_Fz_pub.publish(dist_data_msg);

    std_msgs::Float64 exe_time_cmd;
    exe_time_cmd.data = estimationstruct.exe_time;
    nmhe_exeTime_pub.publish(exe_time_cmd);

    std_msgs::Float64 kkt_tol_cmd;
    kkt_tol_cmd.data = estimationstruct.kkt_tol;
    nmhe_kkt_pub.publish(kkt_tol_cmd);

    std_msgs::Float64 obj_val_msg;
    obj_val_msg.data = estimationstruct.obj_val;
    nmhe_obj_pub.publish(obj_val_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nmhe_fxfyfzlearning");
    ros::NodeHandle nh;

    ros::param::get("mocap_topic_part", mocap_topic_part);

    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);

    estimation_on_sub = nh.subscribe<std_msgs::Bool>("regression_on", 1, estimation_on_cb);
//    vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_body", 1, vel_cb);
    vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/" + mocap_topic_part + "/velocity_body", 1, vel_cb);
    nmpc_cmd_rpy_sub = nh.subscribe<std_msgs::Float64MultiArray>("outer_nmpc_cmd/rpy", 1, nmpc_cmd_rpy_cb);
    nmpc_cmd_Fz_sub = nh.subscribe<std_msgs::Float64MultiArray>("outer_nmpc_cmd/Fz_FzScaled", 1, nmpc_cmd_Fz_cb);

    // ----------
    // Publishers
    // ----------
    nmhe_predInit_pub = nh.advertise<std_msgs::Bool>("nmhe_learning/predInit", 1, true);
    nmhe_vel_pub = nh.advertise<std_msgs::Float64MultiArray>("nmhe_learning/uvw", 1, true);
    nmhe_dist_Fx_pub = nh.advertise<std_msgs::Float64MultiArray>("nmhe_learning/Fx", 1, true);
    nmhe_dist_Fy_pub = nh.advertise<std_msgs::Float64MultiArray>("nmhe_learning/Fy", 1, true);
    nmhe_dist_Fz_pub = nh.advertise<std_msgs::Float64MultiArray>("nmhe_learning/Fz", 1, true);
    nmhe_exeTime_pub = nh.advertise<std_msgs::Float64>("nmhe_learning/exeTime", 1, true);
    nmhe_kkt_pub = nh.advertise<std_msgs::Float64>("nmhe_learning/kkt", 1, true);
    nmhe_obj_pub = nh.advertise<std_msgs::Float64>("nmhe_learning/obj", 1, true);

    nmhe_struct.X0.resize(NMHE_NX);
    nmhe_struct.W.resize(NMHE_NY);
    nmhe_struct.WN.resize(NMHE_NYN);
    nmhe_struct.process_noise_cov.resize(NMHE_NX);
    nmhe_struct.SAC.resize(NMHE_NX);
    nmhe_struct.xAC.resize(NMHE_NX);

    // Roslaunch parameters
    ros::param::get("verbose",nmhe_struct.verbose);
    ros::param::get("X0_1",nmhe_struct.X0(0));
    ros::param::get("X0_2",nmhe_struct.X0(1));
    ros::param::get("X0_3",nmhe_struct.X0(2));
    ros::param::get("X0_4",nmhe_struct.X0(3));
    ros::param::get("X0_5",nmhe_struct.X0(4));
    ros::param::get("X0_6",nmhe_struct.X0(5));

    nmhe_struct.W << 0.0002, 0.0002, 0.0002, 0.0001, 0.0001, 0.0005;
    nmhe_struct.WN << 0.0002, 0.0002, 0.0002;

    nmhe_struct.process_noise_cov << 30, 30, 30, 8, 8, 8;
    nmhe_struct.SAC << 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3;
    nmhe_struct.xAC = nmhe_struct.X0;

    NMHE_FXFYFZ *nmhe = new NMHE_FXFYFZ(nmhe_struct);
    ros::Rate rate(1/sampleTime);

    current_vel_rate.resize(6);

    estimator_stop = false;

    for (int i=0; i<(int)(1/sampleTime); ++i)
    {
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok() && !estimator_stop)
    {
        t = ros::Time::now().toSec();

        if( current_state_msg.mode != "OFFBOARD" && print_flag_offboard == 1)
        {
            ROS_INFO("OFFBOARD mode is not enabled!");
            print_flag_offboard = 0;
        }
        if( !current_state_msg.armed && print_flag_arm == 1)
        {
            ROS_INFO("Vehicle is not armed!");
            print_flag_arm = 2;
        }
        else if(current_state_msg.armed && print_flag_arm == 2)
        {
            ROS_INFO("Vehicle is armed!");
            print_flag_arm = 0;
        }

        if( current_state_msg.mode == "ALTCTL")
        {
            if(print_flag_altctl == 1)
            {
                ROS_INFO("ALTCTL mode is enabled!");
                print_flag_altctl = 0;
            }
        }

        if (!nmhe->return_estimator_init_value())
        {
            nmhe->nmhe_init(nmhe->nmhe_struct);
            if (!nmhe_struct.verbose && nmhe->return_estimator_init_value())
            {
                std::cout<<"**********************************\n";
                std::cout<<"NMHE_FXFYFZ: initialized correctly\n";
                std::cout<<"**********************************\n";
            }
        }

        while(ros::ok() && estimation_on_msg.data && !estimator_stop)
        {
            if(estimation_on_msg.data && print_flag_estimation_on)
            {
                ROS_INFO("Estimation switch turned on!");
                print_flag_estimation_on = 0;
            }
            t_est_loop = ros::Time::now().toSec() - t;
            if (std::fmod(std::abs(t_est_loop - (int)(t_est_loop)), (double)(sampleTime)) == 0)
                std::cout<<"loop time for NMHE: " << t_est_loop << " (sec)"<<"\n";

            nmhe->nmhe_core(nmhe->nmhe_struct, nmhe->nmhe_est_struct, current_vel_rate, nmpc_cmd_ryp, nmpc_cmd_Fz);
            if(nmhe->acado_feedbackStep_fb != 0)
                estimator_stop = true;

            if(std::isnan(nmhe->nmhe_struct.x[(nmhe->nmhe_struct.acado_N*nmhe->nmhe_struct.acado_NX)]) == true ||
               std::isnan(nmhe->nmhe_struct.x[(nmhe->nmhe_struct.acado_N*nmhe->nmhe_struct.acado_NX + 1)]) == true ||
               std::isnan(nmhe->nmhe_struct.x[(nmhe->nmhe_struct.acado_N*nmhe->nmhe_struct.acado_NX) + 2]) == true ||
               std::isnan(nmhe->nmhe_struct.x[(nmhe->nmhe_struct.acado_N*nmhe->nmhe_struct.acado_NX) + 3]) == true ||
               std::isnan(nmhe->nmhe_struct.x[(nmhe->nmhe_struct.acado_N*nmhe->nmhe_struct.acado_NX) + 4]) == true ||
               std::isnan(nmhe->nmhe_struct.x[(nmhe->nmhe_struct.acado_N*nmhe->nmhe_struct.acado_NX) + 5]) == true)
            {
                ROS_ERROR_STREAM("Estimator ERROR at time = " << ros::Time::now().toSec() - t <<" (sec)" );
                estimator_stop = true;
                exit(0);
            }

            nmhe->publish_uvw_FxFyFz(nmhe->nmhe_est_struct);

            print_flag_offboard = 1;
            print_flag_arm = 1;
            print_flag_altctl = 1;

            ros::spinOnce();
            rate.sleep();
        }

        if(!estimation_on_msg.data && !print_flag_estimation_on)
        {
            ROS_INFO("Waiting for estimation switch to begin!");
            print_flag_estimation_on = 1;
        }
        else if(estimation_on_msg.data && print_flag_estimation_on)
        {
            ROS_INFO("estimation switch turned on!");
            print_flag_estimation_on = 0;
        }

        nmhe->publish_uvw_FxFyFz(nmhe->nmhe_est_struct);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
