/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <vector>
#include <fstream>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Dense>
// #include <Eigen/LU>
#include "opencvtest/contours.h"
// #include <mavros_msgs/AttitudeTarget.h>
#define M_DEG_TO_RAD 1/57.295
#define CONSTANTS_RADIUS_OF_EARTH 6371000
#define CAM_CENTER_X 0
#define CAM_CENTER_Z 0

//将地理学坐标系(geographic coordinate system)中的点(球)投影到本地方位等距平面(XOY)中
// geometry_msgs::Point world_to_local(const double ref_lat, const double ref_lon, double lat, double lon) {

//     double lat_rad = lat * M_DEG_TO_RAD; // 度 -> 弧度 A/57.295
//     double lon_rad = lon * M_DEG_TO_RAD; // GPS数据角度单位为弧度
//     double ref_lat_rad = ref_lat * M_DEG_TO_RAD;
//     double ref_lon_rad = ref_lon * M_DEG_TO_RAD;
//     double sin_lat = sin(lat_rad); //程序中三角运算使用的是弧度
//     double cos_lat = cos(lat_rad);
//     double sin_ref_lat = sin(ref_lat_rad);
//     double cos_ref_lat = cos(ref_lat_rad);
// //    double sin_ref_lon = sin(ref_lon_rad);
//     double cos_d_lon = cos(lon_rad - ref_lon_rad);
//     double arg = sin_ref_lat * sin_lat + cos_ref_lat * cos_lat * cos_d_lon;
//     geometry_msgs::Point local_position_from_geography;

//     if (arg > 1.0) {
//         arg = 1.0;
//     } else if (arg < -1.0) {
//         arg = -1.0; //限幅
//     }
//     double c = acos(arg);
//     double k = (fabs(c) < DBL_EPSILON) ? 1.0 : (c / sin(c));// c为正数
//     local_position_from_geography.x = k * (cos_ref_lat * sin_lat - sin_ref_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
//     local_position_from_geography.y = k * cos_lat * sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;

//     return local_position_from_geography;
// }



// void race_track_to_local(std::vector<sensor_msgs::NavSatFix> race_track, double &yaw, sensor_msgs::NavSatFix& target_global) {
//     if (race_track.size() != 4) {
//         return;
//     } else {
//         sensor_msgs::NavSatFix world_origin;
//         world_origin.latitude = 0.5 * (race_track[0].latitude + race_track[1].latitude);
//         world_origin.longitude = 0.5 * (race_track[0].longitude + race_track[1].longitude);
//         std::vector<geometry_msgs::Point> four_base_point_local = {
//                 world_to_local(world_origin.latitude, world_origin.longitude, race_track[0].latitude, race_track[0].longitude),
//                 world_to_local(world_origin.latitude, world_origin.longitude, race_track[1].latitude, race_track[1].longitude),
//                 world_to_local(world_origin.latitude, world_origin.longitude, race_track[2].latitude, race_track[2].longitude),
//                 world_to_local(world_origin.latitude, world_origin.longitude, race_track[3].latitude, race_track[3].longitude)};
//         yaw = atan2((four_base_point_local[2].y + four_base_point_local[3].y)/2, (four_base_point_local[2].x + four_base_point_local[3].x)/2);
//         target_global.latitude = 0.5 * (race_track[2].latitude + race_track[3].latitude);
//         target_global.longitude = 0.5 * (race_track[2].longitude + race_track[3].longitude);
//     }
// }

// void write_txt(string filePath)
// {
//     ofstream fly_log(filePath);
//     if(fly_log.is_open())
//     {
//         fly_log << "position" << std::setw(16) << "attitude" << std::setw(16) << "velocity" << std::endl;
//         fly_log.close();
//     }
// }

int flag_take_off = 1;
tfScalar yaw,pitch,roll;
tf::Quaternion q;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
geometry_msgs::PoseStamped current_pose;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& curr_p)
{
    current_pose = *curr_p;
}
opencvtest::img_pro_info camera_data;
void cam_subCallback(const opencvtest::img_pro_info::ConstPtr& cam_data){ // camera data callback
	camera_data = *cam_data;
}

geometry_msgs::Vector3 Quaternion2Euler(const geometry_msgs::Quaternion msg)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);
 
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll_, pitch_, yaw_;
    tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);
 
    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll_ ;
    rpy.y = pitch_ ;
    rpy.z = yaw_ ;

	return rpy;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    ros::Publisher local_target_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    // ros::Publisher body_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
    // ros::Publisher body_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel", 1);
    // ros::Publisher body_att_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 1);

    ros::Subscriber local_position_sub = nh.subscribe("/mavros/local_position/pose", 1, local_pos_cb);
    ros::Subscriber cam_sub = nh.subscribe("/contours_topic", 1, cam_subCallback);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting...");
    }

    // 目的地的local坐标位置
    geometry_msgs::Pose destination;
    destination.position.x = 300;
    destination.position.y = 300;

    mavros_msgs::PositionTarget position_target_local;
    position_target_local.position.x = 0;
    position_target_local.position.y = 0;
    position_target_local.position.z = 3;
    position_target_local.yaw = atan2(destination.position.y, destination.position.x);
    // position_target_local.velocity.x = 0.5;
    // position_target_local.velocity.y = 2;

    // 新坐标系下的坐标
    mavros_msgs::PositionTarget position_target_new;
    geometry_msgs::Pose current_pose_new;
    Eigen::Matrix3d Matrix_from_local_to_new;
    Matrix_from_local_to_new << cos(position_target_local.yaw), -sin(position_target_local.yaw), 0,
                                sin(position_target_local.yaw), cos(position_target_local.yaw), 0,
                                0, 0, 1;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_target_pub.publish(position_target_local);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(flag_take_off)
        {
            position_target_local.position.x = 0;
            position_target_local.position.y = 0;
            position_target_local.position.z = 3;

            if(3 - current_pose.pose.position.z < 0.2)
            {
                flag_take_off = 0;
            }
            local_target_pub.publish(position_target_local);
        }
        else 
        {
            Eigen::Vector3d temp_pos(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
            Eigen::Vector3d temp_pos_new;
            temp_pos_new = Matrix_from_local_to_new.transpose() * temp_pos;
            current_pose_new.position.x = temp_pos_new(0);
            current_pose_new.position.y = temp_pos_new(1);
            current_pose_new.position.z = temp_pos_new(2);
            std::cout << temp_pos_new.adjoint() << std::endl;
            // position_target_new.position.x = 300;
            // position_target_new.position.y = 0;
            position_target_new.velocity.x = 0;
            if (camera_data.find_obs_flag)
            {
                position_target_new.position.x = current_pose_new.position.x;
                position_target_new.velocity.x = 0;
                position_target_new.position.y = current_pose_new.position.y - 0.01 * (camera_data.x_pos - CAM_CENTER_X);
                position_target_new.position.z = current_pose_new.position.z + 0.01 * (camera_data.y_pos - CAM_CENTER_Z);
            }
            else
            {
                position_target_new.position.x = 0;
                position_target_new.position.y = 0;
                position_target_new.position.z = 3;
                position_target_new.velocity.x = 0;
            }
            // if(current_pose_new.position.x > 310)
            // {
            //     position_target_new.position.x = current_pose_new.position.x;
            //     position_target_new.position.y = current_pose_new.position.y;
            //     position_target_new.velocity.x = 0;
            // }

            temp_pos_new << position_target_new.position.x,
                            position_target_new.position.y,
                            position_target_new.position.z;
            Eigen::Vector3d temp_vel_new(position_target_new.velocity.x, position_target_new.velocity.y, position_target_new.velocity.z);
            Eigen::Vector3d temp_vel;
            temp_pos = Matrix_from_local_to_new * temp_pos_new;
            position_target_local.position.x = temp_pos(0);
            position_target_local.position.y = temp_pos(1);
            position_target_local.position.z = temp_pos(2);
            temp_vel = Matrix_from_local_to_new * temp_vel_new;
            position_target_local.velocity.x = temp_vel(0);
            position_target_local.velocity.y = temp_vel(1);
            // position_target_local.velocity.z = temp_vel(2);
            local_target_pub.publish(position_target_local);
        }
        
        
        // std::cout << fly_log.is_open() << std::endl;
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
