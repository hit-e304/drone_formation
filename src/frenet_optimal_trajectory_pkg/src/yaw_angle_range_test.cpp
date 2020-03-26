/**此程序用以测试Quaternion2Euler()函数结果范围是0~2pi还是0～pi~-pi~0;
   以及position_target_local.yaw等yaw角度范围是0~2pi还是0～pi~-pi~0;
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
#include "opencvtest/contours.h"
#include "frenet_optimal_trajectory_pkg/CubicSplinePlanner.h"
#include "frenet_optimal_trajectory_pkg/RangeImpl.h"
// #include <mavros_msgs/AttitudeTarget.h>
#define M_DEG_TO_RAD 1/57.295
#define CONSTANTS_RADIUS_OF_EARTH 6371000
#define CAM_CENTER_X 0
#define CAM_CENTER_Z 0

using namespace CubicSplinePlanner;

double ahead_distance = 50;//实时目标点向前的距离
double k = 0.1; //速度的增益

enum MissionState
{
    TakeOff,//起飞
    Test_yaw
    // Patrol,//巡逻
    // TrackingCam
};
// int flag_take_off = 1;
MissionState PX4_current_mission_state = TakeOff;
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
geometry_msgs::TwistStamped current_velocity;
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& curr_v)
{
    current_velocity = *curr_v;
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
    ros::init(argc, argv, "yaw_angle_range_test_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_target_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

    ros::Subscriber local_position_sub = nh.subscribe("/mavros/local_position/pose", 1, local_pos_cb);
    ros::Subscriber local_velocity_sub = nh.subscribe("/mavros/local_position/velocity_body", 1, local_vel_cb);
    
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
    ROS_INFO("connected");

    mavros_msgs::PositionTarget position_target_local;
    PX4_current_mission_state = TakeOff;

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
        if(PX4_current_mission_state == TakeOff)
        {
            position_target_local.position.x = 0;
            position_target_local.position.y = 0;
            position_target_local.position.z = 14.3;
            if(abs(current_pose.pose.position.z -14.3) < 0.2)
            {
                PX4_current_mission_state = Test_yaw;
            }
        }
        else if (PX4_current_mission_state == Test_yaw)
        {
            position_target_local.position.x = current_pose.pose.position.x;
            position_target_local.position.y = current_pose.pose.position.y;
            position_target_local.position.z = 14.3;
            position_target_local.yaw = Quaternion2Euler(current_pose.pose.orientation).z;
            position_target_local.yaw_rate = 0.3;
            
            std::cout << "position_target_local.yaw = "<< position_target_local.yaw << "    " << "Quaternion2Euler() = " << Quaternion2Euler(current_pose.pose.orientation).z << std::endl;
        }
        
        local_target_pub.publish(position_target_local);
        // std::cout << fly_log.is_open() << std::endl;
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}