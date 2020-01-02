#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
using namespace std;
mavros_msgs::State uav0_current_state;
mavros_msgs::State uav1_current_state;
mavros_msgs::State uav2_current_state;
void uav0_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav0_current_state = *msg;
}
void uav1_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav1_current_state = *msg;
}
void uav2_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav2_current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "three_uav");
    ros::NodeHandle nh;

    ros::Subscriber uav0_state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav0/mavros/state", 10, uav0_state_cb);
    ros::Publisher uav0_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav0/mavros/setpoint_position/local", 10);
    ros::ServiceClient uav0_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming");
    ros::ServiceClient uav0_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");
    ros::Subscriber uav1_state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav1/mavros/state", 10, uav1_state_cb);
    ros::Publisher uav1_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav1/mavros/setpoint_position/local", 10);
    ros::ServiceClient uav1_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav1/mavros/cmd/arming");
    ros::ServiceClient uav1_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav1/mavros/set_mode");
    ros::Subscriber uav2_state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav2/mavros/state", 10, uav2_state_cb);
    ros::Publisher uav2_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav2/mavros/setpoint_position/local", 10);
    ros::ServiceClient uav2_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav2/mavros/cmd/arming");
    ros::ServiceClient uav2_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav2/mavros/set_mode");
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !uav0_current_state.connected){
        ROS_INFO("UAV0 Connecting...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("UAV0 Connected!");
    while(ros::ok() && !uav1_current_state.connected){
        ROS_INFO("UAV1 Connecting...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("UAV1 Connected!");
    while(ros::ok() && !uav2_current_state.connected){
        ROS_INFO("UAV2 Connecting...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("UAV2 Connected!");

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        uav0_local_pos_pub.publish(pose);
        uav1_local_pos_pub.publish(pose);
        uav2_local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    
    while(ros::ok()){
        if( uav0_current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( uav0_set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("UAV0 Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !uav0_current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( uav0_arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("UAV0 Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        if( uav1_current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( uav1_set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("UAV1 Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !uav1_current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( uav1_arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("UAV1 Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        if( uav2_current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( uav2_set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("UAV2 Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !uav2_current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( uav2_arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("UAV2 Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        
        uav0_local_pos_pub.publish(pose);
        uav1_local_pos_pub.publish(pose);
        uav2_local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}