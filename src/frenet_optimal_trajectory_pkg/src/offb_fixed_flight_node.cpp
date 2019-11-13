/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>


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
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Publisher body_frame_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    
    ros::Publisher body_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);

    ros::Publisher body_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel", 1);

    ros::Publisher body_att_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 1);

    ros::Subscriber local_position_sub = nh.subscribe("/mavros/local_position/pose", 1, local_pos_cb);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // mavros_msgs::AttitudeTarget body_attitude;
    // yaw = 1;
    // pitch = 3.14159/18;
    // roll = 0;
    // q.setRPY(roll, pitch, yaw);
    // // body_attitude.type_mask = 71;
    // body_attitude.orientation.x = q[0];
    // body_attitude.orientation.y = q[1];
    // body_attitude.orientation.z = q[2];
    // body_attitude.orientation.w = q[3];
    // body_attitude.thrust = 0.6;

    // geometry_msgs::TwistStamped body_vel;
    // body_vel.twist.linear.x = 1;

    // geometry_msgs::PoseStamped body_pose;
    // yaw = 1;
    // pitch = 3.1415962/10;
    // roll = 0;
    // q.setRPY(roll, pitch, yaw);
    // body_pose.pose.orientation.x = q[0];
    // body_pose.pose.orientation.y = q[1];
    // body_pose.pose.orientation.z = q[2];
    // body_pose.pose.orientation.w = q[3];

    mavros_msgs::PositionTarget position_target;
    position_target.position.x = 2;
    position_target.position.y = 2;
    position_target.position.z = 2;
    position_target.yaw = 0;
    // position_target.velocity.x = 0.5;
    // position_target.velocity.y = 2;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
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
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 2;
            // pose.pose.orientation.x = -0.00079153842542;
            // pose.pose.orientation.y = 0.000993744619493;
            // pose.pose.orientation.z = -0.483787700401;
            // pose.pose.orientation.w = -0.875184493391;

            if(2 - current_pose.pose.position.z < 0.2)
            {
                flag_take_off = 0;
            }
            local_pos_pub.publish(pose);
        }
        else 
        {
            body_frame_pub.publish(position_target);
            // body_attitude_pub.publish(body_attitude);
        }
        


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}