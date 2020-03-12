#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<mavros_msgs/PositionTarget.h>
#include<mavros_msgs/SetMode.h>
#include<mavros_msgs/State.h>
#include<mavros_msgs/CommandBool.h>
#include<tf/tf.h>

enum MissionState2
{
    TakeOff,//起飞
    TrackingUAV1
};
MissionState2 UAV2_current_mission_state = TakeOff;

mavros_msgs::State current_state2;
void state_cb2(const mavros_msgs::State::ConstPtr& msg)
{
    current_state2 = *msg;
}
geometry_msgs::PoseStamped current_pose2;
void local_pos_cb2(const geometry_msgs::PoseStamped::ConstPtr& curr_p)
{
    current_pose2 = *curr_p;
}
mavros_msgs::PositionTarget local_target_position2;
void local_tar_pos_cb2(const mavros_msgs::PositionTarget::ConstPtr& local_tar_pos2)
{
    local_target_position2 = *local_tar_pos2;
}
geometry_msgs::PoseStamped UAV1_local_pose;
void UAV1_local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& UAV1_local_po)
{
    UAV1_local_pose = *UAV1_local_po;
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
    ros::init(argc, argv, "UAV2_follower_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub2 = nh.subscribe<mavros_msgs::State>("/uav2/mavros/state", 10, state_cb2);
    ros::Subscriber local_position_sub2 = nh.subscribe("/uav2/mavros/local_position/pose",10,local_pos_cb2);
    ros::Subscriber local_target_position_sub2 = nh.subscribe("/uav1/mavros/setpoint_raw/local",10,local_tar_pos_cb2);
    ros::Subscriber UAV1_local_pose_sub = nh.subscribe("/uav1/mavros/local_position/pose",10,UAV1_local_pose_cb);
    ros::Publisher UAV2_target_position_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav2/mavros/setpoint_raw/local", 10);


    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav2/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav2/mavros/set_mode");
    
    ros::Rate rate(20.0);

    while (ros::ok() && !current_state2.connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting...");
    }
    ROS_INFO("connected!");

    mavros_msgs::PositionTarget position_target_local2;
    position_target_local2.position.x = -3;
    position_target_local2.position.y = 0;
    position_target_local2.position.z = 14.3;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        UAV2_target_position_pub.publish(position_target_local2);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode2;
    offb_set_mode2.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd2;
    arm_cmd2.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        if( current_state2.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode2) &&
                offb_set_mode2.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state2.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd2) &&
                    arm_cmd2.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(UAV2_current_mission_state == TakeOff){
            position_target_local2.position.x = -3;
            position_target_local2.position.y = 0;
            position_target_local2.position.z = 14.3;

            if(14.3 - current_pose2.pose.position.z < 0.2){
                UAV2_current_mission_state = TrackingUAV1;
            }
        }
        else if(UAV2_current_mission_state == TrackingUAV1){
            if(Quaternion2Euler(UAV1_local_pose.pose.orientation).z < 1.57 && Quaternion2Euler(UAV1_local_pose.pose.orientation).z > -1.57){
                position_target_local2.position.x = local_target_position2.position.x - 3;
            }
            else{
                position_target_local2.position.x = local_target_position2.position.x + 3;
            }
            position_target_local2.position.y = local_target_position2.position.y;
            position_target_local2.position.z = local_target_position2.position.z;
            // position_target_local2.velocity.x = local_target_position2.velocity.x;
            // position_target_local2.velocity.y = local_target_position2.velocity.y;
            // position_target_local2.velocity.z = local_target_position2.velocity.z;
        }
        UAV2_target_position_pub.publish(position_target_local2);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}




