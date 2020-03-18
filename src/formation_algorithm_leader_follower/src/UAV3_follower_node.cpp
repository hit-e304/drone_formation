#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<mavros_msgs/PositionTarget.h>
#include<mavros_msgs/SetMode.h>
#include<mavros_msgs/State.h>
#include<mavros_msgs/CommandBool.h>
#include<tf/tf.h>


double rho = 5.0;//距离UAV1的距离
double beta = 0.0;//与UAV1方向的夹角
double d_threshold = 2.0;
double k_v = 1.0;

enum MissionState2
{
    TakeOff,//起飞
    TrackingUAV1
};
MissionState2 UAV3_current_mission_state = TakeOff;

mavros_msgs::State current_state3;
void state_cb3(const mavros_msgs::State::ConstPtr& msg)
{
    current_state3 = *msg;
}
geometry_msgs::PoseStamped current_pose3;
void local_pos_cb3(const geometry_msgs::PoseStamped::ConstPtr& curr_p)
{
    current_pose3 = *curr_p;
}
mavros_msgs::PositionTarget UAV1_local_tar_pose;
void local_tar_pos_cb3(const mavros_msgs::PositionTarget::ConstPtr& local_tar_pos3)
{
    UAV1_local_tar_pose = *local_tar_pos3;
}
geometry_msgs::PoseStamped UAV1_local_pose;
void UAV1_local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& UAV1_local_po)
{
    UAV1_local_pose = *UAV1_local_po;
}
geometry_msgs::PoseStamped UAV2_local_pose;
void local_pos_cb2(const geometry_msgs::PoseStamped::ConstPtr& curr_p)
{
    UAV2_local_pose = *curr_p;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "UAV3_follower_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub3 = nh.subscribe<mavros_msgs::State>("/uav3/mavros/state", 10, state_cb3);
    ros::Subscriber UAV3_local_pose_sub = nh.subscribe("/uav3/mavros/local_position/pose",10,local_pos_cb3);
    ros::Subscriber local_target_position_sub3 = nh.subscribe("/uav1/mavros/setpoint_raw/local",10,local_tar_pos_cb3);
    ros::Subscriber UAV1_local_pose_sub = nh.subscribe("/uav1/mavros/local_position/pose",10,UAV1_local_pose_cb);
    ros::Publisher UAV3_target_position_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav3/mavros/setpoint_raw/local", 10);
    ros::Subscriber UAV2_local_pose_sub = nh.subscribe("/uav2/mavros/local_position/pose",10,local_pos_cb2);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav3/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav3/mavros/set_mode");
    
    ros::Rate rate(20.0);

    while (ros::ok() && !current_state3.connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting...");
    }
    ROS_INFO("connected!");

    // double distance_UAV1_UAV2 = sqrt(pow(UAV1_local_pose.pose.position.x - current_pose2.pose.position.x, 2) + pow(UAV1_local_pose.pose.position.y - current_pose2.pose.position.y, 2) + pow(UAV1_local_pose.pose.position.z - current_pose2.pose.position.z, 2));
    double distance_UAV1_UAV3 = sqrt(pow(UAV1_local_pose.pose.position.x - current_pose3.pose.position.x, 2) + pow(UAV1_local_pose.pose.position.y - current_pose3.pose.position.y, 2) + pow(UAV1_local_pose.pose.position.z - current_pose3.pose.position.z, 2));
    double distance_UAV2_UAV3 = sqrt(pow(UAV2_local_pose.pose.position.x - current_pose3.pose.position.x, 2) + pow(UAV2_local_pose.pose.position.y - current_pose3.pose.position.y, 2) + pow(UAV2_local_pose.pose.position.z - current_pose3.pose.position.z, 2));
    // double v12_x = 0.0;
    // double v12_y = 0.0;
    double v23_x = 0.0;
    double v23_y = 0.0;
    double v13_x = 0.0;
    double v13_y = 0.0;

    mavros_msgs::PositionTarget position_target_local3;
    position_target_local3.position.x = 0;
    position_target_local3.position.y = 0;
    position_target_local3.position.z = 14.3;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        UAV3_target_position_pub.publish(position_target_local3);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode3;
    offb_set_mode3.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd3;
    arm_cmd3.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        if( current_state3.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode3) &&
                offb_set_mode3.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state3.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd3) &&
                    arm_cmd3.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(UAV3_current_mission_state == TakeOff){
            position_target_local3.position.x = 0;
            position_target_local3.position.y = 0;
            // position_target_local3.position.x = UAV1_local_tar_pose.position.x - rho * cos(UAV1_local_tar_pose.yaw + beta);
            // position_target_local3.position.y = UAV1_local_tar_pose.position.x - rho * sin(UAV1_local_tar_pose.yaw + beta);
            position_target_local3.position.z = 14.3;

            if(14.3 - current_pose3.pose.position.z < 0.2){
                UAV3_current_mission_state = TrackingUAV1;
            }
        }
        else if(UAV3_current_mission_state == TrackingUAV1)
        {
            position_target_local3.yaw = UAV1_local_tar_pose.yaw;
            position_target_local3.position.x = UAV1_local_tar_pose.position.x - rho * cos(position_target_local3.yaw + beta);
            position_target_local3.position.y = UAV1_local_tar_pose.position.y - rho * sin(position_target_local3.yaw + beta);
            // distance_UAV1_UAV2 = sqrt(pow(UAV1_local_pose.pose.position.x - current_pose2.pose.position.x, 2) + pow(UAV1_local_pose.pose.position.y - current_pose2.pose.position.y, 2) + pow(UAV1_local_pose.pose.position.z - current_pose2.pose.position.z, 2));
            distance_UAV1_UAV3 = sqrt(pow(UAV1_local_pose.pose.position.x - current_pose3.pose.position.x, 2) + pow(UAV1_local_pose.pose.position.y - current_pose3.pose.position.y, 2) + pow(UAV1_local_pose.pose.position.z - current_pose3.pose.position.z, 2));
            distance_UAV2_UAV3 = sqrt(pow(UAV2_local_pose.pose.position.x - current_pose3.pose.position.x, 2) + pow(UAV2_local_pose.pose.position.y - current_pose3.pose.position.y, 2) + pow(UAV2_local_pose.pose.position.z - current_pose3.pose.position.z, 2));
    
            if(distance_UAV1_UAV3 < d_threshold)
            {
                v13_x = k_v / (current_pose3.pose.position.x - UAV1_local_pose.pose.position.x);
                v13_y = k_v / (current_pose3.pose.position.x - UAV1_local_pose.pose.position.x);
            }
            else
            {
                v13_x = 0.0;
                v13_y = 0.0;
            }
            if(distance_UAV2_UAV3 < d_threshold)
            {
                v23_x = k_v / (current_pose3.pose.position.x - UAV2_local_pose.pose.position.x);
                v23_y = k_v / (current_pose3.pose.position.y - UAV2_local_pose.pose.position.y);
            }
            else
            {
                v23_x = 0.0;
                v23_y = 0.0;
            }
            position_target_local3.velocity.x = v13_x + v23_x;
            position_target_local3.velocity.y = v13_y + v23_y;
        }
        UAV3_target_position_pub.publish(position_target_local3);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}




