#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>
#include <fstream>
#include <tf/tf.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Dense>
#include "frenet_optimal_trajectory_pkg/CubicSplinePlanner.h"
#include "frenet_optimal_trajectory_pkg/RangeImpl.h"
using namespace CubicSplinePlanner;

// super parameter
double HIGHT = 4.0;
double ahead_distance = 0.6;//实时目标点向前的距离
double k = 0.1; //速度的增益
// flag
int flag_take_off = 1;
// uav0 callback function
mavros_msgs::State uav0_current_state;
void uav0_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav0_current_state = *msg;
}
geometry_msgs::PoseStamped uav0_current_pose;
void uav0_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& curr_p)
{
    uav0_current_pose = *curr_p;
}
geometry_msgs::TwistStamped uav0_current_velocity_body;
void uav0_body_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& curr_v)
{
    uav0_current_velocity_body = *curr_v;
}
geometry_msgs::TwistStamped uav0_current_velocity_local;
void uav0_local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& curr_v)
{
    uav0_current_velocity_local = *curr_v;
}

// uav1 callback function
mavros_msgs::State uav1_current_state;
void uav1_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav1_current_state = *msg;
}
geometry_msgs::PoseStamped uav1_current_pose;
void uav1_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& curr_p)
{
    uav1_current_pose = *curr_p;
}
geometry_msgs::TwistStamped uav1_current_velocity;
void uav1_body_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& curr_v)
{
    uav1_current_velocity = *curr_v;
}

// uav2 callback funciton
mavros_msgs::State uav2_current_state;
void uav2_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav2_current_state = *msg;
}
geometry_msgs::PoseStamped uav2_current_pose;
void uav2_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& curr_p)
{
    uav2_current_pose = *curr_p;
}
geometry_msgs::TwistStamped uav2_current_velocity;
void uav2_body_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& curr_v)
{
    uav2_current_velocity = *curr_v;
}
// Calculate Tools
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
// Cubic Spline Planner
Spline2D
generate_target_course(vector<double> x, vector<double> y, vector<double> &rx, vector<double> &ry, vector<double> &ryaw,
                       vector<double> &rc) {
    Spline2D csp(x, y);
    vector<double> ixy;
    // Cosmos::RangeImpl<int> s = Cosmos::Range(0, int(csp.s.back()), 5);
    for (auto i_s : Cosmos::Range(0, int(csp.s.back()), 0.02)) {
        ixy = csp.calc_position(i_s);
        rx.push_back(ixy[0]);
        ry.push_back(ixy[1]);
        ryaw.push_back(csp.calc_yaw(i_s));
        rc.push_back(csp.calc_curvature(i_s));
    }
    return csp;
}
// pure pursuit controller
int calc_target_index(geometry_msgs::PoseStamped curr_pose, vector<double> tx, vector<double> ty)
{
    int ind;
    vector<double> dx;
    vector<double> dy;
    vector<double> d;
    for(auto& itx : tx)
    {
        dx.push_back(curr_pose.pose.position.x - itx);
    }
    for(auto& ity : ty)
    {
        dy.push_back(curr_pose.pose.position.y - ity);
    }
    for(int i = 0; i < tx.size(); i++)
    {
        d.push_back(abs(sqrt(dx[i] * dx[i] + dy[i] * dy[i])));
    }
    auto d_min = min_element(d.begin(), d.end());
    ind = distance(d.begin(), d_min);

    double L = 0.0;
    double LF = k * uav0_current_velocity_body.twist.linear.x + ahead_distance;

    while (LF > L && ind + 1 < tx.size())
    {
        L += sqrt(pow(tx[ind + 1] - tx[ind], 2) + pow(ty[ind + 1] - ty[ind], 2));
        ind += 1;
    }
    
    return ind;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "three_uav");
    ros::NodeHandle nh;

    ros::Subscriber uav0_state_sub = nh.subscribe<mavros_msgs::State>("/uav1/mavros/state", 10, uav0_state_cb);
    ros::Publisher uav0_local_target_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/local", 10);
    ros::Subscriber uav0_local_position_sub = nh.subscribe("/uav1/mavros/local_position/pose", 1, uav0_local_pos_cb);
    ros::Subscriber uav0_body_velocity_sub = nh.subscribe("/uav1/mavros/local_position/velocity_body", 1, uav0_body_vel_cb);
    ros::Subscriber uav0_local_velocity_sub = nh.subscribe("/uav1/mavros/local_position/velocity_local", 1, uav0_local_vel_cb);
    ros::ServiceClient uav0_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
    ros::ServiceClient uav0_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");

    ros::Subscriber uav1_state_sub = nh.subscribe<mavros_msgs::State>("/uav2/mavros/state", 10, uav1_state_cb);
    ros::Publisher uav1_local_target_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav2/mavros/setpoint_raw/local", 10);
    ros::Subscriber uav1_local_position_sub = nh.subscribe("/uav2/mavros/local_position/pose", 1, uav1_local_pos_cb);
    ros::Subscriber uav1_body_velocity_sub = nh.subscribe("/uav2/mavros/local_position/velocity_body", 1, uav1_body_vel_cb);
    ros::ServiceClient uav1_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav2/mavros/cmd/arming");
    ros::ServiceClient uav1_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav2/mavros/set_mode");

    ros::Subscriber uav2_state_sub = nh.subscribe<mavros_msgs::State>("/uav3/mavros/state", 10, uav2_state_cb);
    ros::Publisher uav2_local_target_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav3/mavros/setpoint_raw/local", 10);
    ros::Subscriber uav2_local_position_sub = nh.subscribe("/uav3/mavros/local_position/pose", 1, uav2_local_pos_cb);
    ros::Subscriber uav2_body_velocity_sub = nh.subscribe("/uav3/mavros/local_position/velocity_body", 1, uav2_body_vel_cb);
    ros::ServiceClient uav2_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav3/mavros/cmd/arming");
    ros::ServiceClient uav2_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav3/mavros/set_mode");

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

    // Planning

    vector<double> x = {0.0, 2.0, 3.0, 4.0, 5.0, 4.5, 3.5, 2.0, 0.0};
    vector<double> y = {0.0, 0.5, 0.0, -0.5, 0.0, 1.5, 2.5, 2.0, 1.0};

    vector<double> tx;  //x坐标
    vector<double> ty;  //y坐标
    vector<double> tyaw;//偏航
    vector<double> tc;  //曲率
    int target_index;          //目标索引

    Spline2D csp = generate_target_course(x, y, tx, ty, tyaw, tc);
// //////////////////////Planning over////////////////////////////////////////////////////////
    mavros_msgs::PositionTarget uav0_position_target_local;
    uav0_position_target_local.position.x = 0;
    uav0_position_target_local.position.y = 0;
    uav0_position_target_local.position.z = HIGHT / 2;
    mavros_msgs::PositionTarget uav1_position_target_local;
    uav1_position_target_local.position.x = 0;
    uav1_position_target_local.position.y = 0;
    uav1_position_target_local.position.z = HIGHT / 2;
    mavros_msgs::PositionTarget uav2_position_target_local;
    uav2_position_target_local.position.x = 0;
    uav2_position_target_local.position.y = 0;
    uav2_position_target_local.position.z = HIGHT / 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        uav0_local_target_pub.publish(uav0_position_target_local);
        uav1_local_target_pub.publish(uav1_position_target_local);
        uav2_local_target_pub.publish(uav2_position_target_local);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    
    while(ros::ok()){
        if( uav0_current_state.mode != "OFFBOARD" && uav1_current_state.mode != "OFFBOARD" && uav2_current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( uav0_set_mode_client.call(offb_set_mode) && uav1_set_mode_client.call(offb_set_mode) && uav2_set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("UAV0 Offboard enabled");
                ROS_INFO("UAV1 Offboard enabled");
                ROS_INFO("UAV2 Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !uav0_current_state.armed && !uav1_current_state.armed && !uav2_current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( uav0_arming_client.call(arm_cmd) && uav1_arming_client.call(arm_cmd) && uav2_arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("UAV0 Vehicle armed");
                    ROS_INFO("UAV1 Vehicle armed");
                    ROS_INFO("UAV2 Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(flag_take_off)
        {
            // uav0_position_target_local.position.x = 0;
            // uav0_position_target_local.position.y = 0;
            // uav0_position_target_local.position.z = HIGHT / 2;

            if(HIGHT/2 - uav0_current_pose.pose.position.z < 0.2)
            {
                flag_take_off = 0;
            }
            uav0_local_target_pub.publish(uav0_position_target_local);
            uav1_local_target_pub.publish(uav1_position_target_local);
            uav2_local_target_pub.publish(uav2_position_target_local);
        }
        else
        {
            target_index = calc_target_index(uav0_current_pose, tx, ty);
            uav0_position_target_local.position.x = tx[target_index];
            uav0_position_target_local.position.y = ty[target_index];
            uav0_position_target_local.yaw = tyaw[target_index];

            uav1_position_target_local.position.x = uav0_current_pose.pose.position.x - uav0_current_velocity_local.twist.linear.x + 1;
            uav1_position_target_local.position.y = uav0_current_pose.pose.position.y - uav0_current_velocity_local.twist.linear.y;
            uav1_position_target_local.position.z = uav0_current_pose.pose.position.z + 0.5;
            uav1_position_target_local.yaw = Quaternion2Euler(uav0_current_pose.pose.orientation).z;

            uav2_position_target_local.position.x = uav0_current_pose.pose.position.x - uav0_current_velocity_local.twist.linear.x;
            uav2_position_target_local.position.y = uav0_current_pose.pose.position.y - uav0_current_velocity_local.twist.linear.y + 1;
            uav2_position_target_local.position.z = uav0_current_pose.pose.position.z - 0.5;
            uav2_position_target_local.yaw = Quaternion2Euler(uav0_current_pose.pose.orientation).z;

            uav0_local_target_pub.publish(uav0_position_target_local);
            uav1_local_target_pub.publish(uav1_position_target_local);
            uav2_local_target_pub.publish(uav2_position_target_local);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}