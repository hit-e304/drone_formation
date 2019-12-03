/**
 * @file path_tracking_node.cpp
 * @brief Generate global path waypoint with cubic spline and tracking the path used pure pusuit planning,
 * written with MAVROS version 0.19.x, PX4 Pro Flight
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
#include "frenet_optimal_trajectory_pkg/CubicSplinePlanner.h"
#include "frenet_optimal_trajectory_pkg/RangeImpl.h"

using namespace CubicSplinePlanner;

// #include <Eigen/LU>
// #include "opencvtest/contours.h"

double HIGHT = 10.0;
double ahead_distance = 0.6;//实时目标点向前的距离
double k = 0.1; //速度的增益

int flag_take_off = 1;
tfScalar yaw,pitch,roll;
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

Spline2D
generate_target_course(vector<double> x, vector<double> y, vector<double> &rx, vector<double> &ry, vector<double> &ryaw,
                       vector<double> &rc) {
    Spline2D csp(x, y);
    vector<double> ixy;
    // Cosmos::RangeImpl<int> s = Cosmos::Range(0, int(csp.s.back()), 5);
    for (auto i_s : Cosmos::Range(0, int(csp.s.back()), 0.2)) {
        ixy = csp.calc_position(i_s);
        rx.push_back(ixy[0]);
        ry.push_back(ixy[1]);
        ryaw.push_back(csp.calc_yaw(i_s));
        rc.push_back(csp.calc_curvature(i_s));
    }
    return csp;
}

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
    double LF = k * current_velocity.twist.linear.x + ahead_distance;
    while (LF > L && ind + 1 < tx.size())
    {
        L += sqrt(pow(tx[ind + 1] - tx[ind], 2) + pow(ty[ind + 1] - ty[ind], 2));
        ind += 1;
    }
    
    return ind;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_tracking_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_target_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::Subscriber local_position_sub = nh.subscribe("/mavros/local_position/pose", 1, local_pos_cb);
    ros::Subscriber local_velocity_sub = nh.subscribe("/mavros/local_position/velocity_body", 1, local_vel_cb);
    // ros::Subscriber cam_sub = nh.subscribe("/contours_topic", 1, cam_subCallback);
    
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
    ROS_INFO("connected!");

    mavros_msgs::PositionTarget position_target_local;
    position_target_local.position.x = 0;
    position_target_local.position.y = 0;
    position_target_local.position.z = HIGHT / 2;

    vector<double> x = {0.0, 2.0, 3.0, 4.0, 5.0, 4.5, 3.5, 2.0, 0.0};
    vector<double> y = {0.0, 0.5, 0.0, -0.5, 0.0, 1.5, 2.5, 2.0, 1.0};

    vector<double> tx;  //x坐标
    vector<double> ty;  //y坐标
    vector<double> tyaw;//偏航
    vector<double> tc;  //曲率
    int target_index;          //目标索引

    Spline2D csp = generate_target_course(x, y, tx, ty, tyaw, tc);

    ofstream plan_log("/home/dqn/drone_formation/src/plan_log.txt");
    if(plan_log.is_open())
    {
        for(int i = 0; i < tx.size(); i++)
        {
            plan_log << std::setw(16) << tx[i]
                     << std::setw(16) << ty[i]
                     << std::setw(16) << tyaw[i]
                     << std::endl;
        }
    }
    plan_log.close();

    //send a few setpoints before starting
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     local_target_pub.publish(position_target_local);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    ofstream fly_log("/home/dqn/drone_formation/src/fly_log.txt");
    

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
            position_target_local.position.z = HIGHT / 2;

            if(HIGHT/2 - current_pose.pose.position.z < 0.2)
            {
                flag_take_off = 0;
            }
            local_target_pub.publish(position_target_local);
        }
        else
        {
            target_index = calc_target_index(current_pose, tx, ty);
            position_target_local.position.x = tx[target_index];
            position_target_local.position.y = ty[target_index];
            position_target_local.yaw = tyaw[target_index];
            local_target_pub.publish(position_target_local);
        }


        if(fly_log.is_open())
        {
            fly_log << std::setw(16) << current_pose.pose.position.x 
                    << std::setw(16) << current_pose.pose.position.y 
                    << std::setw(16) << current_pose.pose.position.z 
                    << std::setw(16) << Quaternion2Euler(current_pose.pose.orientation).x
                    << std::setw(16) << Quaternion2Euler(current_pose.pose.orientation).y
                    << std::setw(16) << Quaternion2Euler(current_pose.pose.orientation).z 
                    << std::endl;
                    
        }
        // std::cout << fly_log.is_open() << std::endl;
        ros::spinOnce();
        rate.sleep();
    }
    fly_log.close();
    return 0;
}








