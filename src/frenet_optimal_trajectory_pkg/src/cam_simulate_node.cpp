#include <ros/ros.h>
#include <vector>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

#include "opencvtest/img_pro_info.h"

//两个框的位置和方向
geometry_msgs::PoseStamped box_1;
box_1.pose.position.x = 500;
box_1.pose.position.y = 100;//y可以随机加减14m
box_1.pose.position.z = 14.3;
box_1.pose.orientation = tf::createQuaternionMsgFromYaw(0.00);//只通过y即绕z的旋转角度计算四元数，用于平面小车。返回四元数 
//tf::createQuaternionMsgFromRollPitchYaw(double r, double p, double y);//返回四元数
// box_1_yaw = 0.00;
geometry_msgs::PoseStamped box_2;
box_2.pose.position.x = 500;
box_2.pose.position.y = 300;//y可以随机加减14m
box_2.pose.position.z = 14.3;
box_2.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI)
// box_2_yaw = M_PI;

//可视范围
double d_min = 16;//m
double d_max = 80;//m

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
    ros::init(argc, argv, "cam_simulate_node");
    ros::NodeHandle nh;
    //获取飞机的位置
    ros::Subscriber local_position_sub = nh.subscribe("/mavros/local_position/pose", 1, local_pos_cb);
    contours_pub_ = nh_.advertise<opencvtest::img_pro_info>("/contours_topic", 50);

    ros::Rate rate(20.0);

    opencvtest::img_pro_info sim_cam_info;
    // sim_cam_info.find_obs_flag = false;
    // sim_cam_info.dis = -1;
    // sim_cam_info.pos_left = -1;
    // sim_cam_info.pos_right = -1;
    // sim_cam_info.x_pos = -1;
    // sim_cam_info.y_pos = -1;

    double distance_from_plane_to_box1;
    double distance_from_plane_to_box2;
    double error_yaw_between_plane_and_box1;
    double error_yaw_between_plane_and_box2;

    //生成高斯分布（正态分布）的随机数
    std::default_random_engine e; //引擎
    std::normal_distribution<double> gauss_rand(0, 0.1);//均值，方差

    while(ros::ok())
    {
        sim_cam_info.find_obs_flag = false;
        sim_cam_info.dis = -1;
        sim_cam_info.pos_left = -1;
        sim_cam_info.pos_right = -1;
        sim_cam_info.x_pos = -1;
        sim_cam_info.y_pos = -1;

        distance_from_plane_to_box1 = sqrt(pow((current_pose.pose.position.x-box_1.pose.position.x),2) + pow((current_pose.pose.position.y-box_1.pose.position.y),2) + pow((current_pose.pose.position.z-box_1.pose.position.z),2));
        distance_from_plane_to_box2 = sqrt(pow((current_pose.pose.position.x-box_2.pose.position.x),2) + pow((current_pose.pose.position.y-box_2.pose.position.y),2) + pow((current_pose.pose.position.z-box_2.pose.position.z),2));
        error_yaw_between_plane_and_box1 = current_pose.pose.orientation.z - box_1.pose.orientation.z;
        error_yaw_between_plane_and_box2 = current_pose.pose.orientation.w - box_2.pose.orientation.w;
        //飞机距离框1在16~80m之间，且yaw偏角与box1偏差在23度（用姿态四元数换算过来的，不一定准确，box1认为yaw指向为0）
        if(distance_from_plane_to_box1 < d_max && distance_from_plane_to_box1 > d_min && error_yaw_between_plane_and_box1 < 0.2474 && error_yaw_between_plane_and_box1 > -0.2474)
        {
            sim_cam_info.find_obs_flag = true;
            sim_cam_info.dis = distance_from_plane_to_box1 + gauss_rand(e);
            sim_cam_info.x_pos = (int) 10 * atan2(current_pose.pose.position.y - box_1.pose.position.y, distance_from_plane_to_box1);
            sim_cam_info.y_pos = (int) 10 * atan2(box_1.pose.position.z - current_pose.pose.position.z, distance_from_plane_to_box1);
        }
        //飞机距离框2在16~80m之间，且yaw偏角与box2偏差在23度（用姿态四元数换算过来的，不一定准确，box1认为yaw指向为pi）
        if(distance_from_plane_to_box2 < d_max && distance_from_plane_to_box2 > d_min && error_yaw_between_plane_and_box2 < 0.2474 && error_yaw_between_plane_and_box2 > 0)
        {
            sim_cam_info.find_obs_flag = true;
            sim_cam_info.dis = distance_from_plane_to_box2 + gauss_rand(e);
            sim_cam_info.x_pos = (int) 10 * atan2(current_pose.pose.position.y - box_2.pose.position.y, distance_from_plane_to_box2);
            sim_cam_info.y_pos = (int) 10 * atan2(box_2.pose.position.z - current_pose.pose.position.z, distance_from_plane_to_box2);
        }

        contours_pub_.publish(sim_cam_info);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
