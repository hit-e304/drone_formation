/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
note：1.position_target_local.yaw和Quaternion2Euler()函数输出的角度范围都是0～pi～-pi～0（详情可见yaw_angle_range_test.cpp）
     2.mavros_msgs::PositionTarget里面的yaw和yaw_rate两个参数在设置时,不能设置成yaw=0;yaw_rate=number(number表示角速率)，这样飞机不会转的；
       而应该设置为yaw=Quaternion2Euler(current_pose.pose.orientation).z（即当前yaw值）;yaw_rate=number（number表示角速率），这样飞机才会按照设定的角速率转；
       或者设置为yaw=Quaternion2Euler(current_pose.pose.orientation).z + xx（即当前yaw值加上一个角度xx）；yaw_rate=0（0表示飞控默认的角速率），这样飞机也会转，但是缺点是这样相当于开环跑，yaw可能一直跟不上；
       另一种情况若设定yaw=xx1；yaw_rate=xx2（即表示既设定yaw目标角度xx1，又设定角速率xx2），这时候表示的意思是飞机转到xx1角度的那个时刻拥有xx2的角速度；而并非以xx2角速度转到xx1角度去。
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
#include "frenet_optimal_trajectory_pkg/CubicSplinePlanner.h"
#include "frenet_optimal_trajectory_pkg/RangeImpl.h"
// #include <mavros_msgs/AttitudeTarget.h>
#define M_DEG_TO_RAD 1/57.295
#define CONSTANTS_RADIUS_OF_EARTH 6371000
#define CAM_CENTER_X 0
#define CAM_CENTER_Z 0

using namespace CubicSplinePlanner;
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

double ahead_distance = 50;//实时目标点向前的距离
double k = 0.1; //速度的增益
double d_min = 8;//m 模拟能看见框最近距离
double d_max = 80;//m 模拟能看见框最远距离
double d_warning = 45;//低于这个距离还未看到框即为不正常情况
bool box1_on_the_left = 1;//不正常情况看不见框1时先假设框1在无人机左边
bool box2_on_the_left = 1;//不正常情况看不见框2时先假设框2在无人机左边
bool no_emergency_flag = 1;//没有出现做了搜索措施还找不到框的情况
int task_process_flag =0;//记录已经成功穿越框的数量
bool armed_flag = 0;

enum MissionState
{
    TakeOff,//起飞
    emergency,//不正常情况采取措施后仍无法发现框即出现紧急情况
    Patrol,//巡逻
    TrackingCam,//基于视觉信息跟踪框
    Firstframe_cannotfind,//不正常情况太近不能看见第一个框
    Firstframe_acrossing,//正常情况距离太近不能看见第一个框,即正在穿越第一个框
    Secondframe_cannotfind,//不正常情况太近不能看见第二个框
    Secondframe_acrossing,//正常情况距离太近不能看见第二个框,即正在穿越第二个框
    Maneuvering_after_searching,//搜索到框后的机动任务状态
    landing//降落
};



//两个框的位置和方向
geometry_msgs::PoseStamped box_1;
geometry_msgs::PoseStamped box_2;

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

Spline2D
generate_target_course(vector<double> x, vector<double> y, vector<double> &rx, vector<double> &ry, vector<double> &ryaw,
                       vector<double> &rc) {
    Spline2D csp(x, y);
    vector<double> ixy;
    // Cosmos::RangeImpl<int> s = Cosmos::Range(0, int(csp.s.back()), 5);
    for (auto i_s : Cosmos::Range(0, int(csp.s.back()), 1)) {
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
    ros::Subscriber local_velocity_sub = nh.subscribe("/mavros/local_position/velocity_body", 1, local_vel_cb);
    ros::Subscriber cam_sub = nh.subscribe("/contours_topic", 1, cam_subCallback);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    double distance_currentpose_target;

    box_1.pose.position.x = 500;
    box_1.pose.position.y = 114;//y可以随机加减14m
    box_1.pose.position.z = 14.3;
    box_1.pose.orientation = tf::createQuaternionMsgFromYaw(0.00);

    box_2.pose.position.x = 500;
    box_2.pose.position.y = 314;//y可以随机加减14m
    box_2.pose.position.z = 14.3;
    box_2.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting...");
    }
    ROS_INFO("connected");

    // 目的地的local坐标位置
    // geometry_msgs::Pose destination;
    // destination.position.x = 300;
    // destination.position.y = 300;

    mavros_msgs::PositionTarget position_target_maneuvering;
    mavros_msgs::PositionTarget position_target_local;
    position_target_local.position.x = 0;
    position_target_local.position.y = 0;
    position_target_local.position.z = 14.3;
    // position_target_local.yaw = atan2(destination.position.y, destination.position.x);
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
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     local_target_pub.publish(position_target_local);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    vector<double> x = {0.0, 50.0, 150.0, 200.0, 400.0, 500.0, 600.0, 750.0, 800.0, 800.0, 750.0, 600.0, 500.0, 400.0, 200.0, 150.0, 100.0, 0.0};
    vector<double> y = {0.0, 50.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 250.0, 0.0};

    vector<double> tx;  //x坐标
    vector<double> ty;  //y坐标
    vector<double> tyaw;//偏航
    vector<double> tc;  //曲率
    int target_index;          //目标索引

    Spline2D csp = generate_target_course(x, y, tx, ty, tyaw, tc);//规划轨迹

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();


    double distance_drone_box1;//飞机距离框1距离
    double distance_box2_drone;//飞机距离框2距离
    double yaw_now;//需要做机动时的yaw角度
    double yaw_initial;//起飞悬停好时飞机的yaw角度

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
        else 
        {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)) && !armed_flag)
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                    armed_flag = 1;
                }
                last_request = ros::Time::now();
            }
        }

        distance_drone_box1 = sqrt(pow(box_1.pose.position.x - current_pose.pose.position.x, 2) + pow(box_1.pose.position.y - current_pose.pose.position.y, 2) + pow(box_1.pose.position.z - current_pose.pose.position.z, 2));
        distance_box2_drone = sqrt(pow(box_2.pose.position.x - current_pose.pose.position.x, 2) + pow(box_2.pose.position.y - current_pose.pose.position.y, 2) + pow(box_2.pose.position.z - current_pose.pose.position.z, 2));

        if(PX4_current_mission_state == TakeOff)
        {
            position_target_local.position.x = 0;
            position_target_local.position.y = 0;
            position_target_local.position.z = 14.3;

            if(14.3 - current_pose.pose.position.z < 0.2)
            {
                yaw_initial = 0;
                ROS_INFO("Takeoff process is successfully completed. Next is patrol process.");
                PX4_current_mission_state = Patrol;
            }
            
        }
        else if(PX4_current_mission_state == emergency)
        {
            position_target_local.yaw_rate = 0;
            position_target_local.position.x = current_pose.pose.position.x;
            position_target_local.position.y = current_pose.pose.position.y;
            position_target_local.position.z = 0;
            if(current_pose.pose.position.z < 0.3)
            {
               arm_cmd.request.value = false;
               if(current_state.armed)
                {
                    if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle disarmed,game over!!!");
                    }
                }
                else 
                {
                    ROS_INFO("Vehicle disarmed,game over!!!");
                } 
            }
            
        }
        else if(PX4_current_mission_state == Patrol)
        {
            target_index = calc_target_index(current_pose, tx, ty);
            position_target_local.position.x = tx[target_index];
            position_target_local.position.y = ty[target_index];
            position_target_local.yaw = tyaw[target_index];
            position_target_local.yaw_rate = 0;

            if(camera_data.find_obs_flag)
            {
                PX4_current_mission_state = TrackingCam;
            }
            else if(!camera_data.find_obs_flag && cos(abs(Quaternion2Euler(current_pose.pose.orientation).z - yaw_initial)) > 0 && distance_drone_box1 < d_warning && box_1.pose.position.x > current_pose.pose.position.x)
            {
                PX4_current_mission_state = Firstframe_cannotfind;
                yaw_now = Quaternion2Euler(current_pose.pose.orientation).z;
                position_target_maneuvering.position.x = current_pose.pose.position.x;
                position_target_maneuvering.position.y = current_pose.pose.position.y;
                position_target_maneuvering.position.z = 14.3;
                position_target_maneuvering.yaw = yaw_now; 
                position_target_maneuvering.yaw_rate = 0;
                distance_currentpose_target = sqrt(pow(current_pose.pose.position.x - position_target_maneuvering.position.x, 2) + pow(current_pose.pose.position.y - position_target_maneuvering.position.y, 2) + pow(current_pose.pose.position.z - position_target_maneuvering.position.z, 2));
            }
            else if(!camera_data.find_obs_flag && cos(abs(Quaternion2Euler(current_pose.pose.orientation).z - yaw_initial)) < 0 && distance_box2_drone < d_warning && current_pose.pose.position.x > box_2.pose.position.x)
            {
                PX4_current_mission_state = Secondframe_cannotfind;
                yaw_now = Quaternion2Euler(current_pose.pose.orientation).z;
                position_target_maneuvering.position.x = current_pose.pose.position.x;
                position_target_maneuvering.position.y = current_pose.pose.position.y;
                position_target_maneuvering.position.z = 14.3;
                position_target_maneuvering.yaw = yaw_now; 
                position_target_maneuvering.yaw_rate = 0;
                distance_currentpose_target = sqrt(pow(current_pose.pose.position.x - position_target_maneuvering.position.x, 2) + pow(current_pose.pose.position.y - position_target_maneuvering.position.y, 2) + pow(current_pose.pose.position.z - position_target_maneuvering.position.z, 2));
            }
            else if(task_process_flag == 2 && current_pose.pose.position.x < 10 && current_pose.pose.position.y < 10)
            {
                ROS_INFO("We have completed all tasks successfully and return to landing.");
                PX4_current_mission_state = landing;
            }

        }
        else if(PX4_current_mission_state == TrackingCam)
        {
            //旋转矩阵是时刻变化的，所以下面的左右移动都是在当前姿态下的左右移动，tracking_cam_node.cpp中旋转矩阵就是固定的，只相对于设定好的方向左右移动
            Matrix_from_local_to_new << cos(Quaternion2Euler(current_pose.pose.orientation).z), -sin(Quaternion2Euler(current_pose.pose.orientation).z), 0,
                                        sin(Quaternion2Euler(current_pose.pose.orientation).z), cos(Quaternion2Euler(current_pose.pose.orientation).z), 0,
                                        0, 0, 1;
            Eigen::Vector3d temp_pos(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
            Eigen::Vector3d temp_pos_new;
            temp_pos_new = Matrix_from_local_to_new.transpose() * temp_pos;
            current_pose_new.position.x = temp_pos_new(0);
            current_pose_new.position.y = temp_pos_new(1);
            current_pose_new.position.z = temp_pos_new(2);
            
            // position_target_new.position.x = 300;
            // position_target_new.position.y = 0;
            position_target_new.velocity.x = 0;
            if (camera_data.find_obs_flag)
            {
                position_target_new.position.x = current_pose_new.position.x;
                position_target_new.velocity.x = 5;
                position_target_new.position.y = current_pose_new.position.y - 0.02 * (camera_data.x_pos - CAM_CENTER_X);
                position_target_new.position.z = current_pose_new.position.z + 0.01 * (camera_data.y_pos - CAM_CENTER_Z);
            }
            else if(!camera_data.find_obs_flag && (box_1.pose.position.x - current_pose.pose.position.x) > 0 && (box_1.pose.position.x - current_pose.pose.position.x) <= d_min && cos(abs(Quaternion2Euler(current_pose.pose.orientation).z - yaw_initial)) > 0)
            {
                PX4_current_mission_state = Firstframe_acrossing;
            }
            else if(!camera_data.find_obs_flag && (current_pose.pose.position.x - box_2.pose.position.x) > 0 && (current_pose.pose.position.x - box_2.pose.position.x) <= d_min && cos(abs(Quaternion2Euler(current_pose.pose.orientation).z - yaw_initial)) < 0)
            {
                PX4_current_mission_state = Secondframe_acrossing;
            }
            else if(!camera_data.find_obs_flag && cos(abs(Quaternion2Euler(current_pose.pose.orientation).z - yaw_initial)) > 0 && distance_drone_box1 < d_warning && box_1.pose.position.x > current_pose.pose.position.x)
            {
                PX4_current_mission_state = Firstframe_cannotfind;
                yaw_now = Quaternion2Euler(current_pose.pose.orientation).z;
                position_target_maneuvering.position.x = current_pose.pose.position.x;
                position_target_maneuvering.position.y = current_pose.pose.position.y;
                position_target_maneuvering.position.z = 14.3;
                position_target_maneuvering.yaw = yaw_now; 
                position_target_maneuvering.yaw_rate = 0;
                distance_currentpose_target = sqrt(pow(current_pose.pose.position.x - position_target_maneuvering.position.x, 2) + pow(current_pose.pose.position.y - position_target_maneuvering.position.y, 2) + pow(current_pose.pose.position.z - position_target_maneuvering.position.z, 2));
            }
            else if(!camera_data.find_obs_flag && cos(abs(Quaternion2Euler(current_pose.pose.orientation).z - yaw_initial)) < 0 && distance_box2_drone < d_warning && current_pose.pose.position.x > box_2.pose.position.x)
            {
                PX4_current_mission_state = Secondframe_cannotfind;
                yaw_now = Quaternion2Euler(current_pose.pose.orientation).z;
                position_target_maneuvering.position.x = current_pose.pose.position.x;
                position_target_maneuvering.position.y = current_pose.pose.position.y;
                position_target_maneuvering.position.z = 14.3;
                position_target_maneuvering.yaw = yaw_now; 
                position_target_maneuvering.yaw_rate = 0;
                distance_currentpose_target = sqrt(pow(current_pose.pose.position.x - position_target_maneuvering.position.x, 2) + pow(current_pose.pose.position.y - position_target_maneuvering.position.y, 2) + pow(current_pose.pose.position.z - position_target_maneuvering.position.z, 2));
            }
            else
            {   
                PX4_current_mission_state = Patrol;
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
            position_target_local.yaw_rate = 0;
            // position_target_local.velocity.z = temp_vel(2);
            if(cos(abs(Quaternion2Euler(current_pose.pose.orientation).z - yaw_initial)) > 0)
                ROS_INFO("Find the first box and the drone is moving forward towards the center of the first box...");
            if(cos(abs(Quaternion2Euler(current_pose.pose.orientation).z - yaw_initial)) < 0)
                ROS_INFO("Find the second box and the drone is moving forward towards the center of the second box...");
      
        }
        else if(PX4_current_mission_state == Firstframe_acrossing)
        {
            ROS_INFO("acrossing the first box...");
            position_target_local.position.x = current_pose.pose.position.x + 5 * cos(Quaternion2Euler(current_pose.pose.orientation).z);
            position_target_local.position.y = current_pose.pose.position.y + 5 * sin(Quaternion2Euler(current_pose.pose.orientation).z);
            position_target_local.position.z = 14.3;
            position_target_local.velocity.x = 5 * cos(Quaternion2Euler(current_pose.pose.orientation).z);
            position_target_local.velocity.y = 5 * sin(Quaternion2Euler(current_pose.pose.orientation).z);
            position_target_local.yaw_rate = 0;

            if((current_pose.pose.position.x - box_1.pose.position.x) > 3)
            {
                PX4_current_mission_state = Patrol;
                task_process_flag += 1;
                ROS_INFO("successfully acrossed the first box and begin patrol...");
            }    
        }
        else if(PX4_current_mission_state == Secondframe_acrossing)
        {
            ROS_INFO("acrossing the second box...");
            position_target_local.position.x = current_pose.pose.position.x + 5 * cos(Quaternion2Euler(current_pose.pose.orientation).z);
            position_target_local.position.y = current_pose.pose.position.y + 5 * sin(Quaternion2Euler(current_pose.pose.orientation).z);
            position_target_local.position.z = 14.3;
            position_target_local.velocity.x = 5 * cos(Quaternion2Euler(current_pose.pose.orientation).z);
            position_target_local.velocity.y = 5 * sin(Quaternion2Euler(current_pose.pose.orientation).z);
            position_target_local.yaw_rate = 0;
            if((box_2.pose.position.x - current_pose.pose.position.x) > 3)
            {
                PX4_current_mission_state = Patrol;
                task_process_flag += 1;
                ROS_INFO("successfully acrossed the second box and begin patrol...");
            }
        }
        else if(PX4_current_mission_state == Firstframe_cannotfind)
        {
            ROS_INFO("warning: cannot see the box1 abnormally, and start searching box1");
            if(!no_emergency_flag)
            {
                PX4_current_mission_state = emergency;
                ROS_INFO("Error:emergency!!!.We still cannot see the box1 after taking action,so we start to landing on the ground locally.");
            }
            else if(box1_on_the_left)
            {
                if(!camera_data.find_obs_flag && cos(Quaternion2Euler(current_pose.pose.orientation).z) > 0.5 ) //0.5表示最多转60度
                {
                    position_target_local.position.x = current_pose.pose.position.x;
                    position_target_local.position.y = current_pose.pose.position.y;
                    position_target_local.position.z = 14.3;
                    position_target_local.yaw = Quaternion2Euler(current_pose.pose.orientation).z;//在当前yaw角度逆时针转20°（0.349rad对应20°角度）
                    position_target_local.yaw_rate = 0.25;
                    std::cout << "current yaw = " << Quaternion2Euler(current_pose.pose.orientation).z << endl;
                }
                else if(camera_data.find_obs_flag)
                {
                    ROS_INFO("find the box1 on the left, now do some maneuvering to center the box1");
                    PX4_current_mission_state = Maneuvering_after_searching;
                }
                else if(cos(Quaternion2Euler(current_pose.pose.orientation).z) < 0.5)
                {
                    std::cout << "left searching completed!" << endl;
                    position_target_local.position.x = current_pose.pose.position.x;
                    position_target_local.position.y = current_pose.pose.position.y;
                    position_target_local.position.z = 14.3;
                    position_target_local.yaw = yaw_now;
                    position_target_local.yaw_rate = 0;
                    box1_on_the_left = 0;
                }
            }
            else if(!box1_on_the_left)
            {
                std::cout << "Box1 is not on the left, let's search right!" << endl;
                if(!camera_data.find_obs_flag && cos(Quaternion2Euler(current_pose.pose.orientation).z) > 0.4)
                {
                    position_target_local.position.x = current_pose.pose.position.x;
                    position_target_local.position.y = current_pose.pose.position.y;
                    position_target_local.position.z = 14.3;
                    position_target_local.yaw = Quaternion2Euler(current_pose.pose.orientation).z  ;//在当前yaw角度逆时针转20°（0.349rad对应20°角度）
                    position_target_local.yaw_rate = -0.25;
                }
                else if(camera_data.find_obs_flag)
                {
                    ROS_INFO("find the box1 on the right, now do some maneuvering to center the box1");
                    PX4_current_mission_state = Maneuvering_after_searching;
                }
                else if(cos(Quaternion2Euler(current_pose.pose.orientation).z - yaw_now) < 0.4)
                {
                    no_emergency_flag = 0;
                }  
            }
        }
        else if(PX4_current_mission_state == Secondframe_cannotfind)
        {
            ROS_INFO("warning: cannot see the box2 abnormally, and start searching box2");
            if(!no_emergency_flag)
            {
                ROS_INFO("Error:emergency!!!.We still cannot see the box2 after taking action,so we start landing to the ground locally.");
                PX4_current_mission_state = emergency;
            }
            else if(box2_on_the_left)
            {
                if(!camera_data.find_obs_flag && cos(Quaternion2Euler(current_pose.pose.orientation).z) < -0.5)
                {
                    std::cout << "current yaw = " << Quaternion2Euler(current_pose.pose.orientation).z << endl;
                    position_target_local.position.x = current_pose.pose.position.x;
                    position_target_local.position.y = current_pose.pose.position.y;
                    position_target_local.position.z = 14.3;
                    position_target_local.yaw = Quaternion2Euler(current_pose.pose.orientation).z  ;//在当前yaw角度逆时针转20°（0.349rad对应20°角度）
                    position_target_local.yaw_rate = 0.25;
                }
                else if(camera_data.find_obs_flag)
                {
                    ROS_INFO("find the box2 on the left, now do some maneuvering to center the box2");
                    PX4_current_mission_state = Maneuvering_after_searching;
                }
                else if(cos(Quaternion2Euler(current_pose.pose.orientation).z) > -0.5)
                {
                    std::cout << "left searching completed!" << endl;
                    position_target_local.position.x = current_pose.pose.position.x;
                    position_target_local.position.y = current_pose.pose.position.y;
                    position_target_local.position.z = 14.3;
                    position_target_local.yaw = yaw_now;
                    position_target_local.yaw_rate = 0;
                    box2_on_the_left = 0;
                }
            }
            else if(!box2_on_the_left)
            {
                std::cout << "Box2 is not on the left, let's search right!" << endl;
                if(!camera_data.find_obs_flag && cos(Quaternion2Euler(current_pose.pose.orientation).z) < -0.4)
                {
                    std::cout << "current yaw = " << Quaternion2Euler(current_pose.pose.orientation).z << endl;
                    position_target_local.position.x = current_pose.pose.position.x;
                    position_target_local.position.y = current_pose.pose.position.y;
                    position_target_local.position.z = 14.3;
                    position_target_local.yaw = Quaternion2Euler(current_pose.pose.orientation).z  ;//在当前yaw角度逆时针转20°（0.349rad对应20°角度）
                    position_target_local.yaw_rate = -0.25;
                }
                else if(camera_data.find_obs_flag)
                {
                    ROS_INFO("find the box2 on the right, now do some maneuvering to center the box2");
                    PX4_current_mission_state = Maneuvering_after_searching;
                }
                else if(cos(Quaternion2Euler(current_pose.pose.orientation).z) > -0.4)
                {
                    no_emergency_flag = 0;
                }  
            }
        }
        else if(PX4_current_mission_state == Maneuvering_after_searching)
        {
            if(box1_on_the_left || box2_on_the_left)
            {
                position_target_local.position.x = position_target_maneuvering.position.x - 3 * sin(yaw_now);
                position_target_local.position.y = position_target_maneuvering.position.y + 3 * cos(yaw_now);
                position_target_local.position.z = position_target_maneuvering.position.z;
                position_target_local.yaw = position_target_maneuvering.yaw; 
                position_target_local.yaw_rate = position_target_maneuvering.yaw_rate;
            }
            else if(!box1_on_the_left || !box2_on_the_left)
            {
                position_target_local.position.x = position_target_maneuvering.position.x + 3 * sin(yaw_now);
                position_target_local.position.y = position_target_maneuvering.position.y - 3 * cos(yaw_now);
                position_target_local.position.z = position_target_maneuvering.position.z;
                position_target_local.yaw = position_target_maneuvering.yaw; 
                position_target_local.yaw_rate = position_target_maneuvering.yaw_rate;
            }
            if(distance_currentpose_target < 0.2 && abs(Quaternion2Euler(current_pose.pose.orientation).z - position_target_maneuvering.yaw) < 0.1)
            {
                ROS_INFO("manuevering completed.");
                PX4_current_mission_state = TrackingCam;
            }
        }
        else if(PX4_current_mission_state == landing)
        {
            // mavros_msgs::SetMode return_set_mode;
            // return_set_mode.request.custom_mode = "RETURN";
            // last_request = ros::Time::now();
            // if( current_state.mode != "RETURN" &&
            // (ros::Time::now() - last_request > ros::Duration(5.0))){
            // if( set_mode_client.call(return_set_mode) &&
            //     return_set_mode.response.mode_sent){
            //     ROS_INFO("return mode enabled");
                
            // }
            // last_request = ros::Time::now();
            // }
            break;
        }

        //std::cout << current_pose.pose.position.x << " " << current_pose.pose.position.y << " " << current_pose.pose.position.z << std::endl;
        local_target_pub.publish(position_target_local);
        // std::cout << fly_log.is_open() << std::endl;
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}