// 节点功能：单无人机轨迹规划

#include <ros/ros.h>
#include "frenet_optimal_trajectory_pkg/CubicSplinePlanner.h"
#include "frenet_optimal_trajectory_pkg/RangeImpl.h"
#include "frenet_optimal_trajectory_pkg/QuarticPolynomial.h"
#include "frenet_optimal_trajectory_pkg/QuinticPolynomial.h"
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

using namespace CubicSplinePlanner;

double HIGHT = 12.0;
double MAX_SPEED = 50.0 / 3.6; // 最大速度[m/s]
double MAX_ACCEL = 5.0; // 最大加速度[m/ss]
double MAX_CURVATURE = 10.0; // 最大曲率[1/m]
double MAX_ROAD_WIDTH = 7.0; // 最大道路宽度[m]
double D_ROAD_W = 1.0; // 道路宽度方向采样长度，即几米一个格[m]
double DT = 0.2; // 时钟刻度，是预测的局部路径的最小时间单位[s]
double MAXT = 5.0; // 最大预测时间[s]
double MINT = 4.0;  // 最小预测时间[s]
double TARGET_SPEED = 50.0 / 3.6; // 目标速度，无人机要维持的速度[m/s]
double D_T_S = 5.0 / 3.6; // 目标速度采样长度[m/s]
int N_S_SAMPLE = 1; // 目标速度取样个数【个】
double ROBOT_RADIUS = 3.0; // 机器人半径[m]

// cost weights
double KJ = 0.1;
double KT = 0.1;
double KD = 1.0;
double KLAT = 1.0;
double KLON = 1.0;

struct Frenet_path {
    vector<double> t;       //time
    vector<double> d;       // 横向距离
    vector<double> d_d;     // 横向速度
    vector<double> d_dd;    // 横向加速度
    vector<double> d_ddd;   //横向加加速度
    vector<double> s;       //纵向距离
    vector<double> s_d;     //纵向速度
    vector<double> s_dd;    //纵向加速度
    vector<double> s_ddd;   //纵向加加速度
    double cd = 0.0;
    double cv = 0.0;
    double cf = 0.0;
    vector<double> x;
    vector<double> y;
    vector<double> vx;
    vector<double> vy;
    vector<double> yaw; // yaw
    vector<double> ds; // distance
    vector<double> c;// curvature
};

// geometry_msgs::PoseStamped current_pose;
// void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& curr_p)
// {
//     current_pose = *curr_p;
// }

bool check_collision(Frenet_path fp, const MatrixXd& ob){
    vector<double> d;
    bool collision;
    for (int i = 0; i < ob.rows(); ++i) {
        for (int j = 0; j < fp.x.size(); ++j) {
            d.push_back(pow(fp.x[j] - ob(i, 0), 2) + pow(fp.y[j] - ob(i, 1), 2));
        }
        collision = any_of(d.begin(), d.end(), [](double di){return di <= ROBOT_RADIUS * ROBOT_RADIUS;});
        if (collision)
        {
            return false;
        }
    }
    return true;
}

vector<Frenet_path> calc_frenet_paths(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0)
{
    vector<Frenet_path> frenet_paths;

    double Jd;
    double Js;
    double ds;
    vector<double> SquareOfJerks_d_ddd;
    vector<double> SquareOfJerks_s_ddd;
    // 由末端状态的不同生成每一条可能的路径（末端取决于将道路分成了几块）
    for (auto di : Cosmos::Range(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W))
    {
        for (auto Ti : Cosmos::Range(MINT, MAXT, DT)) // Lateral motion planning横向的运动规划，采用五次多项式规划的方式。
        {
            Frenet_path fp;
            QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
            for (auto t : Cosmos::Range(0.0, Ti, DT))
            {
                fp.t.push_back(t);
            }
            for (auto& t : fp.t)
            {
                fp.d.push_back(lat_qp.calc_point(t));
                fp.d_d.push_back(lat_qp.calc_first_derivative(t));
                fp.d_dd.push_back((lat_qp.calc_second_derivative(t)));
                fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
            }

            // 在速度保持模式下，生成纵向运动规划 Longitudinal motion planning (Velocity keeping)纵向运动规划采用的是四次多项式规划。
            for (auto tv : Cosmos::Range(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S))
            {
                Frenet_path tfp;
                tfp = fp;
                QuarticPolynomial lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);

                for (auto& t : fp.t)
                {
                    tfp.s.push_back(lon_qp.calc_point(t));
                    tfp.s_d.push_back(lon_qp.calc_first_derivative(t));
                    tfp.s_dd.push_back(lon_qp.calc_second_derivative(t));
                    tfp.s_ddd.push_back(lon_qp.calc_third_derivative(t));
                }
                for (int i = 0; i < tfp.d_ddd.size(); i++) {
                    SquareOfJerks_d_ddd.push_back(pow(tfp.d_ddd[i], 2));
                    SquareOfJerks_s_ddd.push_back(pow(tfp.s_ddd[i], 2));
                }

                Jd = accumulate(SquareOfJerks_d_ddd.begin() , SquareOfJerks_d_ddd.end() , 0.00); //横向加速度突变对路径cost的影响
                Js = accumulate(SquareOfJerks_s_ddd.begin() , SquareOfJerks_s_ddd.end() , 0.00); //纵向加速度突变对路径cost的影响
                ds = (TARGET_SPEED - tfp.s_d.back()) * (TARGET_SPEED - tfp.s_d.back());
                tfp.cd = KJ * Jd + KT * Ti + KD * tfp.d.back() * tfp.d.back();
                tfp.cv = KJ * Js + KT * Ti + KD * ds;
                tfp.cf = KLAT * tfp.cd + KLON * tfp.cv;
                SquareOfJerks_d_ddd.clear();
                SquareOfJerks_s_ddd.clear();
                frenet_paths.push_back(tfp);
            }
        }
    }

    return frenet_paths;
}

void calc_global_paths(vector<Frenet_path>& fplist, Spline2D& csp)
{
    vector<double> ixy;
    double iyaw;
    double fx;
    double fy;
    double dx;
    double dy;
    for (auto& fp : fplist)
    {
        // 计算全局路径点calculate global position
        for (int i = 0; i < fp.s.size(); ++i)
        {
            ixy = csp.calc_position(fp.s[i]);
            if (ixy.empty()) break;
            iyaw = csp.calc_yaw(fp.s[i]);
            fx = ixy[0] + fp.d[i] * cos(iyaw + M_PI_2);
            fy = ixy[1] + fp.d[i] * sin(iyaw + M_PI_2);
            fp.x.push_back(fx);
            fp.y.push_back(fy);
        }

        // 计算偏航角和每步走的距离calculate yaw and ds
        for (int i = 0; i < fp.x.size() - 1; ++i)
        {
            dx = fp.x[i + 1] - fp.x[i];
            dy = fp.y[i + 1] - fp.y[i];
            fp.vx.push_back(dx / DT);
            fp.vy.push_back(dy / DT);
            fp.yaw.push_back(atan2(dy, dx));
            fp.ds.push_back(sqrt(dx * dx + dy * dy));
        }
        fp.vx.push_back(fp.vx.back());
        fp.vy.push_back(fp.vy.back());
        fp.yaw.push_back(fp.yaw.back());
        fp.ds.push_back(fp.ds.back());

        // 计算轨迹曲率calculate curvature
        for (int i = 0; i < fp.yaw.size() - 1; ++i) {
            fp.c.push_back((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i]);
        }
    }
}

vector<Frenet_path> check_paths(vector<Frenet_path> fplist, const MatrixXd& ob)
{
    vector<int> okind;
    vector<Frenet_path> collision_avoid_paths;
    for (int i = 0; i < fplist.size(); ++i) {
        if (any_of(fplist[i].s_d.begin(), fplist[i].s_d.end(), [](double v){return v > MAX_SPEED;})){
            continue;
        } else if (any_of(fplist[i].s_dd.begin(), fplist[i].s_dd.end(), [](double a){return abs(a) > MAX_ACCEL;})){
            continue;
        } else if (any_of(fplist[i].c.begin(), fplist[i].c.end(), [](double c){return abs(c) > MAX_CURVATURE;})){
            continue;
        } else if(!check_collision(fplist[i], ob)){
            continue;
        }
        okind.push_back(i);
    }
    for (auto i : okind)
    {
        collision_avoid_paths.push_back(fplist[i]);
    }
    return collision_avoid_paths;
}

Frenet_path
frenet_optimal_planning(Spline2D& csp, const double s0, const double c_speed, const double c_d, const double c_d_d,
                        const double c_d_dd, const MatrixXd& ob) {
    auto mincost = DBL_MAX;
    Frenet_path best_path; // 没有初始化，如果出现问题了记得查一下
    vector<Frenet_path> collision_avoid_paths;
    vector<Frenet_path> fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
    calc_global_paths(fplist, csp);
    collision_avoid_paths = check_paths(fplist, ob);

    for (auto& fp : collision_avoid_paths)
    {
        if (mincost >= fp.cf) // 找到cost最小的路径赋值给best_path
        {
            mincost = fp.cf;
            best_path = fp; // C语言中结构体不能直接赋值，但是C++可以
        }
    }
    return best_path;
}

Spline2D
generate_target_course(vector<double> x, vector<double> y, vector<double> &rx, vector<double> &ry, vector<double> &ryaw,
                       vector<double> &rc) {
    Spline2D csp(x, y);
    vector<double> ixy;
    // Cosmos::RangeImpl<int> s = Cosmos::Range(0, int(csp.s.back()), 5);
    for (auto i_s : Cosmos::Range(0, int(csp.s.back()), 5)) {
        ixy = csp.calc_position(i_s);
        rx.push_back(ixy[0]);
        ry.push_back(ixy[1]);
        ryaw.push_back(csp.calc_yaw(i_s));
        rc.push_back(csp.calc_curvature(i_s));
    }
    return csp;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sim_planning_path");
    ros::NodeHandle nh("~");
    ros::Publisher sim_global_path_vis_pub = nh.advertise<visualization_msgs::Marker>("global_way_point", 1);
    ros::Publisher sim_local_path_vis_pub = nh.advertise<visualization_msgs::Marker>("local_way_point", 1);

    // ros::Subscriber local_position_sub = nh.subscribe("/mavros/local_position/pose", 1, local_pos_cb);
    

    vector<double> x = {-0.2, 305.5};
    vector<double> y = {-0.2, 0};
    MatrixXd ob(6, 2);
    ob << 20.0, 0.0,
          75.0, 1.0,
          125.0, -0.5,
          175.0, 0.0,
          225.0, 0.0,
          260, 1.5;

    vector<double> tx;
    vector<double> ty;
    vector<double> tyaw;
    vector<double> tc;
    Frenet_path path;
    Spline2D csp = generate_target_course(x, y, tx, ty, tyaw, tc);

    // double min_d = DBL_MAX;
    // double dis;
    // double s;
    // int index;
    // for (int i = 0; i < csp.s.size(); i++)
    // {
    //     dis = sqrt(pow(current_pose.pose.position.x - tx[i], 2) + pow(current_pose.pose.position.y - ty[i], 2));
    //     if(min_d > dis)
    //     {
    //         min_d = dis;
    //         // s = csp.s[i];
    //         index = i;
    //     }
    // }
    // s = csp.s[index];
    

    double c_speed = 10.0 / 3.6;//当前的速度，frenet坐标系下的纵向速度
    double c_d = 2.0;//当前的横向位置
    double c_d_d = 0.0;//当前的横向速度
    double c_d_dd = 0.0;//当前的横向加速度
    double s0 = 0.0;//当前的纵向位置

    visualization_msgs::Marker sim_global_path_marker;
    sim_global_path_marker.id = 10;
    sim_global_path_marker.header.frame_id = "local_origin";
    sim_global_path_marker.header.stamp = ros::Time();
    sim_global_path_marker.ns = "global_path";
    sim_global_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    sim_global_path_marker.action = visualization_msgs::Marker::ADD;
    sim_global_path_marker.scale.x = 0.1;
    sim_global_path_marker.scale.y = 0.1;
    sim_global_path_marker.color.a = 0.9;
    sim_global_path_marker.color.r = 1.0;
    sim_global_path_marker.color.g = 0.0;
    sim_global_path_marker.color.b = 0.0;

    vector<double> xy;
    geometry_msgs::Point gwp;//中间变量
    for (auto i_s : Cosmos::Range(0, int(csp.s.back()), 0.1))
    {
        xy = csp.calc_position(i_s);
        gwp.x = xy[0];
        gwp.y = xy[1];
        gwp.z = HIGHT / 2;
        // cout << "x = " << xy[0] << ", y = " << xy[1] << endl;
        sim_global_path_marker.points.push_back(gwp);
    }

    visualization_msgs::Marker sim_local_path_marker;
    sim_local_path_marker.id = 11;
    sim_local_path_marker.header.frame_id = "local_origin";
    sim_local_path_marker.header.stamp = ros::Time();
    sim_local_path_marker.ns = "local_path";
    sim_local_path_marker.type = visualization_msgs::Marker::POINTS;
    sim_local_path_marker.action = visualization_msgs::Marker::ADD;
    sim_local_path_marker.scale.x = 0.5;
    sim_local_path_marker.scale.y = 0.5;
    sim_local_path_marker.color.a = 0.9;
    sim_local_path_marker.color.r = 1.0;
    sim_local_path_marker.color.g = 1.0;
    sim_local_path_marker.color.b = 0.0;
    geometry_msgs::Point lwp; //中间变量

    ros::Rate rate(10);

    while (ros::ok())
    {
        path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob);

        s0 = path.s[1];
        c_d = path.d[1];
        c_d_d = path.d_d[1];
        c_d_dd = path.d_dd[1];
        c_speed = path.s_d[1];
        cout << "x = " << path.x[1] << " y = " << path.y[1] << " speed: " << c_speed << endl;
        for (int i = 1; i < path.x.size(); i++)
        {
            lwp.x = path.x[i];
            lwp.y = path.y[i];
            lwp.z = HIGHT / 2;
            sim_local_path_marker.points.push_back(lwp);
        }
        
        if (hypot(path.x[1] - tx.back(), path.y[1] - ty.back()) <= 1.0)
        {
            cout << "Goal" << endl;
            break;
        }

        sim_global_path_vis_pub.publish(sim_global_path_marker);
        sim_local_path_vis_pub.publish(sim_local_path_marker);
        ros::spinOnce();
        rate.sleep();
        sim_local_path_marker.points.clear();
    }
    
    
    return 0;
}