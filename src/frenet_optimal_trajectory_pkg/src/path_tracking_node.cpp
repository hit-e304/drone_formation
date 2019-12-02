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
// #include <Eigen/LU>
#include "opencvtest/contours.h"










