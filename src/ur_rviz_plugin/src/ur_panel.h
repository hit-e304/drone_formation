#ifndef TELEOP_PAD_H
#define TELEOP_PAD_H


//所需要包含的头文件
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件


#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <mavros_msgs/State.h>
// #include <sensor_msgs/NavSatFix.h>         //GPS_fix    ---rosmsg show sensor_msgs/NavSatFix
#include <sensor_msgs/NavSatStatus.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h> 
//#include <ur_controller_action/multi_grasptarget_box.h>


// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>


// Opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf/transform_broadcaster.h>
typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient; 

class QLineEdit;
class QLabel;
class QPushButton;
class QTextBrowser;
class QToolBox;






namespace ur_rviz_plugin
{
// 所有的plugin都必须是rviz::Panel的子类
class PX4_Panel: public rviz::Panel
{
// 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
Q_OBJECT
public:
  // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
  PX4_Panel( QWidget* parent = 0 );

  void publish_speedj_(int joint_index , int d_or_i);
  void publish_speedl_(int dimension , int d_or_i);
  bool ReadMatrix(std::string FileName, Eigen::Matrix4d& transMat);
  Eigen::Matrix4d transRosMsgPose2EigenMatrix(geometry_msgs::Pose pose);
  Eigen::Matrix4d transRosMsgTransform2EigenMatrix(geometry_msgs::Transform transform);
  geometry_msgs::Pose transEigenMatrix2RosMsgPose(Eigen::Matrix4d matrix);



  // 重载rviz::Panel积累中的函数，用于保存、加载配置文件中的数据，在我们这个plugin中，数据就是topic的名称
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;
  



      //public slots：在这个区内声明的槽意味着任何对象都可将信号与之相连接。这对于组件编程非常有用，你可以创建彼此互不了解的对象，将它们的信号与槽进行连接以便信息能够正确的传递。
      //protected slots：在这个区内声明的槽意味着当前类及其子类可以将信号与之相连接。这适用于那些槽，它们是类实现的一部分，但是其界面接口却面向外部。
      //private slots：在这个区内声明的槽意味着只有类自己可以将信号与之相连接。这适用于联系非常紧密的类。


  // 公共槽.
public Q_SLOTS:
  // 当用户输入topic的命名并按下回车后，回调用此槽来创建一个相应名称的topic publisher
  void setTopic( const QString& topic );
    
  // 内部槽.
protected Q_SLOTS:
  void sendVel();                 // 发布当前的速度值
  void update_Linear_Velocity();  // 根据用户的输入更新线速度值
  void update_Angular_Velocity(); // 根据用户的输入更新角速度值
  void updateTopic();             // 根据用户的输入更新topic name
  void RobotSTOP_clicked();       // 机器人急停键
  void control_robot_movej();     // movej 模式
  void track_aruco_enable();      //track aruco 
  void switch_radian_degree();    // 切换显示关节信息的单位
  void updata_plane_state();        // 更新飞机的状态信息，每100ms触发一次
  void updata_movej_editors();    // 把当天的关节信息复制到movej的editors里
  void take_off_result();        // take_off的结果
  void autoScroll();               //光标到最后
  void update_recogniton_result();  //锁定识别结果
  void shut_down_result();     //发送识别结果，call抓取service

  void control_robot_speedj_1();   // jog joint响应按钮
  void control_robot_speedj_2();   // jog joint响应按钮
  void control_robot_speedj_3();   // jog joint响应按钮
  void control_robot_speedj_4();   // jog joint响应按钮
  void control_robot_speedj_5();   // jog joint响应按钮
  void control_robot_speedj_6();   // jog joint响应按钮
  void control_robot_speedj_7();   // jog joint响应按钮
  void control_robot_speedj_8();   // jog joint响应按钮
  void control_robot_speedj_9();   // jog joint响应按钮
  void control_robot_speedj_10();   // jog joint响应按钮
  void control_robot_speedj_11();   // jog joint响应按钮
  void control_robot_speedj_12();   // jog joint响应按钮

  void control_robot_speedl_1();   // robot speedl mode 响应按钮
  void control_robot_speedl_2();   // robot speedl mode 响应按钮
  void control_robot_speedl_3();   // robot speedl mode 响应按钮
  void control_robot_speedl_4();   // robot speedl mode 响应按钮
  void control_robot_speedl_5();   // robot speedl mode 响应按钮
  void control_robot_speedl_6();   // robot speedl mode 响应按钮

 
  // 内部变量.
protected:

  //一些按钮
  QPushButton* push_button_RobotSTOP;
  QPushButton* push_button_movej;
  QPushButton* push_button_track_aruco_Mode;     //右边那三个
  QPushButton* push_button_4;
  QPushButton* push_button_5;

  // topic name输入框,控制UR的topic
  QLineEdit* output_topic_editor_;
  QString output_topic_;
  
  // 线速度值输入框
  QLineEdit* output_topic_editor_1;
  QString output_topic_1;
  
  // 角速度值输入框
  QLineEdit* output_topic_editor_2;
  QString output_topic_2;
  
  //capability 中的部件
  QPushButton* push_button_joint_control_jog_[6*2];     //框里面那12个控件
  
  float joint_speedj_speed_;
  QPushButton* push_button_speedl_[6*2];
 
  QPushButton* push_button_recognition_;
  QPushButton* push_button_take_off_;
  QPushButton* push_button_call_service_;

  QTextBrowser* text_browser_recogniton_result_;



  // 关节信息显示
  //QLabel* joint_states_label_[6];   
  QPushButton* push_button_radian_degree_switch_;
  bool jointstates_use_radian_FLAG_;
  QLineEdit* plane_state_msgs[6];
  QPushButton* push_button_updata_movej_editors_;

  //ROS service client
  //ur_controller_action::multi_grasptarget_box multi_grasptarget_srv_;
  ros::ServiceClient execute_grasp_service_client_;

  //ROS action 
  
  TrajClient* TrajClient_;

  /****************** ROS publisher ******************/ 

  ros::Publisher velocity_publisher_;

  // ROS的publisher，发布机器人期望位置，gazebo:/arm_controller/command
  //                                 hardware: /movej
  ros::Publisher control_robot_position_movej_publisher_; 
  std::vector<float> joint_positions_movej_;
  ros::Publisher control_robot_speedj_publisher_;
  ros::Publisher control_robot_speddl_publisher_;
  ros::Publisher track_aruco_publisher_;
  bool track_aruco_flag_ ; 

  //ROS subscriber订阅需要的消息
  ros::Subscriber robot_state_subscriber_;
  virtual void robot_state_callback(const sensor_msgs::JointStateConstPtr& state);

  ros::Subscriber recogtion_result_subscriber_;
  ros::Subscriber state_sub,GPS_sub;
  virtual void recognition_result_callback(const geometry_msgs::PoseStampedPtr msg);
  geometry_msgs::Pose target_pose_in_robot_base_;
  // The ROS node handle.
  ros::NodeHandle nh_;

  // 当前保存的线速度和角速度值
  float linear_velocity_;
  float angular_velocity_;
  std::vector<float> joint_states_;

  Eigen::Matrix4d d435i_2_wrist2link_;

  tf2_ros::Buffer tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
};

} // end namespace ur_rviz_plugin

#endif // TELEOP_PANEL_H
