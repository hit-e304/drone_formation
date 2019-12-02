// 标签显示的可以直接在使用的时候new
// 功能类的插件在头文件中声明

#include <stdio.h>
#include <string>
#include <time.h>
#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextBrowser>
#include <QLabel>
#include <QTabWidget>
#include <QSpacerItem>
#include <QTimer>
#include <QProcess>

#include <geometry_msgs/Twist.h>
#include <QDebug>

#include "ur_panel.h"


#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Int16.h>
using namespace std;


/*
std_msgs/Header header
bool connected
bool armed
bool guided
string mode
uint8 system_status
*/

mavros_msgs::State current_state;    //无人机的状态
sensor_msgs::NavSatStatus GPS_fix_state;   //GPS搜索数量
QString output_msgs;
QProcess *take_off_Process = new QProcess;

void state_sub_fuction(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
void GPS_fix_sub(const sensor_msgs::NavSatStatus::ConstPtr& msg)
{
    GPS_fix_state = *msg;
}
namespace ur_rviz_plugin
{

// 构造函数，初始化变量
PX4_Panel::PX4_Panel( QWidget* parent )
  : rviz::Panel( parent )
  , joint_positions_movej_(6,0.0) 
  , joint_states_(6,0.0) 
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
  , jointstates_use_radian_FLAG_(true)
  , joint_speedj_speed_(0.0)
  , track_aruco_flag_(false)
{
  if(!ReadMatrix("ExperimentData/D435i_in_UR_calibration/Eye_in_Hand.yaml",d435i_2_wrist2link_))
  {
        std::cout<< " Failed to read the transformation matrxi from d435i color optical frame to wrist 2 link" << std::endl;
        d435i_2_wrist2link_ <<9.9993570373921825e-01, -2.4903423590655338e-04,
                                1.1336947097973678e-02, -8.2316597721692183e-02,
                                -1.1336658095765673e-02, 1.1327690762379938e-03,
                                9.9993509640247935e-01, -7.5402015681807544e-02,
                                -2.6186021578067376e-04, -9.9999932740785691e-01,
                                1.1298730274386451e-03, 2.7008370053956099e-01, 0., 0., 0., 1.;
  }
  
   
  /*ROS action*/
  TrajClient_ = new TrajClient("/arm_controller/follow_joint_trajectory", true);

  /*ROS publisher*/
  control_robot_position_movej_publisher_ = nh_.advertise<trajectory_msgs::JointTrajectory>( "/ur_driver/joint_movej", 10 );
  control_robot_speedj_publisher_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed", 10);
  control_robot_speddl_publisher_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speedl", 10);
  track_aruco_publisher_ = nh_.advertise<std_msgs::Int16>("/ur_track_aruco", 10);




  
  
  //  

  /*capability tab控件*/                              //中间的框
  QHBoxLayout* capbility_layout = new QHBoxLayout;   //竖直(Vertical)排列部件的layout层
  QTabWidget* capbility_tag_widget = new QTabWidget(this);
  capbility_tag_widget->setSizePolicy(QSizePolicy::Maximum,QSizePolicy::Fixed);
  capbility_tag_widget->setMinimumWidth(100);
  capbility_tag_widget->setMaximumWidth(300);
  
  QWidget *tab_recognition=new QWidget(this);
  capbility_tag_widget->addTab(tab_recognition,"recognition");
  QWidget *tab_control_robot=new QWidget(this);
  capbility_tag_widget->addTab(tab_control_robot,"robot control");

  //tab_control_robot
  QGridLayout* robot_control_gridlayout = new QGridLayout(tab_control_robot);        //中间的网格布局
  robot_control_gridlayout->setHorizontalSpacing(10);
  for(int i=0;i<12;i++)
  {
     push_button_joint_control_jog_[i] = new QPushButton(i%2==0?"J+":"J-");
     push_button_joint_control_jog_[i]->setAutoRepeat(true);
     push_button_joint_control_jog_[i]->setAutoRepeatDelay(8);
     push_button_joint_control_jog_[i]->setSizePolicy(QSizePolicy::Maximum,QSizePolicy::Fixed);
     push_button_joint_control_jog_[i]->setMinimumWidth(10);
     push_button_joint_control_jog_[i]->setMaximumWidth(40);
     robot_control_gridlayout->addWidget(push_button_joint_control_jog_[i],i/2,i%2+1,1,1);
  }

  QLabel* jog_label[6];
  jog_label[0] = new QLabel("<u>J</u>1:");
  jog_label[1] = new QLabel("<u>J</u>2:");
  jog_label[2] = new QLabel("<u>J</u>3:");
  jog_label[3] = new QLabel("<u>J</u>4:");
  jog_label[4] = new QLabel("<u>J</u>5:");
  jog_label[5] = new QLabel("<u>J</u>6:");
  for( int i=0;i<6 ;i++)
  {
      jog_label[i]->setSizePolicy(QSizePolicy::Maximum,QSizePolicy::Fixed);
      jog_label[i]->setMinimumWidth(10);
      jog_label[i]->setMinimumWidth(20);
      robot_control_gridlayout->addWidget(jog_label[i],i,0,1,1);
  }
 
  connect(push_button_joint_control_jog_[0],SIGNAL(pressed()),this,SLOT(control_robot_speedj_1() ));
  connect(push_button_joint_control_jog_[1],SIGNAL(pressed()),this,SLOT(control_robot_speedj_2() ));
  connect(push_button_joint_control_jog_[2],SIGNAL(pressed()),this,SLOT(control_robot_speedj_3() ));
  connect(push_button_joint_control_jog_[3],SIGNAL(pressed()),this,SLOT(control_robot_speedj_4() ));
  connect(push_button_joint_control_jog_[4],SIGNAL(pressed()),this,SLOT(control_robot_speedj_5() ));
  connect(push_button_joint_control_jog_[5],SIGNAL(pressed()),this,SLOT(control_robot_speedj_6() ));
  connect(push_button_joint_control_jog_[6],SIGNAL(pressed()),this,SLOT(control_robot_speedj_7() ));
  connect(push_button_joint_control_jog_[7],SIGNAL(pressed()),this,SLOT(control_robot_speedj_8() ));
  connect(push_button_joint_control_jog_[8],SIGNAL(pressed()),this,SLOT(control_robot_speedj_9() ));
  connect(push_button_joint_control_jog_[9],SIGNAL(pressed()),this,SLOT(control_robot_speedj_10() ));
  connect(push_button_joint_control_jog_[10],SIGNAL(pressed()),this,SLOT(control_robot_speedj_11() ));
  connect(push_button_joint_control_jog_[11],SIGNAL(pressed()),this,SLOT(control_robot_speedj_12() ));

  push_button_speedl_[0] = new QPushButton("X+");
  push_button_speedl_[0]->setSizePolicy(QSizePolicy::Maximum,QSizePolicy::Fixed);
  push_button_speedl_[0]->setMinimumWidth(10);
  push_button_speedl_[0]->setMaximumWidth(40);  robot_control_gridlayout->addWidget(push_button_speedl_[0],0,4,1,1);
  push_button_speedl_[1] = new QPushButton("X-");
  push_button_speedl_[1]->setSizePolicy(QSizePolicy::Maximum,QSizePolicy::Fixed);
  push_button_speedl_[1]->setMinimumWidth(10);
  push_button_speedl_[1]->setMaximumWidth(40);  robot_control_gridlayout->addWidget(push_button_speedl_[1],0,5,1,1);
  push_button_speedl_[2] = new QPushButton("Y+");
  push_button_speedl_[2]->setSizePolicy(QSizePolicy::Maximum,QSizePolicy::Fixed);
  push_button_speedl_[2]->setMinimumWidth(10);
  push_button_speedl_[2]->setMaximumWidth(40);  robot_control_gridlayout->addWidget(push_button_speedl_[2],1,4,1,1);
  push_button_speedl_[3] = new QPushButton("Y-");
  push_button_speedl_[3]->setSizePolicy(QSizePolicy::Maximum,QSizePolicy::Fixed);
  push_button_speedl_[3]->setMinimumWidth(10);
  push_button_speedl_[3]->setMaximumWidth(40);  robot_control_gridlayout->addWidget(push_button_speedl_[3],1,5,1,1); 
  push_button_speedl_[4] = new QPushButton("Z+");
  push_button_speedl_[4]->setSizePolicy(QSizePolicy::Maximum,QSizePolicy::Fixed);
  push_button_speedl_[4]->setMinimumWidth(10);
  push_button_speedl_[4]->setMaximumWidth(40);  robot_control_gridlayout->addWidget(push_button_speedl_[4],2,4,1,1);
  push_button_speedl_[5] = new QPushButton("Z-");
  push_button_speedl_[5]->setSizePolicy(QSizePolicy::Maximum,QSizePolicy::Fixed);
  push_button_speedl_[5]->setMinimumWidth(10);
  push_button_speedl_[5]->setMaximumWidth(40);  robot_control_gridlayout->addWidget(push_button_speedl_[5],2,5,1,1);
  for(int i=0;i<6;i++)
  {
     push_button_speedl_[i]->setAutoRepeat(true);
     push_button_speedl_[i]->setAutoRepeatDelay(8);
  }
  connect(push_button_speedl_[0],SIGNAL(pressed()),this,SLOT(control_robot_speedl_1() ));
  connect(push_button_speedl_[1],SIGNAL(pressed()),this,SLOT(control_robot_speedl_2() ));
  connect(push_button_speedl_[2],SIGNAL(pressed()),this,SLOT(control_robot_speedl_3() ));
  connect(push_button_speedl_[3],SIGNAL(pressed()),this,SLOT(control_robot_speedl_4() ));
  connect(push_button_speedl_[4],SIGNAL(pressed()),this,SLOT(control_robot_speedl_5() ));
  connect(push_button_speedl_[5],SIGNAL(pressed()),this,SLOT(control_robot_speedl_6() ));

  
  tab_control_robot->setLayout(robot_control_gridlayout);

    //tab_recognition
  QGridLayout* recogniton_gridlayout = new QGridLayout(tab_recognition);

  push_button_recognition_ = new QPushButton("Recognition");
  push_button_recognition_->setStyleSheet("background-color:rgb(255,255,255)");
  connect(push_button_recognition_,SIGNAL(clicked()),this,SLOT(update_recogniton_result() ));
  recogniton_gridlayout->addWidget(push_button_recognition_,0,0,1,2,Qt::AlignRight | Qt::AlignVCenter);
  

  push_button_take_off_ = new QPushButton("Take_off");
  push_button_take_off_->setStyleSheet("background-color:rgb(255,255,255)");
  connect(push_button_take_off_,SIGNAL(clicked()),this,SLOT(take_off_result() ));
  recogniton_gridlayout->addWidget(push_button_take_off_,0,2,1,2,Qt::AlignRight | Qt::AlignVCenter);
  

  push_button_call_service_ = new QPushButton("shut_down");
  push_button_call_service_->setStyleSheet("background-color:rgb(255,255,255)");
  push_button_call_service_->setEnabled(false);
  connect(push_button_call_service_,SIGNAL(clicked()),this,SLOT(shut_down_result() ));
  recogniton_gridlayout->addWidget(push_button_call_service_,0,4,1,2,Qt::AlignRight | Qt::AlignVCenter);
  
  text_browser_recogniton_result_ = new QTextBrowser;
  recogniton_gridlayout->addWidget(text_browser_recogniton_result_,1,0,3,6);        //四个数，左上角的点及框的大小 左上y-左上x-纵向长度-
  connect(text_browser_recogniton_result_, SIGNAL(cursorPositionChanged()), this, SLOT(autoScroll()));
  

  tab_recognition->setLayout(recogniton_gridlayout);

  capbility_layout->addWidget(capbility_tag_widget); 
 
 




  
  /*左侧状态信息层*/                                          //最左边的框
  QGridLayout *PX4_state_gridlayout_n_2 = new QGridLayout();


 


//   push_button_radian_degree_switch_ = new QPushButton("&R/D");
//   push_button_radian_degree_switch_->setFixedWidth(60);
//   PX4_state_gridlayout_n_2->addWidget(push_button_radian_degree_switch_,0,1,1,1);
//   connect(push_button_radian_degree_switch_,SIGNAL(clicked()),this,SLOT( switch_radian_degree() )  );

  /*movej 按钮*/
  push_button_updata_movej_editors_ = new QPushButton("&Update") ;
  push_button_updata_movej_editors_->setFixedWidth(60);
  PX4_state_gridlayout_n_2->addWidget( push_button_updata_movej_editors_,0,1,1,1,Qt::AlignRight | Qt::AlignVCenter );
  connect(push_button_updata_movej_editors_,SIGNAL(clicked()),this,SLOT( updata_movej_editors() )  );
  
  PX4_state_gridlayout_n_2->addWidget(new QLabel("Parameters"),0,0);
  PX4_state_gridlayout_n_2->addWidget(new QLabel("connected:"),1,0);
  PX4_state_gridlayout_n_2->addWidget(new QLabel("armed:"),2,0);
  PX4_state_gridlayout_n_2->addWidget(new QLabel("guided:"),3,0);
  PX4_state_gridlayout_n_2->addWidget(new QLabel("GPS_Ready:"),4,0);
  PX4_state_gridlayout_n_2->addWidget(new QLabel("mode:"),5,0);
  PX4_state_gridlayout_n_2->addWidget(new QLabel("system_status:"),6,0);
  



  //无人机的状态显示及更新
  for(int i=0;i<6;i++)
  {
    //   joint_states_label_[i-1] = new QLabel;
    //   PX4_state_gridlayout_n_2->addWidget(joint_states_label_[i-1],i,1,1,1,Qt::AlignRight | Qt::AlignVCenter);
    //   joint_states_label_[i-1]->setFixedWidth(60);
      plane_state_msgs[i] = new QLineEdit;
      PX4_state_gridlayout_n_2->addWidget(plane_state_msgs[i],i+1,1,1,1,Qt::AlignRight | Qt::AlignVCenter);
    //   connect(plane_state_msgs[i-1],SIGNAL(editingFinished()),this,SLOT(updata_plane_state()));
      plane_state_msgs[i]->setFixedWidth(60);
  }
  // 设置定时器的回调函数，按周期调用sendVel()
  QTimer* plane_state_timer = new QTimer( this );
  // 设置信号与槽的连接
  connect( plane_state_timer, SIGNAL(timeout()),this,SLOT( updata_plane_state() ));
  // 设置定时器的周期，100ms
  plane_state_timer->start(100);








  /*****按钮层*****/
  QVBoxLayout* pushbutton_layout_1x1 = new QVBoxLayout;          //最右边那三个按钮

  /*一些按钮*/
  //急停键
  push_button_RobotSTOP = new QPushButton("&STOP") ;
  push_button_RobotSTOP->setSizePolicy(QSizePolicy::Maximum,QSizePolicy::Fixed);
  push_button_RobotSTOP->setMinimumWidth(10);
  push_button_RobotSTOP->setMaximumWidth(40);
  push_button_RobotSTOP->setStyleSheet("background-color:rgb(255,0,0)");
  connect(push_button_RobotSTOP,SIGNAL(clicked()),this,SLOT( RobotSTOP_clicked() )  );
  // push_button_RobotSTOP->setStyleSheet("color:red");
  pushbutton_layout_1x1->addWidget( push_button_RobotSTOP );
 
  push_button_movej = new QPushButton("&Movej") ;
  push_button_movej->setEnabled(false);       //设置是否使能
  push_button_movej->setSizePolicy(QSizePolicy::Maximum,QSizePolicy::Fixed);
  push_button_movej->setMinimumWidth(10);
  push_button_movej->setMaximumWidth(60);
  connect(push_button_movej,SIGNAL(clicked()),this,SLOT( control_robot_movej() )  );
  pushbutton_layout_1x1->addWidget( push_button_movej );

  push_button_track_aruco_Mode = new QPushButton("Track") ;
  push_button_track_aruco_Mode->setStyleSheet("background-color:rgb(255,0,0)");
  push_button_track_aruco_Mode->setSizePolicy(QSizePolicy::Maximum,QSizePolicy::Fixed);
  push_button_track_aruco_Mode->setMinimumWidth(10);
  push_button_track_aruco_Mode->setMaximumWidth(60);
  connect(push_button_track_aruco_Mode,SIGNAL(clicked()),this,SLOT( track_aruco_enable() )  );
  pushbutton_layout_1x1->addWidget( push_button_track_aruco_Mode );


//   push_button_4 = new QPushButton("&4D") ;
//   pushbutton_layout_1x1->addWidget( push_button_4 );
//   push_button_5 = new QPushButton("&5D") ;
//   pushbutton_layout_1x1->addWidget( push_button_5 );







  QHBoxLayout* horizontal_layout = new QHBoxLayout;//水平(Horizontal)排列部件的layout层
  horizontal_layout->addLayout( PX4_state_gridlayout_n_2 );
  //horizontal_layout->addStretch();
  horizontal_layout->addLayout( capbility_layout );
  //horizontal_layout->addStretch();
  horizontal_layout->addLayout( pushbutton_layout_1x1 );
  horizontal_layout->addStretch();

  setLayout( horizontal_layout );
 

  // 创建一个定时器，用来定时发布消息
  QTimer* output_timer = new QTimer( this );

  // 设置信号与槽的连接
    


  // 设置定时器的回调函数，按周期调用sendVel()
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  // 设置定时器的周期，100ms
  output_timer->start( 100 );

  //订阅消息，在定时器中进入回调函数
  robot_state_subscriber_ = nh_.subscribe("joint_states", 1000, &PX4_Panel::robot_state_callback, this);
  recogtion_result_subscriber_ = nh_.subscribe("/aruco_single/pose", 1000, &PX4_Panel::recognition_result_callback, this);
  //state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, state_sub);
  state_sub = nh_.subscribe("mavros/state",10,state_sub_fuction);
  //  mavros/global_position/raw/fix   -----GPS_fix话题
  GPS_sub = nh_.subscribe("mavros/global_position/raw/fix", 10, GPS_fix_sub);
  //service client 
  //execute_grasp_service_client_ = nh_.serviceClient<ur_controller_action::multi_grasptarget_box>("/grasp_target_receiver");

  tfListener_.reset( new tf2_ros::TransformListener(tfBuffer_) ) ;
 
}

// 更新线速度值
void PX4_Panel::update_Linear_Velocity()
{
    // 获取输入框内的数据
    QString temp_string = output_topic_editor_1->text();
	
	// 将字符串转换成浮点数
    float lin = temp_string.toFloat();  
	
	// 保存当前的输入值
    linear_velocity_ = lin;
}

// 更新角速度值
void PX4_Panel::update_Angular_Velocity()
{
    QString temp_string = output_topic_editor_2->text();
    float ang = temp_string.toFloat();  
    angular_velocity_ = ang;
}

// 更新topic命名
void PX4_Panel::updateTopic()
{
  setTopic( output_topic_editor_->text() );
}

//界面中按按钮程序停止机器人
void PX4_Panel::RobotSTOP_clicked()
{
    TrajClient_->cancelAllGoals();	//机器人停下来
    ROS_ERROR("EMERGENCY! STOP ROBOT!");
}

//界面中movej按钮
void PX4_Panel::control_robot_movej()
{
    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.push_back("shoulder_pan_joint");
    msg.joint_names.push_back("shoulder_lift_joint");
    msg.joint_names.push_back("elbow_joint");
    msg.joint_names.push_back("wrist_1_joint");
    msg.joint_names.push_back("wrist_2_joint");
    msg.joint_names.push_back("wrist_3_joint");
    msg.points.resize(1);
    msg.points[0].time_from_start = ros::Duration(2);
    msg.points[0].positions.resize(6);
    std::copy(joint_positions_movej_.begin(),joint_positions_movej_.end(),msg.points[0].positions.begin());
    msg.points[0].velocities.push_back(0.05);
    msg.points[0].velocities.push_back(0);
    msg.points[0].velocities.push_back(0);
    msg.points[0].velocities.push_back(0);
    msg.points[0].velocities.push_back(0);
    msg.points[0].velocities.push_back(0);
    
    msg.points[0].accelerations.push_back(0);
    msg.points[0].accelerations.push_back(0);
    msg.points[0].accelerations.push_back(0);
    msg.points[0].accelerations.push_back(0);
    msg.points[0].accelerations.push_back(0);
    msg.points[0].accelerations.push_back(0.5);
    control_robot_position_movej_publisher_.publish(msg);
}
//界面中Track按钮
void PX4_Panel::track_aruco_enable()
{
     
    std_msgs::Int16 msg;
    msg.data = track_aruco_flag_?-1:1;
    track_aruco_flag_ = !track_aruco_flag_;
    if(track_aruco_flag_)
    {
        push_button_track_aruco_Mode->setStyleSheet("background-color:rgb(0,255,0)");
    }
    else
    {
        push_button_track_aruco_Mode->setStyleSheet("background-color:rgb(255,0,0)");
    }
    track_aruco_publisher_.publish(msg);
}

//界面中的updata按键
void PX4_Panel::updata_movej_editors()
{
    if(!push_button_movej->isEnabled())
    {
        push_button_movej->setEnabled(true);
    }
    for(int i=0;i<6;i++)
    {
        std::string temp_str = std::to_string(joint_states_[i]*((jointstates_use_radian_FLAG_==true)?1:1/3.1415926*180.0));    
        //plane_state_msgs[i]->setText( QString::fromStdString(temp_str.substr(0,temp_str.size()-3) ) );
    }
     
}

//tab widget 中的 recognition按键
void PX4_Panel::update_recogniton_result()
{
    std::string temp_str = std::to_string(123443435); 
    //text_browser_recogniton_result_->append( QString::fromStdString(temp_str.substr(0,temp_str.size()-2) ) );    //添加
    // text_browser_recogniton_result_->setText( output_msgs );  //设置
}

void PX4_Panel::autoScroll() 
{
    text_browser_recogniton_result_->moveCursor(QTextCursor::End);  //将接收文本框的滚动条滑到最下面
}
//tab widget 中的 confirm 按键
void PX4_Panel::take_off_result()
{
    //参考：https://www.cnblogs.com/wang1994/p/5943154.html
    //ROS_INFO("EMERGENCY! STOP ROBOT!");
    //std::cout<<"aaaaaaaa"<<std::endl;
    QString program = "rosrun frenet_optimal_trajectory_pkg offboard_take_off_test_node";
    QStringList arg;
    //take_off_Process->setStandardOutputFile("/usb_cam_log.txt"); //可将终端上显示的内容写入到log.txt文件中
    take_off_Process->start(program);
    if(!take_off_Process->waitForStarted())
    {
        qDebug()<<"failure!";
    }else
    {
        qDebug()<<"succ!";
        //启动成功的判断条件
        push_button_take_off_->setStyleSheet("background-color:rgb(0,255,0)");
        push_button_take_off_->setEnabled(false);
        push_button_call_service_->setEnabled(true);
        push_button_call_service_->setStyleSheet("background-color:rgb(255,0,0)");

    }
    QProcess::ProcessState take_off_state = take_off_Process->state();    //获取当前进程的状态
    switch(take_off_state)
    {
        case QProcess::NotRunning:
            qDebug()<<"Not Running";
            break;
        case QProcess::Starting:
            qDebug()<<"Starting";
            break;
        case QProcess::Running:
            qDebug()<<"Running";
            break;
        default:
            qDebug()<<"otherState";
            break;
    }


    //system("roslaunch usb_cam usb_cam.launch");
    //qDebug()<<"code ending.";
    //qDebug()<<QString::fromLocal8Bit(dos2unix_Process->readAllStandardOutput());

}
//tab widget 中的 grasp按键
void PX4_Panel::shut_down_result()
{   
    // multi_grasptarget_srv_.request.grasp_target_pose.resize(0);
    // multi_grasptarget_srv_.request.placement_pose.resize(1);

    // Eigen::Vector3d rotationVector = Eigen::Vector3d(0.0,3.14,0.0);
    // Eigen::AngleAxisd V2(rotationVector.norm(),rotationVector/rotationVector.norm());
    // Eigen::Quaterniond rotationQuaterniond(V2);
    // //std::cout<<"rotationQuaterniond"<<std::endl;
    
    // geometry_msgs::Pose temp3;
    // temp3 = target_pose_in_robot_base_ ;
    // multi_grasptarget_srv_.request.grasp_target_pose.push_back(temp3);

    // temp3.orientation.x = rotationQuaterniond.x();
    // temp3.orientation.y = rotationQuaterniond.y();
    // temp3.orientation.z = rotationQuaterniond.z();
    // temp3.orientation.w = rotationQuaterniond.w();
    // multi_grasptarget_srv_.request.placement_pose[0].position.x = multi_grasptarget_srv_.request.grasp_target_pose[0].position.x - 0.1;
    // multi_grasptarget_srv_.request.placement_pose[0].position.y = multi_grasptarget_srv_.request.grasp_target_pose[0].position.y - 0.1;
    // multi_grasptarget_srv_.request.placement_pose[0].position.z = multi_grasptarget_srv_.request.grasp_target_pose[0].position.z;
    // multi_grasptarget_srv_.request.placement_pose[0].orientation = temp3.orientation;

    // if (execute_grasp_service_client_.call(multi_grasptarget_srv_))
    // {
    //     ROS_INFO("send grasp target, grasp start" );
    // }
    // else
    // {
    //     ROS_ERROR("Failed to call service target_position_and_pos");
    // }

    //关闭Take_off程序
    QString c = "rosrun frenet_optimal_trajectory_pkg offboard_take_off_test_node";
    //int pInt = QProcess::execute(c);    //关闭后台notepad.exe进程，阻塞式运行,一直占用cpu,成功返回0，失败返回1
    //qDebug()<<"pInt:"<<pInt;
    take_off_Process->close();
    // if(!take_off_Process->waitForFinished())
    // {
    //     qDebug()<<"failure!";
    // }else
    // {
    //     qDebug()<<"succ!";
    // }
    QProcess::ProcessState take_off_state = take_off_Process->state();    //获取当前进程的状态
    switch(take_off_state)
    {
        case QProcess::NotRunning:
            qDebug()<<"Not Running";
            break;
        case QProcess::Starting:
            qDebug()<<"Starting";
            break;
        case QProcess::Running:
            qDebug()<<"Running";
            break;
        default:
            qDebug()<<"otherState";
            break;
    }
    push_button_take_off_->setStyleSheet("background-color:rgb(255,255,255)");
    push_button_take_off_->setEnabled(true);
    push_button_call_service_->setEnabled(false);
    push_button_call_service_->setStyleSheet("background-color:rgb(255,255,255)");

}


//tab widget 中的 control robot 中的按键
void PX4_Panel::control_robot_speedj_1()
{
    publish_speedj_(0,1);
}
void PX4_Panel::control_robot_speedj_2()
{
    publish_speedj_(0,-1);
}
void PX4_Panel::control_robot_speedj_3()
{
    publish_speedj_(1,1);
}
void PX4_Panel::control_robot_speedj_4()
{
    publish_speedj_(1,-1);
}
void PX4_Panel::control_robot_speedj_5()
{
    publish_speedj_(2,1);
}
void PX4_Panel::control_robot_speedj_6()
{
    publish_speedj_(2,-1);
}
void PX4_Panel::control_robot_speedj_7()
{
    publish_speedj_(3,1);
}
void PX4_Panel::control_robot_speedj_8()
{
    publish_speedj_(3,-1);
}
void PX4_Panel::control_robot_speedj_9()
{
    publish_speedj_(4,1);
}
void PX4_Panel::control_robot_speedj_10()
{
    publish_speedj_(4,-1);
}
void PX4_Panel::control_robot_speedj_11()
{
    publish_speedj_(5,1);
}
void PX4_Panel::control_robot_speedj_12()
{
    publish_speedj_(5,-1);
}

void PX4_Panel::control_robot_speedl_1()
{
    publish_speedl_(0,1);
}
void PX4_Panel::control_robot_speedl_2()
{
    publish_speedl_(0,-1);
}
void PX4_Panel::control_robot_speedl_3()
{
    publish_speedl_(1,1);
}
void PX4_Panel::control_robot_speedl_4()
{
    publish_speedl_(1,-1);
}
void PX4_Panel::control_robot_speedl_5()
{
    publish_speedl_(2,1);
}
void PX4_Panel::control_robot_speedl_6()
{
    publish_speedl_(2,-1);
}

void PX4_Panel::switch_radian_degree()
{
    if(jointstates_use_radian_FLAG_==true)
    {
        jointstates_use_radian_FLAG_ = false;
        //push_button_radian_degree_switch_->setText("R/&D");
    }
    else
    {
        jointstates_use_radian_FLAG_ = true;
        //push_button_radian_degree_switch_->setText("&R/D");
    }   

    for(int i=0;i<6;i++)
    {
        std::string temp_str = std::to_string(joint_states_[i]*((jointstates_use_radian_FLAG_==true)?1:1/3.1415926*180.0));    
        plane_state_msgs[i]->setText( QString::fromStdString(temp_str.substr(0,temp_str.size()-3) ) );
    }
}


//更新无人机的状态
void PX4_Panel::updata_plane_state()
{
    // srand((unsigned)time(NULL));  
    // for(int i=0;i<6;i++)
    // {
    //     int aaa = rand()%100;
    //     std::string abcd =  std::to_string(aaa);
    //     plane_state_msgs[i]->setText( QString::fromStdString(abcd) );
    //     // QString temp_string = plane_state_msgs[i]->text();
    //     // float position = temp_string.toFloat();  
    //     // joint_positions_movej_[i] = position*(jointstates_use_radian_FLAG_==true?1:1/180.0*3.1415926);
    // }
    std::string data;
    if(current_state.connected == false)
    {
        data = "false";
    }
    else
    {
        data = "true";
    }
    plane_state_msgs[0]->setText( QString::fromStdString(data) );
    if(current_state.armed == false)
    {
        data = "false";
    }
    else
    {
        data = "true";
    }
    plane_state_msgs[1]->setText( QString::fromStdString(data) );
    if(current_state.guided == false)
    {
        data = "false";
    }
    else
    {
        data = "true";
    }
    plane_state_msgs[2]->setText( QString::fromStdString(data) );
    // if(current_state.manual_input == false)
    // {
    //     data = "false";
    // }
    // else
    // {
    //     data = "true";
    // }
    data = std::to_string(GPS_fix_state.status);
    plane_state_msgs[3]->setText( QString::fromStdString(data) );
    plane_state_msgs[4]->setText( QString::fromStdString(current_state.mode) );
    data = std::to_string(current_state.system_status);
    plane_state_msgs[5]->setText( QString::fromStdString(data) );
    // //test
    // std::cout<<GPS_fix_state.status<<std::endl;
    // std::cout<<"-------------"<<std::endl;
    output_msgs = take_off_Process->readAllStandardOutput();  //将takeoff程序的输出内容获取
    if(output_msgs.size()>0)
    {
        text_browser_recogniton_result_->append( output_msgs );  //追加文字

    }

}


// 设置topic命名
void PX4_Panel::setTopic( const QString& new_topic )
{
  // 检查topic是否发生改变.
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
	
    // 如果命名为空，不发布任何信息
    if( output_topic_ == "" )
    {
      velocity_publisher_.shutdown();
    }
	// 否则，初始化publisher
    else
    {
      velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( output_topic_.toStdString(), 1 );
    }

    Q_EMIT configChanged();//应该是rviz中定义的信号，表示rviz的config改变，退出时提示是否保存
  }
}

// 发布消息
void PX4_Panel::sendVel()
{
    if( ros::ok() && velocity_publisher_ )
    {
        geometry_msgs::Twist msg;
        msg.linear.x = linear_velocity_;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = angular_velocity_;
        velocity_publisher_.publish( msg );
    }

    for(int i=0;i<6;i++)
    {
        std::string temp_str;
        if(jointstates_use_radian_FLAG_)
        {
            temp_str = std::to_string(joint_states_[i]);    
        }
        else
        {
            temp_str = std::to_string(joint_states_[i]/3.1415926*180.0);
        }
        //joint_states_label_[i]->setText( QString::fromStdString(temp_str.substr(0,temp_str.size()-3) ) );
    }


}

// 重载父类的功能
void PX4_Panel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}

// 重载父类的功能，加载配置数据
void PX4_Panel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
  }
}



void PX4_Panel::robot_state_callback(const sensor_msgs::JointStateConstPtr& state)
{
    joint_states_.resize(6);
    double velocities_square_sum =0;

    for(int i=0;i<state->name.size();i++)
    {
        if(state->name[i]=="shoulder_pan_joint")
        {
            joint_states_[0]=state->position[i];
            velocities_square_sum += fabs(state->velocity[i]*state->velocity[i]);
        }
        else if(state->name[i]=="shoulder_lift_joint")
        {
            joint_states_[1]=state->position[i];
            velocities_square_sum += fabs(state->velocity[i]*state->velocity[i]);
        }
        else if(state->name[i]=="elbow_joint")
        {
            joint_states_[2]=state->position[i];
            velocities_square_sum += fabs(state->velocity[i]*state->velocity[i]);
        }
        else if(state->name[i]=="wrist_1_joint")
        {
            joint_states_[3]=state->position[i];
            velocities_square_sum += fabs(state->velocity[i]*state->velocity[i]);
        }
        else if(state->name[i]=="wrist_2_joint")
        {
            joint_states_[4]=state->position[i];
            velocities_square_sum += fabs(state->velocity[i]*state->velocity[i]);
        }
        else if(state->name[i]=="wrist_3_joint")
        {
            joint_states_[5]=state->position[i];
            velocities_square_sum += fabs(state->velocity[i]*state->velocity[i]);
        }
    }    
}

void PX4_Panel::recognition_result_callback(const geometry_msgs::PoseStampedPtr msg)
{
    try{
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer_.lookupTransform("base", "wrist_2_link",ros::Time(0));
        Eigen::Matrix4d wrist2_2_base = Eigen::Matrix4d::Identity();
        // std::cout << transformStamped.transform  <<std::endl; 

        Eigen::Quaterniond q_temp;
        q_temp.x()=transformStamped.transform.rotation.x;
        q_temp.y()=transformStamped.transform.rotation.y;
        q_temp.z()=transformStamped.transform.rotation.z;
        q_temp.w()=transformStamped.transform.rotation.w;
         
        wrist2_2_base.block(0,0,3,3) << q_temp.toRotationMatrix();
        wrist2_2_base.block(0,3,1,1) << transformStamped.transform.translation.x;    
        wrist2_2_base.block(1,3,1,1) << transformStamped.transform.translation.y;
        wrist2_2_base.block(2,3,1,1) << transformStamped.transform.translation.z;
        // wrist2_2_base = transRosMsgTransform2EigenMatrix(transformStamped.transform);
        Eigen::Matrix4d flip_x_matrix = Eigen::Matrix4d::Identity();
        flip_x_matrix(1,1) = 0;
        flip_x_matrix(1,2) = -1;
        flip_x_matrix(2,1) = 1;
        flip_x_matrix(2,2) = 0;
        target_pose_in_robot_base_  = transEigenMatrix2RosMsgPose( wrist2_2_base * d435i_2_wrist2link_ * transRosMsgPose2EigenMatrix(msg->pose) *flip_x_matrix);
        std::string pose_str;
        pose_str = std::to_string(target_pose_in_robot_base_.position.x) + "," + std::to_string(target_pose_in_robot_base_.position.y) + "," + std::to_string(target_pose_in_robot_base_.position.z) + "\n ";
        std::string orientation_ptr;
        orientation_ptr  = "x: "+std::to_string(target_pose_in_robot_base_.orientation.x) + ",y: " + std::to_string(target_pose_in_robot_base_.orientation.y) + "\nz: " + std::to_string(target_pose_in_robot_base_.orientation.z)+ ",w: " + std::to_string(target_pose_in_robot_base_.orientation.w) ;
        
        tf::Transform tf_transform;
        tf_transform.setOrigin(tf::Vector3(target_pose_in_robot_base_.position.x,target_pose_in_robot_base_.position.y,target_pose_in_robot_base_.position.z));
        tf::Quaternion tfq_temp;
        tfq_temp.setValue(target_pose_in_robot_base_.orientation.x,target_pose_in_robot_base_.orientation.y,target_pose_in_robot_base_.orientation.z,target_pose_in_robot_base_.orientation.w);
 
        tf_transform.setRotation(tfq_temp);
        static tf::TransformBroadcaster transform_broadcaster_;
        transform_broadcaster_.sendTransform(tf::StampedTransform(tf_transform,ros::Time::now(),"base","target_aruco"));
        
        text_browser_recogniton_result_->setText( QString::fromStdString( pose_str+orientation_ptr ) );
        // ROS_ERROR("!");
     }
	catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
	}	

 

}
void PX4_Panel::publish_speedj_(int joint_index , int d_or_i)
{
    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.push_back("shoulder_pan_joint");
    msg.joint_names.push_back("shoulder_lift_joint");
    msg.joint_names.push_back("elbow_joint");
    msg.joint_names.push_back("wrist_1_joint");
    msg.joint_names.push_back("wrist_2_joint");
    msg.joint_names.push_back("wrist_3_joint");
    msg.points.resize(1);
    msg.points[0].velocities.resize(6);
    msg.points[0].accelerations.resize(6);
    msg.points[0].time_from_start = ros::Duration(0.2);
    double speed = 0.05;
    for(int i=0;i<6;i++)
    {
        msg.points[0].velocities[i] = 0.0;
        if(i==joint_index)
        {
            msg.points[0].velocities[i] = speed * d_or_i;
        }
    }
    msg.points[0].accelerations[0] = speed*125; 

    control_robot_speedj_publisher_.publish(msg);
}

void PX4_Panel::publish_speedl_(int dimension , int d_or_i)
{
    trajectory_msgs::JointTrajectory msg;
    msg.points.resize(1);
    msg.points[0].velocities.resize(6);
    msg.points[0].accelerations.resize(6);
    msg.points[0].time_from_start = ros::Duration(0.2);
    double speed = 0.025;
    for(int i=0;i<6;i++)
    {
        msg.points[0].velocities[i] = 0.0;
        if(i==dimension)
        {
            msg.points[0].velocities[i] = speed*d_or_i;
        }
    }
    
    msg.points[0].accelerations[0] = speed*125;

     
    control_robot_speddl_publisher_.publish(msg);
}

bool PX4_Panel::ReadMatrix(std::string FileName, Eigen::Matrix4d& transMat)
{
    cv::FileStorage fs(FileName, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        return false;
    }
    cv::Mat cvtransMat;
    fs["TransMat"] >> cvtransMat;
    cv2eigen(cvtransMat, transMat);
    fs.release();
    return true;
}

Eigen::Matrix4d PX4_Panel::transRosMsgPose2EigenMatrix(geometry_msgs::Pose pose)
{
	Eigen::Matrix4d matrix_temp = Eigen::Matrix4d::Identity();
	Eigen::Quaterniond q_temp; 
	q_temp.x()=pose.orientation.x;
	q_temp.y()=pose.orientation.y;
	q_temp.z()=pose.orientation.z;
	q_temp.w()=pose.orientation.w;
	matrix_temp.block(0,0,3,3) << q_temp.toRotationMatrix();
	matrix_temp.block(0,3,1,1) << pose.position.x;    
	matrix_temp.block(1,3,1,1) << pose.position.y;
	matrix_temp.block(2,3,1,1) << pose.position.z;
	return matrix_temp;
}
Eigen::Matrix4d transRosMsgTransform2EigenMatrix(geometry_msgs::Transform transform)
{
    // ROS_ERROR("0");
	Eigen::Matrix4d matrix_temp = Eigen::Matrix4d::Identity();
    // ROS_ERROR("1");
	Eigen::Quaterniond q_temp; 
    // ROS_ERROR("2");
	q_temp.x()=transform.rotation.x;
	q_temp.y()=transform.rotation.y;
	q_temp.z()=transform.rotation.z;
	q_temp.w()=transform.rotation.w;
    // ROS_ERROR("3");
	matrix_temp.block(0,0,3,3) << q_temp.toRotationMatrix();
    // ROS_ERROR("4");
	matrix_temp.block(0,3,1,1) << transform.translation.x;    
	matrix_temp.block(1,3,1,1) << transform.translation.y;
	matrix_temp.block(2,3,1,1) << transform.translation.z;
    // ROS_ERROR("5");
	return matrix_temp;
}
geometry_msgs::Pose PX4_Panel::transEigenMatrix2RosMsgPose(Eigen::Matrix4d matrix)
{
    geometry_msgs::Pose pose_temp;
    pose_temp.position.x = matrix(0,3);
    pose_temp.position.y = matrix(1,3);
    pose_temp.position.z = matrix(2,3);
    

    Eigen::AngleAxisd V2;
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix = matrix.block(0,0,3,3);
    V2.fromRotationMatrix(rotationMatrix);
 	Eigen::Quaterniond q_temp(V2); 
 	pose_temp.orientation.w = q_temp.w();
    pose_temp.orientation.x = q_temp.x();
    pose_temp.orientation.y = q_temp.y();
    pose_temp.orientation.z = q_temp.z();
 
	return pose_temp;
}


}// end namespace ur_rviz_plugin 







// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ur_rviz_plugin::PX4_Panel,rviz::Panel )
// END_TUTORIAL
