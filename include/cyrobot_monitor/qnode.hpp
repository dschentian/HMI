/**
 * @file /include/cyrobot_monitor/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/
 
#ifndef cyrobot_monitor_QNODE_HPP_
#define cyrobot_monitor_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QLabel>
#include <QStringListModel>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>   //image_transport
#include <cv_bridge/cv_bridge.h>              //cv_bridge
#include <sensor_msgs/image_encodings.h>    //图像编码格式
#include <sensor_msgs/Imu.h>                 //话题名：/imu_message
#include <sensor_msgs/NavSatFix.h>           //话题名：/nav_sat_message
#include <geometry_msgs/Pose2D.h>            //话题名：/nav_pos_message
#include <map>
#include <QLabel>
#include <QImage>
#include <QSettings>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cyrobot_monitor {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
  void disinit();
    void move_base(bool key);
    void startSpeed_pub();
    void endSpeed_pub();
    void set_goal(QString frame,double x,double y,double z,double w);
    void Sub_Image(QString topic,int frame_id);
    QMap<QString,QString> get_topic_list();
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
             Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);


Q_SIGNALS:
	void loggingUpdated();
  void rosShutdown();
  void speed_x(double x);
  void speed_y(double y);
  void power(float p);
  void Master_shutdown();
  void Show_image(int,QImage);
  void position(QString frame,double x,double y,double z,double w);
  void start_speed();
  void end_speed();

  //组合贯导x y theta 信号
  void memsMsg_navPos_x_vel(float);
  void memsMsg_navPos_y_vel(float);
  void memsMsg_navPos_theta_vel(float);
  //车辆档位信号
  void carMsg_gear_vel(float);
  //车辆方向盘信号
  void carMsg_angle_vel(float);
  //车辆加速度信号
  void carMsg_acc_vel(float);
  //车辆当前速度信号
  void carMsg_current_speed_vel(float);
  //组合惯导经纬度高度状态信号
  void memsMsg_status_vel(int);
  void memsMsg_service_vel(unsigned int);
  void memsMsg_latitude_vel(float);
  void memsMsg_longitude_vel(float);
  void memsMsg_altitude_vel(float);

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    ros::Subscriber cmdVel_sub;
    ros::Subscriber chatter_subscriber;
    ros::Subscriber pos_sub;
    ros::Subscriber power_sub;
    ros::Publisher goal_pub;
    ros::Publisher cmd_pub;
    QStringListModel logging_model;


    ros::Subscriber nav_pos_sub; //组合贯导位姿订阅
    ros::Subscriber car_gear_sub; //车辆档位订阅
    ros::Subscriber car_angle_sub; //车辆方向盘订阅
    ros::Subscriber car_acc_sub; //车辆加速度订阅
    ros::Subscriber car_current_speed_sub; //车辆当前速度订阅
    ros::Subscriber nac_sat_sub;//组合贯导位经纬度高度订阅



    //开始暂停按钮
    ros::Publisher start_speed_pub;
    ros::Publisher end_speed_pub;



    //图像订阅
    image_transport::Subscriber image_sub0;
    image_transport::Subscriber image_sub1;
    image_transport::Subscriber image_sub2;
    image_transport::Subscriber image_sub3;
    //图像format
    QString video0_format;
    QString video1_format;
    QString video2_format;
    QString video3_format;
    QString odom_topic;
    QString power_topic;
    QString pose_topic;
    QString power_max;
    QString power_min;
    QImage Mat2QImage(cv::Mat const& src);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pos);
    void speedCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void powerCallback(const std_msgs::Float32& message_holder);
    void imageCallback0(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback1(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback2(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback3(const sensor_msgs::ImageConstPtr& msg);
    void myCallback(const std_msgs::Float64& message_holder);

    //参数
    void navPosMsg_callback(const geometry_msgs::Pose2D &msg); //组合贯导位姿回调函数
    void carGear_callback(const std_msgs::Int32 &msg);//车辆档位回调函数
    void carAngle_callback(const std_msgs::Float32 &msg);//车辆档位回调函数
    void carAcc_callback(const std_msgs::Float32 &msg);//车辆加速度回调函数
    void carCurrentSpeed_callback(const std_msgs::Float32 &msg);//车辆当前速度回调函数
    void navSatMsg_callback(const sensor_msgs::NavSatFix &msg);//组合贯导经纬度高度回调函数
};

}  // namespace cyrobot_monitor

#endif /* cyrobot_monitor_QNODE_HPP_ */
