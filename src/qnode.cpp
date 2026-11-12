/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
 
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/cyrobot_monitor/qnode.hpp"
#include <QDebug>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cyrobot_monitor {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{
//    读取topic的设置
    QSettings topic_setting("topic_setting","cyrobot_monitor");
    odom_topic= topic_setting.value("topic_odom","raw_odom").toString();
    power_topic=topic_setting.value("topic_power","power").toString();
    pose_topic=topic_setting.value("topic_amcl","amcl_pose").toString();
    power_min=topic_setting.value("power_min","10").toString();
    power_max=topic_setting.value("power_max","12").toString();
}

QNode::~QNode() {
    if(ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init()
{
    ros::init(init_argc, init_argv, "cyrobot_monitor");
    if ( ! ros::master::check() )
    {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    //创建速度话题的订阅者
    cmdVel_sub = n.subscribe<nav_msgs::Odometry>(odom_topic.toStdString(),200,&QNode::speedCallback,this);
    power_sub = n.subscribe(power_topic.toStdString(),1000,&QNode::powerCallback,this);
    //机器人位置话题
    pos_sub = n.subscribe(pose_topic.toStdString(),1000,&QNode::poseCallback,this);
    //导航目标点发送话题
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1000);
    //速度控制话题
    cmd_pub = n.advertise<std_msgs::Int8>("cmd_vel", 1000);
    start_speed_pub = n.advertise<std_msgs::Int8>("start_target_speed",1000);
    end_speed_pub = n.advertise<std_msgs::Int8>("end_target_speed",1000);
    //参数话题
    nav_pos_sub = n.subscribe("nav_pos_message",1000,&QNode::navPosMsg_callback,this);
    car_gear_sub = n.subscribe("car_gear",1000,&QNode::carGear_callback,this);
    car_angle_sub = n.subscribe("car_angle",1000,&QNode::carAngle_callback,this);
    car_acc_sub = n.subscribe("car_acc",1000,&QNode::carAcc_callback,this);
    car_current_speed_sub = n.subscribe("car_speed",1000,&QNode::carCurrentSpeed_callback,this);
    nac_sat_sub=n.subscribe("nav_sat_message",1000,&QNode::navSatMsg_callback,this);




    start();
    return true;
}

//初始化的函数*********************************
bool QNode::init(const std::string &master_url, const std::string &host_url)
{
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"cyrobot_monitor");
    if ( ! ros::master::check() )
    {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    //创建速度话题的订阅者
    cmdVel_sub = n.subscribe<nav_msgs::Odometry>(odom_topic.toStdString(),200,&QNode::speedCallback,this);
    power_sub = n.subscribe(power_topic.toStdString(),1000,&QNode::powerCallback,this);
    //机器人位置话题
    pos_sub = n.subscribe(pose_topic.toStdString(),1000,&QNode::poseCallback,this);
    //导航目标点发送话题
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1000);
    //速度控制话题
    cmd_pub = n.advertise<std_msgs::Int8>("cmd_vel", 1000);
    start_speed_pub = n.advertise<std_msgs::Int8>("start_target_speed",1000);
    end_speed_pub = n.advertise<std_msgs::Int8>("end_target_speed",1000);
    
    start();
    return true;
}

void QNode::disinit()
{
    if(ros::isStarted())
    {
        ROS_INFO("ROS will shutdown");
        ros::shutdown();
        ros::waitForShutdown();
    }
    this->exit();
}

QMap<QString,QString> QNode::get_topic_list()
{
    ros::master::V_TopicInfo topic_list;
    ros::master::getTopics(topic_list);
    QMap<QString,QString> res;
    for(auto topic:topic_list)
    {

        res.insert(QString::fromStdString(topic.name),QString::fromStdString(topic.datatype));

    }
    return res;
}
//机器人位置话题的回调函数
void QNode::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pos)
{
    emit position(pos.header.frame_id.data(), pos.pose.pose.position.x,pos.pose.pose.position.y,pos.pose.pose.orientation.z,pos.pose.pose.orientation.w);
//    qDebug()<<<<" "<<pos.pose.pose.position.y;
}
void QNode::powerCallback(const std_msgs::Float32 &message_holder)
{

    emit power(message_holder.data);
}
void QNode::myCallback(const std_msgs::Float64 &message_holder)
{
    qDebug()<<message_holder.data<<endl;
}
//发布导航目标点信息
void QNode::set_goal(QString frame,double x,double y,double z,double w)
{
    geometry_msgs::PoseStamped goal;
    //设置frame
    goal.header.frame_id=frame.toStdString();
    //设置时刻
    goal.header.stamp=ros::Time::now();
    goal.pose.position.x=x;
    goal.pose.position.y=y;
    goal.pose.position.z=0;
    goal.pose.orientation.z=z;
    goal.pose.orientation.w=w;
    goal_pub.publish(goal);
    ros::spinOnce();
}
//速度回调函数
void QNode::speedCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

    emit speed_x(msg->twist.twist.linear.x);
    emit speed_y(msg->twist.twist.linear.y);
}




//组合惯导位姿回调函数
void QNode::navPosMsg_callback(const geometry_msgs::Pose2D &msg)
{
    emit memsMsg_navPos_x_vel(msg.x);
    emit memsMsg_navPos_y_vel(msg.y);
    emit memsMsg_navPos_theta_vel(msg.theta);
}


//车辆档位回调函数
void QNode::carGear_callback(const  std_msgs::Int32 &msg)
{
    emit carMsg_gear_vel(msg.data);
}

//车辆方向盘回调函数
void QNode::carAngle_callback(const  std_msgs::Float32 &msg)
{
    emit carMsg_angle_vel(msg.data);
}
//车辆加速度回调函数
void QNode::carAcc_callback(const  std_msgs::Float32 &msg)
{
    emit carMsg_acc_vel(msg.data);
}
//车辆加速度回调函数
void QNode::carCurrentSpeed_callback(const  std_msgs::Float32 &msg)
{
    emit carMsg_current_speed_vel(msg.data);
}
//组合惯导经纬度高度状态回调函数
void QNode::navSatMsg_callback(const sensor_msgs::NavSatFix &msg)
{
    emit memsMsg_status_vel(msg.status.status);
    emit memsMsg_service_vel(msg.status.service);
    emit memsMsg_latitude_vel(msg.latitude);
    emit memsMsg_longitude_vel(msg.longitude);
    emit memsMsg_altitude_vel(msg.altitude);
}



void QNode::run() {
//        int count=0;
        ros::Rate loop_rate(1);
        //当当前节点没有关闭时
        while ( ros::ok() ) {
            //调用消息处理回调函数
            ros::spinOnce();

            loop_rate.sleep();
        }
        //如果当前节点关闭
        //Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)


}
//发布机器人速度控制
void QNode::move_base(bool key)
{
   bool k = key;
   std_msgs::Int8 target_speed;
   if(k == false)
   {
    target_speed.data = -10;
   }
   else
   {
     target_speed.data = 5;
   }

     cmd_pub.publish(target_speed);
     ros::spinOnce();
}


void QNode::startSpeed_pub()
{
  std_msgs::Int8 target_speed;
  target_speed.data = 5;
  start_speed_pub.publish(target_speed);

}

void QNode::endSpeed_pub()
{
  std_msgs::Int8 target_speed;
  target_speed.data = -10;
  start_speed_pub.publish(target_speed);
}






 //订阅图片话题，并在label上显示
 void QNode::Sub_Image(QString topic,int frame_id)
 {
      ros::NodeHandle n;
      image_transport::ImageTransport it_(n);
     switch (frame_id) {
         case 0:
            image_sub0=it_.subscribe(topic.toStdString(),100,&QNode::imageCallback0,this);
         break;
         case 1:
             image_sub1=it_.subscribe(topic.toStdString(),100,&QNode::imageCallback1,this);
          break;
         case 2:
             image_sub2=it_.subscribe(topic.toStdString(),100,&QNode::imageCallback2,this);
          break;
         case 3:
             image_sub3=it_.subscribe(topic.toStdString(),100,&QNode::imageCallback3,this);
          break;
     }
     ros::spinOnce();
 }

 //图像话题的回调函数
 void QNode::imageCallback0(const sensor_msgs::ImageConstPtr& msg)
 {
     cv_bridge::CvImagePtr cv_ptr;

     try
       {
         //深拷贝转换为opencv类型
         cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
         QImage im=Mat2QImage(cv_ptr->image);
         emit Show_image(0,im);
       }
       catch (cv_bridge::Exception& e)
       {

         log(Error,("video frame0 exception: "+QString(e.what())).toStdString());
         return;
       }
 }
 //图像话题的回调函数
 void QNode::imageCallback1(const sensor_msgs::ImageConstPtr& msg)
 {
     cv_bridge::CvImagePtr cv_ptr;
     try
       {
         //深拷贝转换为opencv类型
         cv_ptr = cv_bridge::toCvCopy(msg,video1_format.toStdString());
         QImage im=Mat2QImage(cv_ptr->image);
         emit Show_image(1,im);
       }
       catch (cv_bridge::Exception& e)
       {
         log(Error,("video frame1 exception: "+QString(e.what())).toStdString());
         return;
       }
 }
 //图像话题的回调函数
 void QNode::imageCallback2(const sensor_msgs::ImageConstPtr& msg)
 {
     cv_bridge::CvImagePtr cv_ptr;
     try
       {
         //深拷贝转换为opencv类型
         cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
         QImage im=Mat2QImage(cv_ptr->image);
         emit Show_image(2,im);
       }
       catch (cv_bridge::Exception& e)
       {
         log(Error,("video frame2 exception: "+QString(e.what())).toStdString());
         return;
       }
 }
 //图像话题的回调函数
 void QNode::imageCallback3(const sensor_msgs::ImageConstPtr& msg)
 {
     cv_bridge::CvImagePtr cv_ptr;
     try
       {
         //深拷贝转换为opencv类型
         cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
         QImage im=Mat2QImage(cv_ptr->image);
         emit Show_image(3,im);
       }
       catch (cv_bridge::Exception& e)
       {
         log(Error,("video frame3 exception: "+QString(e.what())).toStdString());
         return;
       }
 }
 QImage QNode::Mat2QImage(cv::Mat const& src)
 {
   QImage dest(src.cols, src.rows, QImage::Format_ARGB32);

   const float scale = 255.0;

   if (src.depth() == CV_8U) {
     if (src.channels() == 1) {
       for (int i = 0; i < src.rows; ++i) {
         for (int j = 0; j < src.cols; ++j) {
           int level = src.at<quint8>(i, j);
           dest.setPixel(j, i, qRgb(level, level, level));
         }
       }
     } else if (src.channels() == 3) {
       for (int i = 0; i < src.rows; ++i) {
         for (int j = 0; j < src.cols; ++j) {
           cv::Vec3b bgr = src.at<cv::Vec3b>(i, j);
           dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
         }
       }
     }
   } else if (src.depth() == CV_32F) {
     if (src.channels() == 1) {
       for (int i = 0; i < src.rows; ++i) {
         for (int j = 0; j < src.cols; ++j) {
           int level = static_cast<int>(scale * src.at<float>(i, j));
           dest.setPixel(j, i, qRgb(level, level, level));
         }
       }
     } else if (src.channels() == 3) {
       for (int i = 0; i < src.rows; ++i) {
         for (int j = 0; j < src.cols; ++j) {
           cv::Vec3f bgr = scale * src.at<cv::Vec3f>(i, j);
           dest.setPixel(j, i, qRgb(static_cast<int>(bgr[2]), static_cast<int>(bgr[1]), static_cast<int>(bgr[0])));
         }
       }
     }
   }

   return dest;
 }
void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace cyrobot_monitor
