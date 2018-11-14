#include "ros/ros.h"
#include "sbg_driver/SbgEkfEuler.h"
#include "sbg_driver/SbgImuData.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"

//-----Timer
ros::Timer tim_25hz;
//-----Subscriber
ros::Subscriber sub_imu_data;
ros::Subscriber sub_ekf_euler;
//-----Publisher
ros::Publisher pub_imu;

sensor_msgs::Imu msg_imu;

double offsetGYRO[3];
double angle_aligned[3];
uint8_t statusGYRO = 0;


void cllbck_tim_25hz(const ros::TimerEvent &event)
{
  msg_imu.header = std_msgs::Header();
  msg_imu.header.stamp = ros::Time::now();

  pub_imu.publish(msg_imu);
}

void cllbck_sub_imu_data(const sbg_driver::SbgImuDataConstPtr &msg)
{

  msg_imu.linear_acceleration.x =  msg->accel.x * 1.0063;
  msg_imu.linear_acceleration.y =  (- msg->accel.y) * 1.0063;
  msg_imu.linear_acceleration.z = (- msg->accel.z) * 1.0063;


  // msg_imu.linear_acceleration.x = 20;
  // msg_imu.linear_acceleration.y = 0;
  ROS_INFO("accelero = %f,%f,%f\n",msg_imu.linear_acceleration.x,msg_imu.linear_acceleration.y,msg_imu.linear_acceleration.z);
}

void cllbck_sub_ekf_euler(const sbg_driver::SbgEkfEulerConstPtr &msg)
{
  
  
  if(statusGYRO != 1){

    offsetGYRO[0] =   msg->angle.x;
    offsetGYRO[1] =   msg->angle.y;
    offsetGYRO[2] =   msg->angle.z;    

    statusGYRO = 1;
  }  
  
  angle_aligned[0] =  msg->angle.x ;
  angle_aligned[1] =  - msg->angle.y ;
  angle_aligned[2] =   atan2f(sinf(msg->angle.z - offsetGYRO[2]),cosf(msg->angle.z - offsetGYRO[2])) ;  
  
  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(angle_aligned[0], angle_aligned[1], angle_aligned[2]), msg_imu.orientation);
  ROS_INFO("%f,%f,%f\n\n\n\n",angle_aligned[0] * 57.2958,angle_aligned[1] * 57.2958,angle_aligned[2] * 57.2958);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_bridge");

  ros::NodeHandle NH;
  ros::MultiThreadedSpinner MTS;

  //-----Timer
  tim_25hz = NH.createTimer(ros::Duration(0.04), cllbck_tim_25hz);
  //-----Subscriber
  sub_imu_data = NH.subscribe("/imu_data", 8, cllbck_sub_imu_data);
  sub_ekf_euler = NH.subscribe("/ekf_euler", 8, cllbck_sub_ekf_euler);
  //-----Publisher
  pub_imu = NH.advertise<sensor_msgs::Imu>("/imu/data", 8);

  MTS.spin();

  return 0;
}