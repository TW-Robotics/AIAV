#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"

#include <iostream>
#include <fstream>

class Node
{
public:
  Node(void);
  ~Node(void) = default;
  void KalmanCallback(const nav_msgs::Odometry::ConstPtr &, const sensor_msgs::Imu::ConstPtr &);
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> mySyncPolicy;
  int cnt = 0;

private:
  ros::NodeHandle n;
  message_filters::Subscriber<nav_msgs::Odometry> odomSub;
  message_filters::Subscriber<sensor_msgs::Imu> imuSub;
  message_filters::Synchronizer<mySyncPolicy> sync;

  double v_x_t;
  double w_t;
  double imu;

  std::vector<double> v_x_t_values;
  std::vector<double> w_t_values;
  std::vector<double> imu_values;
};


Node::Node(void) : n(""), odomSub(n, "odometry/filtered", 10), imuSub(n, "imu_data", 10), sync(mySyncPolicy(10), odomSub, imuSub)
{
  sync.registerCallback(std::bind(&::Node::KalmanCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void Node::KalmanCallback(const nav_msgs::Odometry::ConstPtr &odomptr,const sensor_msgs::Imu::ConstPtr &imuptr)
{
  v_x_t = odomptr->twist.twist.linear.x;
  w_t = odomptr->twist.twist.angular.z;
  imu = imuptr->angular_velocity.z;

  ROS_INFO_STREAM(v_x_t);
  ROS_INFO_STREAM(w_t);
  ROS_INFO_STREAM(imu);

  v_x_t_values.push_back(v_x_t);
  w_t_values.push_back(w_t);
  imu_values.push_back(imu);

  ROS_INFO_STREAM(imu_values.size());

  cnt++;
  if(cnt == 1000)
  {
    std::ofstream myfile;
    myfile.open ("pykalmanData.csv");
    for(int i = 0; i < imu_values.size(); i++)
    {
      myfile << v_x_t_values[i] << "," << w_t_values[i] << "," << imu_values[i] << "\n";
    }
    myfile.close();
    ros::shutdown();
  }


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Parameters");
  Node no;
  ros::spin();
  return 0;
}
