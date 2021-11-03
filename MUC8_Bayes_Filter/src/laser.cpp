#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class Node
{
public:
  Node(void);
  ~Node(void) = default;
  void callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void TimerCallback(const ros::TimerEvent &event);

private:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::Timer myTimer;
  bool interrupt = false;
};

Node::Node(void)
{
  myTimer = n.createTimer(ros::Duration(10.0), &Node::TimerCallback, this);
  pub = n.advertise<sensor_msgs::LaserScan>("/newscan", 10);
  sub = n.subscribe("/scan", 1000, &Node::callback, this);
}

void Node::callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if(interrupt == false){pub.publish(msg);ROS_INFO_STREAM("No Interrupt");}
  else{ROS_ERROR_STREAM("Interrupt");}
}

void Node::TimerCallback(const ros::TimerEvent &event)
{
  if(interrupt == true){interrupt = false;}
  else{interrupt = true;}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Laser");
  ros::NodeHandle n;
  Node no;
  ros::spin();
  return 0;
}
