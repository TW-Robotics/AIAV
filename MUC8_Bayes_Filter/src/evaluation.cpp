#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include <tf/transform_listener.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include "opencv2/core/mat.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>


/**
 * Class to publish specific MiR goals
 */
class Evaluation
{
public:
  Evaluation(void);
  ~Evaluation(void) = default;
  void Evaluationcallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose);


private:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;

};

/**
 * Class constructor
 */
Evaluation::Evaluation(void)
{
  pub = n.advertise<geometry_msgs::Pose>("evaluation", 10000);                      //initialise publisher
  sub = n.subscribe("/pose", 10, &Evaluation::Evaluationcallback, this);         //initialise pose subscriber

}


/**
 * Subscriber callback
 */
void Evaluation::Evaluationcallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &posemsg)
{
  ROS_INFO_STREAM("Subscriber");
  tf::TransformListener listener;
  tf::TransformListener listener2;
  ros::Rate rate(1000.0);
  geometry_msgs::Pose msg;
  tf::StampedTransform transform;
  tf::StampedTransform transform2;
    try{
      listener.waitForTransform("/pose", "/pose_amcl", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("/pose", "/pose_amcl",ros::Time(0), transform);

      msg.position.x = transform.getOrigin().x();
      msg.position.y = transform.getOrigin().y();
      msg.orientation.x = transform.getRotation().x();
      msg.orientation.y = transform.getRotation().y();
      msg.orientation.z = transform.getRotation().z();
      msg.orientation.w = transform.getRotation().w();

      /*double z, y, x;
      tf2::Quaternion quat;
      tf2::convert(msg.orientation, quat);
      tf2::Matrix3x3 mat(quat);
      mat.getEulerZYX(z, y, x);*/

    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    pub.publish(msg);
    rate.sleep();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "evaluator");
  Evaluation no;
  ros::spin();
  return 0;
};
