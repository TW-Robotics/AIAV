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

#include <tf/transform_broadcaster.h>
/**
 * Class to publish specific MiR goals
 */
class Amclpose
{
public:
  Amclpose(void);
  ~Amclpose(void) = default;
  void Posecallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose);

private:
  ros::NodeHandle n;
  ros::Subscriber sub;
  tf::TransformBroadcaster broadcaster;
};

/**
 * Class constructor
 */
Amclpose::Amclpose(void)
{
  sub = n.subscribe("/amcl_pose", 10, &Amclpose::Posecallback, this);         //initialise amcl subscriber
}

/**
 * Subscriber callback
 */
void Amclpose::Posecallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amclposemsg)
{

  broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(amclposemsg->pose.pose.orientation.x,
                                                                              amclposemsg->pose.pose.orientation.y,
                                                                              amclposemsg->pose.pose.orientation.z,
                                                                              amclposemsg->pose.pose.orientation.w),
                                                                tf::Vector3(amclposemsg->pose.pose.position.x,
                                                                            amclposemsg->pose.pose.position.y,
                                                                              0)),ros::Time::now(),"map", "pose_amcl"));
}


int main(int argc, char** argv){
  ros::init(argc, argv, "amclPose");
  Amclpose no;
  ros::spin();
  return 0;
};
