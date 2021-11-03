#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"


/**
 * Class to publish specific MiR goals
 */
class Tasks
{
public:
  Tasks(void);
  ~Tasks(void) = default;
  void Taskscallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);     // Callback of move_base/result
  void goals(double x, double y, double z, double w);                           // Method for publishing a new goal

private:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
  void task();
  int state = 1;                                                                //Variable which represent the actual goal
};

/**
 * Class constructor
 */
Tasks::Tasks(void)
{
  pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);  //initialise publisher
  sub = n.subscribe("/move_base/result", 1000, &Tasks::Taskscallback, this);         //initialise subscriber
}

/**
 * Subscriber callback of move_base/result, if goal is reached
 */
void Tasks::Taskscallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
  ROS_INFO_STREAM("Reached goal");
  if(state < 3) {state++;}                                                      //count the actual state
  else{state = 1;}                                                              //if goal 3 is reached, go back to goal 1
  task();                                                                       //set new goal
}

/**
 * Method for setting new goal
 */
void Tasks::task()
{
  //Going through the states
  if(state == 1){
    ROS_INFO_STREAM("Driving to Goal 1");
    goals(5.0, 8.0, 0.0, 1.0);
  }
  else if(state == 2){
    ROS_INFO_STREAM("Driving to Goal 2");
    goals(2.5, 5.0, 0.0, 1.0);
  }
  else if(state == 3){
    ROS_INFO_STREAM("Driving to Goal 3");
    goals(10.0, 4.0, 0.0, 1.0);
  }
}

/**
 * Method for publishing new MiR goal
 */
void Tasks::goals(double x, double y, double z, double w)
{
  geometry_msgs::PoseStamped goal;                                              //Create new message
  goal.header.stamp = ros::Time::now();                                         //Set header
  goal.header.frame_id = "map";                                                 //Set frame_id
  goal.pose.position.x = x;                                                     //Set x position from method task
  goal.pose.position.y = y;                                                     //Set y position from method task
  goal.pose.position.z = z;                                                     //Set z position from method task (always 0)
  goal.pose.orientation.w = w;                                                  //Set w position from method task
  pub.publish(goal);                                                            //publish new goal
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Task");
  Tasks no;
  ros::spin();
  return 0;
}
