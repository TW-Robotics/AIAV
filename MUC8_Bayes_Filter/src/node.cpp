#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "opencv2/core/mat.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>
#include <tf2_ros/transform_broadcaster.h>

/**
 * @class KalmanFilter
 * Class for implementation of Kalman filter localization upon laser scan dropout
 */
class KalmanFilter
{
public:
  KalmanFilter(void);
  ~KalmanFilter(void) = default;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> mySyncPolicy; /**< Appromixate time synchronization policy for imu and odometry data */
  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &);                                                // laser scanner callback
  void KalmanCallback(const nav_msgs::Odometry::ConstPtr &, const sensor_msgs::Imu::ConstPtr &);              // kalman filter implementation
  void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &);                              // saving current pose provided by amcl
  void PublishPose(geometry_msgs::PoseWithCovarianceStamped *);                                                   // used to update message with current data before publishing
  void TimerCallback(const ros::TimerEvent &);                                                                // timer callback
  void PredictPose(const nav_msgs::Odometry *);                                                               // kf prediction implementation
  void CorrectPose(const sensor_msgs::Imu *);                                                                 // kf correction implementation

private:
  ros::NodeHandle n;
  ros::Subscriber scanSub;                                 // laser scanner subscriber
  ros::Subscriber amclSub;                                 // amcl_pose subscriber
  ros::Timer myTimer;                                      // timer
  bool startLocalisation;                                  // flag to start kf localisation
  message_filters::Subscriber<nav_msgs::Odometry> odomSub; // odometry subscriber
  message_filters::Subscriber<sensor_msgs::Imu> imuSub;    // imu subscriber
  message_filters::Synchronizer<mySyncPolicy> sync;        // topic synchronizer
  ros::Publisher kfPosePub;                                // publisher of pose with covariance stamped gotten from KF
  geometry_msgs::PoseWithCovarianceStamped msgCor;         // corrected pose message

  // variables needed for KF
  ros::Time previousTime;                                           // time of previous callback
  double delta_t;                                                   // time difference between callbacks
  cv::Mat_<double> measurement = cv::Mat::zeros(3, 1, CV_64F);      // measurement, consist of (a_x, a_y, omega_z) in WCS
  cv::Mat_<double> pose = cv::Mat::zeros(8, 1, CV_64F);             // current pose
  cv::Mat_<double> posePrev = cv::Mat::zeros(8, 1, CV_64F);         // previous pose
  cv::Mat_<double> poseAmclLast = cv::Mat::zeros(8, 1, CV_64F);     // latest amcl pose
  cv::Mat_<double> cov = cv::Mat::zeros(8, 8, CV_64F);              // covariance
  cv::Mat_<double> covAmclLast = cv::Mat::zeros(3, 3, CV_64F);      // latest amcl covariance
  cv::Mat_<double> A = cv::Mat::eye(8, 8, CV_64F);                  // state transition matrix
  cv::Mat_<double> B = cv::Mat::zeros(8, 2, CV_64F);                // control-input matrix
  cv::Mat_<double> C = cv::Mat::zeros(3, 8, CV_64F);                // measurement model
  cv::Mat_<double> R = cv::Mat::zeros(8, 8, CV_64F);                // process noise
  cv::Mat_<double> Q = cv::Mat::zeros(3, 3, CV_64F);                // sensor noise
  cv::Mat_<double> K = cv::Mat::zeros(8, 3, CV_64F);                // Kalman gain

  double theta = 0; // angle along the z axis

  tf2_ros::TransformBroadcaster broadcaster;
};

/**
 * @brief Class constructor.
 * 
 * Creates subscribers to "newscan" and "amcl_pose" topic, synchronized subscriber to "odometry"  and "imu" topics, a timer callback and publisher to "pose" topic. Initializes covariance, process and sensor noise matrices.
 */
KalmanFilter::KalmanFilter(void) : n(""), odomSub(n, "odometry/filtered", 10), imuSub(n, "imu_data", 10),
                   sync(mySyncPolicy(10), odomSub, imuSub)
{
  delta_t = 0;
  sync.registerCallback(std::bind(&::KalmanFilter::KalmanCallback, this, std::placeholders::_1, std::placeholders::_2));
  amclSub = n.subscribe("amcl_pose", 10, &KalmanFilter::PoseCallback, this);
  scanSub = n.subscribe("newscan", 1000, &KalmanFilter::ScanCallback, this);
  myTimer = n.createTimer(ros::Duration(1.0), &KalmanFilter::TimerCallback, this);
  kfPosePub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 10);
  startLocalisation = false;

  // initialise covariance, process and sensor noise, and measurement model
  for (int i = 0; i < 3; i++) {
    cov(i, i) = 0.0001;
    C(i, i+5) = 1;
  }
  //data from pykalman
  double dataR[64] = {
    0.001,   0.0,   0.0,    0.0,    0.0,    0.0,     0.0,    0.0,
      0.0, 0.001,   0.0,    0.0,    0.0,    0.0,     0.0,    0.0,
      0.0,   0.0, 0.001,    0.0,    0.0,    0.0,     0.0,    0.0,
      0.0,   0.0,   0.0,  0.526, -0.027, -0.003,  -6.074,  0.260,
      0.0,   0.0,   0.0, -0.027,  0.248, -0.005,   0.197, -3.385,
      0.0,   0.0,   0.0, -0.003, -0.005,  0.339,   0.074, -0.032,
      0.0,   0.0,   0.0, -6.074,  0.197,  0.074,  97.526, -2.527,
      0.0,   0.0,   0.0,  0.260, -3.385, -0.032,  -2.527, 65.611};

  double dataQ[9] = {
    0.055,  0.002, 0.011,
    0.002, 12.367,-0.633,
   -0.011, -0.633, 8.210};

  R = cv::Mat(8, 8, CV_64F, dataR);
  Q = cv::Mat(3, 3, CV_64F, dataQ);

  std::cout << dataR << std::endl;
  std::cout << dataQ << std::endl;
}

/**
 * @brief Method for publishing current robot pose
 * 
 * Updates message before publishing using current pose data. Publishes pose and broadcasts map->pose transform.
 */
void KalmanFilter::PublishPose(geometry_msgs::PoseWithCovarianceStamped *msg)
{
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = "map";

// if laser scan data is still available, copy latest Amcl pose (and covariance) as current pose (and covariance)
  if (!startLocalisation)
  {
    for(int i=0; i<8; i++) {
      if (i<3) pose(i,0) = poseAmclLast(i,0);
      else pose(i,0)=0;
      for(int j=0; j<8; j++) {
        if (j<3 && i<3) {
          cov(i,j) = covAmclLast(i,j);
        }
        else {
          cov(i,j) = 0;
        }
      }
    }
    posePrev = pose;
  }

  msg->pose.pose.position.x = pose(0, 0); // position x
  msg->pose.pose.position.y = pose(1, 0); // position y
  msg->pose.pose.position.z = 0;          // position z

  tf2::Quaternion q;
  q.setEulerZYX(pose(2, 0), 0, 0);             // translate the angle to quaternion
  tf2::convert(q, msg->pose.pose.orientation); // convert quaternion to message

  // translation of the existing 3x3 covariance matrix to the required 6x6
  msg->pose.covariance[0] = cov(0, 0);
  msg->pose.covariance[1] = cov(0, 1);
  msg->pose.covariance[5] = cov(0, 2);
  msg->pose.covariance[6] = cov(1, 0);
  msg->pose.covariance[7] = cov(1, 1);
  msg->pose.covariance[11] = cov(1, 2);
  msg->pose.covariance[30] = cov(2, 0);
  msg->pose.covariance[31] = cov(2, 1);
  msg->pose.covariance[35] = cov(2, 2);

  kfPosePub.publish(*msg); // publish pose with covariance

  // broadcast transform

  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "pose";
  transformStamped.transform.translation.x = pose(0, 0);
  transformStamped.transform.translation.y = pose(1, 0);
  transformStamped.transform.translation.z = 0;
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  broadcaster.sendTransform(transformStamped);
}

/**
 * @brief KF prediction
 * 
 * Predicts pose based on odometry data.
 */
void KalmanFilter::PredictPose(const nav_msgs::Odometry *odom_msg)
{
  ros::Duration diff = odom_msg->header.stamp - previousTime; // time passed between last and current callback
  previousTime = odom_msg->header.stamp;
  delta_t = diff.toSec();

  theta = posePrev.at<double>(2, 0); // orientation angle

  B.at<double>(0, 0) = delta_t * cos(theta);
  B.at<double>(1, 0) = delta_t * sin(theta);
  B.at<double>(2, 1) = delta_t;
  B.at<double>(3, 0) = cos(theta);
  B.at<double>(4, 0) = sin(theta);
  B.at<double>(5, 1) = 1;
  B.at<double>(6, 0) = cos(theta)/delta_t;
  B.at<double>(7, 0) = sin(theta)/delta_t;

  double v_x_t = odom_msg->twist.twist.linear.x; // current linear velocity
  double w_t = odom_msg->twist.twist.angular.z;  // current angular velocity
  double data_u[2] = {v_x_t, w_t};

  A.at<double>(6, 3) = -1/delta_t;
  A.at<double>(7, 4) = -1/delta_t;

  cv::Mat_<double> u = cv::Mat(2, 1, CV_64F, data_u); // command u
  cv::Mat_<double> p_t = A * posePrev + B * u;        // pose prediction
  pose = p_t.clone();

cv::Mat_<double> Cov_t = A * cov * A.t() + R; // covariance prediction

}

/**
 * @brief KF correction
 * 
 * Corrects pose based on imu data.
 */
void KalmanFilter::CorrectPose(const sensor_msgs::Imu *imu_msg)
{
  double z, y, x;
  tf2::Quaternion quat;
  tf2::convert(imu_msg->orientation, quat);
  tf2::Matrix3x3 mat(quat);
  mat.getEulerZYX(z, y, x); // convert imu orientation from quarterion to angles - only z actually needed

  measurement(0, 0) = imu_msg->linear_acceleration.x * cos(z); // measured linear acceleration in x direction
  measurement(1, 0) = imu_msg->linear_acceleration.x * sin(z); // measured linear acceleration in y direction
  measurement(2, 0) = imu_msg->angular_velocity.z; // measured angular velocity in z

  cv::Mat temp = C * cov * C.t() + Q;
  K = cov * C.t() * temp.inv(); // define the Kalman gain

  pose = pose + K * (measurement - C * pose);       // pose correction
  cov = (cv::Mat::eye(8, 8, CV_64F) - K * C) * cov; // covariance correction

  posePrev = pose; // save current pose as last pose
}

/**
 * @brief Synchronized odom and imu callback
 * 
 * Calls PredictPose(), CorrectPose() and PublishPose() if localization has been started.
 */
void KalmanFilter::KalmanCallback(const nav_msgs::Odometry::ConstPtr &odomptr,
                          const sensor_msgs::Imu::ConstPtr &imuptr)
{
  const nav_msgs::Odometry &odom_msg = *odomptr;
  const sensor_msgs::Imu &imu_msg = *imuptr;
  if (startLocalisation)
  {
    PredictPose(&odom_msg);
    CorrectPose(&imu_msg);
    PublishPose(&msgCor); // write current data as message
    ROS_INFO_STREAM("Published KF pose");
  }
}

/**
 * @brief Monitoring of scan availability
 * 
 * Stops Kalman fitler localization, resets timer. Calls PublishPose() to publish latest AMCL pose as current pose.
 */
void KalmanFilter::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  startLocalisation = false; // reset flag to stop Kalman filter localisation
  // reset timer; there is no reset function in ros1, so stopping and starting the timer does the trick
  myTimer.stop();
  myTimer.start();
  PublishPose(&msgCor); // write latest data as message
  previousTime = msg->header.stamp;
  ROS_INFO_STREAM("Published copied pose");
}

/**
 * @brief Monitoring of AMCL pose.
 * 
 * Saves latest amcl_pose.
 * If localisation hasn't been started, also calls PublishPose() to publish latest AMCL pose as current pose
 */
void KalmanFilter::PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgptr) // save current pose with covariance provided by ekf_localization_node
{
  const geometry_msgs::PoseWithCovarianceStamped &msg = *msgptr;
  cv::Mat_<double> poseAmclNew = cv::Mat::zeros(8, 1, CV_64F);     // latest amcl pose

  poseAmclNew(0, 0) = msg.pose.pose.position.x;
  poseAmclNew(1, 0) = msg.pose.pose.position.y;
  tf2::Quaternion q;
  tf2::convert(msg.pose.pose.orientation, q); // convert message to quaternion
  poseAmclNew(2, 0) = q.getAngle();          // save orientation along z axis
  if (q.getAxis().z()<0.5) poseAmclNew(2,0) = -poseAmclNew(2,0);

  if (!startLocalisation)
  {
    ros::Duration diff = msg.header.stamp - previousTime; // time passed between last and current callback
    delta_t = diff.toSec();
    poseAmclNew(3, 0) = (poseAmclLast(0, 0) - poseAmclNew(0, 0))/delta_t;
    poseAmclNew(4, 0) = (poseAmclLast(1, 0) - poseAmclNew(1, 0))/delta_t;
    poseAmclNew(5, 0) = (poseAmclLast(2, 0) - poseAmclNew(2, 0))/delta_t;
    poseAmclNew(6, 0) = (poseAmclLast(3, 0) - poseAmclNew(3, 0))/delta_t;
    poseAmclNew(7, 0) = (poseAmclLast(4 ,0) - poseAmclNew(4, 0))/delta_t;
  }
  // save the current covariance (6x6 to 3x3)
  covAmclLast(0, 0) = msg.pose.covariance[0];
  covAmclLast(0, 1) = msg.pose.covariance[1];
  covAmclLast(0, 2) = msg.pose.covariance[5];
  covAmclLast(1, 0) = msg.pose.covariance[6];
  covAmclLast(1, 1) = msg.pose.covariance[7];
  covAmclLast(1, 2) = msg.pose.covariance[11];
  covAmclLast(2, 0) = msg.pose.covariance[30];
  covAmclLast(2, 1) = msg.pose.covariance[31];
  covAmclLast(2, 2) = msg.pose.covariance[35];

  poseAmclLast = poseAmclNew.clone();

  ROS_INFO_STREAM("Copied pose");

  if (!startLocalisation)
  {
    PublishPose(&msgCor); // write current data as message
    previousTime = msg.header.stamp;
  }
}

/**
 * @brief Trigger for KF filter.
 * 
 * Starts Kalman fitler localization.
 * Triggered after 1 second of scan inactivity.
 */
void KalmanFilter::TimerCallback(const ros::TimerEvent &event)
{
  // set flag to start Kalman filter localisation
  if (!startLocalisation) {
    startLocalisation = true;
    ROS_INFO_STREAM("Scan stopped working. Starting Kalman filter...");
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kalman_filter_node");

  KalmanFilter kf;
  ros::spin();

  return 0;
}
