#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include <string>

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_msgs/String.h"
#include <boost/thread.hpp>

using pose_t = geometry_msgs::PoseWithCovarianceStamped;

pose_t pose_msg;
ros::Publisher pose_pub;


void runBehaviorThread(void)
{
    ros::NodeHandle n;
    ros::Rate r(5.0);

    while (n.ok()) {
	r.sleep();
    }
}

int main(int argc, char **argv)
{
  double position_x = 0.0;
  double position_y = 0.0;
  double position_z = 0.0;

  double q_x = 0.0;
  double q_y = 0.0;
  double q_z = 0.0;
  double q_w = 0.0;

  ros::init(argc, argv, "pose_initial");

  ros::NodeHandle n("~");
  std::cout << "pose_initial: start" << std::endl;

  n.param("pos_x", position_x, 0.0);
  n.param("pos_y", position_y, 0.0);
  n.param("pos_z", position_z, 0.0);

  n.param("q_x", q_x, 1.4035);
  n.param("q_y", q_y, 1.4035);
  n.param("q_z", q_z, 1.4035);
  n.param("q_w", q_w, 1.4035);

  pose_pub = n.advertise<pose_t>("/initialpose", 1);

  //pose_t pose_msg;
  pose_msg.header.stamp = ros::Time::now();

  pose_msg.pose.pose.position.x = position_x;
  pose_msg.pose.pose.position.y = position_y;
  pose_msg.pose.pose.position.z = position_z;

  pose_msg.pose.pose.orientation.x = q_x;
  pose_msg.pose.pose.orientation.y = q_y;
  pose_msg.pose.pose.orientation.z = q_z;
  pose_msg.pose.pose.orientation.w = q_w;

  for (int i = 0; i < 36; ++i)
  {
    pose_msg.pose.covariance[i] = 0;
  }

  pose_msg.pose.covariance[0] = 0.02;
  pose_msg.pose.covariance[6 + 1] = 0.02;
  pose_msg.pose.covariance[6 * 5 + 5] = 0.02;

  pose_msg.header.frame_id = "map";

  ros::Rate r(8.0);

  for (int i = 0; i < 4; i++)
  {
    pose_pub.publish(pose_msg);

    r.sleep();
  }

//  reset_monitor_thread_ = new boost::thread(&runBehaviorThread);

  ros::spin();

  return 0;
}
