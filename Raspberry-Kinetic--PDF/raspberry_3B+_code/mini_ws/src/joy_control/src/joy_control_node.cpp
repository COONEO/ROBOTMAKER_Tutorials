#include <ros/ros.h>
#include<queue>
#include<sensor_msgs/Joy.h>
#include<geometry_msgs/Twist.h>

struct Velocity{
    double linear = 0;
    double angle = 0;
};

Velocity joy_vel;
static bool tag = false;
const int max_vel = 1;

void joy_callback(const sensor_msgs::JoyConstPtr& msg)
{
    joy_vel.linear = msg->axes[1] * max_vel*0.30;
    joy_vel.angle = msg->axes[3] * max_vel*1.0;  
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"joy_control_node");
    ros::NodeHandle n;
    ros::NodeHandle p;
    ros::Subscriber joy_sub;
    ros::Publisher cmd_pub;
    joy_sub = n.subscribe("/joy",1,&joy_callback);
    cmd_pub = p.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    geometry_msgs::Twist car_vel;

    ROS_INFO("start to joy control");
    ros::Rate r(100);
    while(ros::ok())
    {
        car_vel.linear.x = joy_vel.linear;
        car_vel.linear.y = 0.0;
        car_vel.linear.z = 0.0;
        car_vel.angular.z = joy_vel.angle;
        car_vel.angular.y = 0.0;
        car_vel.angular.x = 0.0;

        cmd_pub.publish(car_vel);
        ros::spinOnce();
        r.sleep();
    }

    ros::shutdown();
    return 0;

}

