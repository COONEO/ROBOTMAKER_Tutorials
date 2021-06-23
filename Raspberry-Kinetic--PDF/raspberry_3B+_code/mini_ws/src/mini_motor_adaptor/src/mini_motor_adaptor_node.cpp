/*
 * Copyright (c) 2020, Cooneo Robot, Inc
 * All rights reserved.
 * Author: Zhenghao Li
 * Date: 2020.7.10
 */

#include <ros/ros.h>
#include <queue>
#include <thread>
#include <serial/serial.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <unistd.h>

// 姿势协方差
boost::array<double, 36> odom_pose_covariance = {
  {1e-3, 0, 0, 0, 0, 0,
   0, 1e-3, 0, 0, 0, 0,
   0, 0, 1e6, 0, 0, 0,
   0, 0, 0, 1e6, 0, 0,
   0, 0, 0, 0, 1e6, 0,
   0, 0, 0, 0, 0, 1e-3}};

// 转动协方差
boost::array<double, 36> odom_twist_covariance = {
  {1e-3, 0, 0, 0, 0, 0,
   0, 1e-3, 0, 0, 0, 0,
   0, 0, 1e6, 0, 0, 0,
   0, 0, 0, 1e6, 0, 0,
   0, 0, 0, 0, 1e6, 0,
   0, 0, 0, 0, 0, 1e-3}};

#define PI 3.1415926 //535
static double Angle;
static bool recv_nav_tag = false;
double rear_odom_correct_param;

struct Vel {
        double linear = 0.0;
        double angle = 0.0;
};

class MotorAdaptor {
public:
ros::NodeHandle n;
ros::Subscriber sub_nav_;
ros::Publisher imu_pub_;
ros::Publisher odom_pub_;
ros::Publisher ultra_F_L_pub_;
ros::Publisher ultra_F_C_pub_;
ros::Publisher ultra_F_R_pub_;
ros::Publisher ultra_L_pub_;
ros::Publisher ultra_R_pub_;


std::queue<Vel> nav_queue_;
serial::Serial ser_car;
serial::Serial ser_ultra;

tf::TransformBroadcaster tb_;
ros::Time lastest_response_time_;
ros::Time lastest_send_time_;

std::string wheel_port;
std::string ultra_port;
int wheel_baudrate;
int ultra_baudrate;
bool use_wheel_port;
bool use_ultra_port;
double Steer_correct_angle;  // 舵机转角矫正参数[0 - 10]
double control_rate;         // 向下位机发送速度的频率

double lastest_linear_ = 0;
double lastest_angle_ = 0;
double odom_linear_x_ = 0;
double odom_linear_y_ = 0;
double odom_angle_ = 0;

double Left_Range =0;
double Right_Range =0;
double Front_Left_Range =0;
double Front_Center_Range =0;
double Front_Right_Range =0;

double vehicle_linear;
double vehicle_angle;
int16_t Encoder_right = 0;
int16_t Encoder_left = 0;
int16_t Steer_angle = 0;
double diff_send;

/******************************  若更换电机类型，则需要核查底盘下面这些参数和超声波的参数 ********************************/
double max_linear_ = 2.5;                                                           //最大线速度 0.5
double max_angle_ = 3.1415926/4.0;                                                  //最大角速度 0.78，
double wheel_distance_ = 0.437;                                                     //左右轮间距 (278 mm + 20 mm *2)
double car_length_ = 0.355;                                                         //后轴中心 与 前轮连杆中心之间的距离: 0.355 m
double wheel_circle_length_ = 3.1415 * 0.125;                                       //轮子直径125 mm
double motor_ppr_ = 13;                                                             //编码器转动一圈 每一相13个脉冲；
double motor_gear_ = 27;                                                            //减速比 27 ；
double pluse_per_meter = (motor_ppr_*motor_gear_) / wheel_circle_length_;           //脉冲数/米 (motor_ppr_*motor_gear_) / wheel_circle_length_

float ultrasonics_max_range = 4.5;
float ultrasonics_min_range = 0.02;
float ultrasonics_field_angle = 0.2616;                                             // 弧度:(PI*15)/180
float PI_calculate_time = 30 /1000;                                                 // 下位机PID执行的时间间隔
/****************************************************************************************************************/

void nav_callback(const geometry_msgs::TwistConstPtr &msg);
bool calcVelocity(uint8_t *, double &, double &, double &Yaw,double diff);
uint8_t calcByteToWrite(int16_t &, int16_t &, int16_t &, Vel & );
uint8_t calUltraTorange(uint8_t *, double &, double &, double &,double &, double &);
uint8_t check_data(uint8_t cmd[],int begin,int end);
void send_thread();
void rec_encoders();
void rec_ultrasonics();
void execute();

MotorAdaptor()
{
        ros::param::get("~/wheel_port",wheel_port);
        ros::param::get("~/ultra_port",ultra_port);
        ros::param::get("~/wheel_baudrate",wheel_baudrate);
        ros::param::get("~/ultra_baudrate",ultra_baudrate);
        ros::param::get("~/use_wheel_port",use_wheel_port);
        ros::param::get("~/use_ultra_port",use_ultra_port);
        ros::param::get("~/Steer_correct_angle",Steer_correct_angle);
        ros::param::get("~/control_rate",control_rate); 
        ros::param::get("~/rear_odom_correct_param",rear_odom_correct_param);

        if(use_wheel_port == true)
        {       
            try
            {
                ser_car.setPort(wheel_port);
                ser_car.setBaudrate(wheel_baudrate);
                serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
                ser_car.setTimeout(timeout);
                ser_car.open();
                ROS_WARN("wheel serial open success!");
            } 
            catch (serial::IOException& e)
            {
                ROS_ERROR_STREAM("Unable to open serial port " << ser_car.getPort() << " ");
                return ;
            }

            sub_nav_ = n.subscribe("cmd_vel", 5, &MotorAdaptor::nav_callback, this);     //smooth_cmd_vel
            odom_pub_ = n.advertise<nav_msgs::Odometry>("wheel_odom", 10);
            // imu_pub_ = n.advertise<sensor_msgs::Imu>("/wheel_imu", 10);

            // 这两个线程是分别控制小车运动和接受小车的数据反馈的。
            std::thread s(&MotorAdaptor::send_thread, this);
            s.detach(); 

            std::thread e(&MotorAdaptor::rec_encoders, this);
            e.detach();
        }
        else
        {
            ROS_INFO("you want to not open wheel control port,please check your launch file!");
        }

        if(use_ultra_port == true)
        {
            try
            {
                ser_ultra.setPort(ultra_port);
                ser_ultra.setBaudrate(ultra_baudrate);
                serial::Timeout timeout2 = serial::Timeout::simpleTimeout(10);
                ser_ultra.setTimeout(timeout2);
                ser_ultra.open();
                ROS_WARN("Ultra serial open success!");
            }
            catch (serial::IOException& e)
            {
                ROS_ERROR_STREAM("Unable to open serial port " << ser_ultra.getPort() << " ");
                return ;
            }

            ultra_L_pub_ = n.advertise<sensor_msgs::Range>("Ultra_L",10);
            ultra_F_L_pub_ = n.advertise<sensor_msgs::Range>("Ultra_F_L",10);
            ultra_F_C_pub_ = n.advertise<sensor_msgs::Range>("Ultra_F_C",10);
            ultra_F_R_pub_ = n.advertise<sensor_msgs::Range>("Ultra_F_R",10);
            ultra_R_pub_ = n.advertise<sensor_msgs::Range>("Ultra_R",10);

            std::thread u(&MotorAdaptor::rec_ultrasonics, this);
            u.detach();
        }
        else
        {
            ROS_INFO("you want to not open ultra sensor control port,please check your launch file!");
        }

        lastest_response_time_ = ros::Time::now();
	    lastest_send_time_ = ros::Time::now();

        ROS_INFO("Initial successfully!");
}
};


/************************************************************************************************************************************
* 下位机收到tag=‘u’后，下位机将超声波原始数据(单位 m)*1000，然后分别分为高、低八位打包，通过串口传输。测距范围<4.5m
*  tx[0]	tx[1]	tx[2]   tx[3]   tx[4]	   tx[5]   tx[6]   tx[7]     tx[8]   tx[9]   tx[10]   tx[11]   tx[12]  tx[13]   共14字节
*  0xff	0xfe	‘u’     左_H8	 左_L8	    前左_H8	 前左_L8  前中_H8	前中_L8	 前右_H8 前右_L8   右_H8    右_L8    异或校验
*
************************************************************************************************************************************/
void MotorAdaptor::rec_ultrasonics()
{
        uint8_t resp[14];
        uint8_t send__cmd[3]={0xff,0xfe,0x75};
        uint8_t temp_header[3];
        ros::Rate r(10);

        sensor_msgs::Range Ultra_L,Ultra_F_L,Ultra_F_C,Ultra_F_R,Ultra_R;

        Ultra_F_L.field_of_view = ultrasonics_field_angle; // 超声波的波束角度
        Ultra_F_L.ULTRASOUND;                           // 超声波和红外测距传感器的标志位
        Ultra_F_L.max_range = ultrasonics_max_range;
        Ultra_F_L.min_range = ultrasonics_min_range;

        Ultra_F_C.field_of_view = ultrasonics_field_angle;
        Ultra_F_C.ULTRASOUND;
        Ultra_F_C.max_range = ultrasonics_max_range;
        Ultra_F_C.min_range = ultrasonics_min_range;

        Ultra_F_R.field_of_view = ultrasonics_field_angle;
        Ultra_F_R.ULTRASOUND;
        Ultra_F_R.max_range = ultrasonics_max_range;
        Ultra_F_R.min_range = ultrasonics_min_range;

        Ultra_L.field_of_view = ultrasonics_field_angle;
        Ultra_L.ULTRASOUND;
        Ultra_L.max_range = ultrasonics_max_range;
        Ultra_L.min_range = ultrasonics_min_range;

        Ultra_R.field_of_view = ultrasonics_field_angle;
        Ultra_R.ULTRASOUND;
        Ultra_R.max_range = ultrasonics_max_range;
        Ultra_R.min_range = ultrasonics_min_range;

        while(ros::ok())
        {
                memset(resp,0,14);
                memset(temp_header,0,3);

                ser_ultra.write(send__cmd,3);                // 发送获取超声波指令给stm32
                if(ser_ultra.read(resp, 14) == 14)
                {
                        // ROS_INFO(" %x %x %x %x %x %x %x %x %x %x %x %x %x %x ",resp[0],resp[1],resp[2],resp[3],resp[4],resp[5],resp[6],resp[7],resp[8],resp[9],resp[10],resp[11],resp[12],resp[13]);
                        if(check_data(resp,3,13)!=resp[13]) // 校验位检测  若出错将不使用数据
                                continue;
                        else
                        {
                                calUltraTorange(resp, Left_Range,Front_Left_Range,Front_Center_Range, Front_Right_Range,Right_Range);

                                Ultra_L.header.frame_id = "Ultra_L_Link";
                                Ultra_L.range = Left_Range;

                                Ultra_F_L.header.frame_id="Ultra_F_L_Link";
                                Ultra_F_L.range = Front_Left_Range;

                                Ultra_F_C.header.frame_id="Ultra_F_C_Link";
                                Ultra_F_C.range = Front_Center_Range;

                                Ultra_F_R.header.frame_id="Ultra_F_R_Link";
                                Ultra_F_R.range = Front_Right_Range;

                                Ultra_R.header.frame_id = "Ultra_R_Link";
                                Ultra_R.range = Right_Range;

                                ROS_INFO("Publish topics");
                                ultra_L_pub_.publish(Ultra_L);
                                ultra_F_L_pub_.publish(Ultra_F_L);
                                ultra_F_C_pub_.publish(Ultra_F_C);
                                ultra_F_R_pub_.publish(Ultra_F_R);
                                ultra_R_pub_.publish(Ultra_R);
                        }
                }
                else
                {
                        ROS_WARN("not recv 14 Byte Ultrasonars data");
                }
                r.sleep();
        }
}


/********************************************************************************************************************************************************
* 当下位机收到 ”0xff,0xfe,'s'“指令后，会发送imu，编码器的数据
* 帧头  帧头  标志位	 tx[0]	  tx[1]	      tx[2]	     tx[3]	 tx[4]   tx[5]	     tx[6]	     tx[7]	     tx[8]	    tx[9]   tx[10]	     tx[11]     校验	    共16字节
* 0xff	0xfe  ‘e’	Y_Angle_H8	Y_Angle_L8	Y_小数点后	Z_w_H8	Z_w_L8	motorA_H8	motorA_L8	motorB_H8	motorB_L8  方向     舵机角度_H8	  舵机角度_L8 异或校验
*
********************************************************************************************************************************************************/
void MotorAdaptor::rec_encoders()
{
        uint8_t resp[16];
        uint8_t encoder_cmd[3]={0xff,0xfe,'e'};
        double vel_x;
        double vel_y;
        double diff;
        double Yaw;
        double average_linear;
        double average_angle;

        ros::Rate r(200);
        ROS_INFO("Start read encoder");
        while(ros::ok())
        {
                nav_msgs::Odometry odom_data;
                geometry_msgs::Quaternion q;
                tf::StampedTransform transform_send;
        
                odom_data.header.frame_id = "wheel_odom";
                odom_data.child_frame_id = "base_link";
        
                transform_send.child_frame_id_ = "base_link";
                transform_send.frame_id_ = "wheel_odom";
                ser_car.write(encoder_cmd,3);                                             //发送获取编码值标志位
                if(ser_car.read(resp, 16) == 16)                                          //读取有效数据位
                {
                        // ROS_INFO("encoder recv: %x, %x, %x, %x ,%x, %x, %x, %x,%x, %x, %x, %x ,%x, %x, %x, %x", resp[0], resp[1], resp[2],resp[3], resp[4], resp[5],resp[6],resp[7],resp[8],resp[9],resp[10],resp[11],resp[12],resp[13],resp[14],resp[15]);

                        ros::Time now = ros::Time::now();
                        diff = (now - lastest_response_time_).toSec();
                        Yaw  =0;
                        if(!calcVelocity(resp, vehicle_linear, vehicle_angle, Yaw,diff))
                                continue;

                        average_linear = (lastest_linear_ + vehicle_linear) / 2;
                        average_angle  = (lastest_angle_ + vehicle_angle) / 2;

                        vel_x = vehicle_linear * std::cos(average_angle);
                        vel_y = vehicle_linear * std::sin(average_angle);

                        odom_angle_ += average_angle * diff;
                        odom_linear_x_ += (average_linear * diff) * std::cos(odom_angle_);
                        odom_linear_y_ += (average_linear * diff) * std::sin(odom_angle_);

                        lastest_linear_ = vehicle_linear;                                        //保留上一次的速度值，与下一次接收的速度做均值计算
                        lastest_angle_ = vehicle_angle;                                          //保留上一次的速度值，与下一次接收的速度做均值计算

                        odom_data.pose.pose.position.x = odom_linear_x_;
                        odom_data.pose.pose.position.y = odom_linear_y_;
                        odom_data.pose.pose.position.z = 0;
                        odom_data.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_angle_);
                        odom_data.pose.covariance = odom_pose_covariance;

                        odom_data.twist.twist.linear.x = vel_x;
                        odom_data.twist.twist.linear.y = vel_y;
                        odom_data.twist.twist.linear.z = 0;
                        odom_data.twist.twist.angular.z = vehicle_angle;
                        odom_data.twist.covariance = odom_twist_covariance;
                        odom_data.header.stamp = ros::Time::now();

                        lastest_response_time_ = now;
                        transform_send.stamp_ = ros::Time::now();
                        tf::Vector3 v(odom_linear_x_, odom_linear_y_, 0);
                        transform_send.setOrigin(v);
                        // transform_send.setRotation(tf::createQuaternionFromYaw(odom_angle_));
                        // tb_.sendTransform(transform_send);   //由于robot_pose_ekf功能包自带base_link-->odom的tf转换，注释掉避免多重接收。
                        odom_pub_.publish(odom_data);
                        // ROS_INFO("recv:  x %f,y %f,angle %f", odom_linear_x_, odom_linear_y_, (odom_angle_*(180.0/3.1415726)));
                }
                else
                {
                        ROS_WARN(" not recv encoder data");
                }
                r.sleep();
        }
}

/***************************************************************************************************************************************
 * 上位机(rasp)------->下位机(stm32)
 * tx[0]  tx[1]	 tx[2]	    tx[3]	    tx[4]	      tx[5]	      tx[6]	      tx[7]          tx[8]          tx[9]   tx[10]
 * oxff	  0xfe   tag	   motorA_H8   motorA_L8	 motorB_H8	 motorB_L8	 Servo_angle_H8	 Servo_angle_L8 方向	 校验
 * ************************************************************************************************************************************/
void MotorAdaptor::send_thread()
{
        uint8_t cmd[] = {0xff, 0xfe, 's', 0, 0, 0, 0, 0, 0, 0, 0};
        ros::Rate r(control_rate);
        uint8_t direction_tag;
        uint16_t left_encoder_temp;
        uint16_t right_encoder_temp;
        uint16_t steer_angle_temp ;

        while(ros::ok())
        {
                ros::Time now = ros::Time::now();
                diff_send = (now - lastest_send_time_).toSec();

                if((nav_queue_.size() != 0) && (recv_nav_tag==true))
                {
                        Vel send = nav_queue_.front();
                        nav_queue_.pop();
                        direction_tag      = calcByteToWrite(Encoder_right, Encoder_left, Steer_angle, send);

                        left_encoder_temp  = (uint16_t)Encoder_left;
                        right_encoder_temp = (uint16_t)Encoder_right;
                        steer_angle_temp = (uint16_t)(Steer_angle  - Steer_correct_angle*10);

                        cmd[3] =  (uint8_t)(left_encoder_temp>>8);   // target_a 高八位
                        cmd[4] =  (uint8_t)(left_encoder_temp);      // target_a 低八位
                        cmd[5] =  (uint8_t)(right_encoder_temp>>8);  // target_b 高八位
                        cmd[6] =  (uint8_t)right_encoder_temp;       // target_b 低八位
                        cmd[7] =  (uint8_t)(steer_angle_temp>>8);    // 前轮转向角度 atan2(y,x) = 求 y/x 的反正切  高八位
                        cmd[8] =  (uint8_t)(steer_angle_temp);       // 前轮转向角度 atan2(y,x) = 求 y/x 的反正切  低八位
                        cmd[9] =  direction_tag;                     // 后轮方向控制位 若(0 0 0 0 0 1 0 0) 左轮向前 ；若(0 0 0 0 0 0 1 0) 右轮向前
                        cmd[10] =  check_data(cmd,3,10);
                        ser_car.write(cmd, 11);

                        std::cout<<left_encoder_temp<<"  "<<right_encoder_temp<<"  "<<steer_angle_temp<<"  "<<direction_tag<<"  "<<std::endl;

                        recv_nav_tag = false;
                }
                else
                {
                        cmd[3] = 0x00;                           // target_a 高八位
                        cmd[4] = 0x00;                           // target_a 低八位
                        cmd[5] = 0x00;                           // target_b 高八位
                        cmd[6] = 0x00;                           // target_b 低八位
                        cmd[7] = (uint8_t)(steer_angle_temp>>8); // 前轮转向角度 5a
                        cmd[8] = (uint8_t)(steer_angle_temp);    // 前轮转向角度 5a
                        cmd[9] = direction_tag;                  // 后轮方向控制位 若0x0F:停车 ； 0x05 ：前进 ; 0x0A:倒车
                        cmd[10] = check_data(cmd,3,10);
                        ser_car.write(cmd, 11);
                }

                direction_tag      = 0x0F;
                left_encoder_temp  = 0.0;
                steer_angle_temp = (9000  - Steer_correct_angle*10);
                right_encoder_temp = 0.0;

                // ROS_INFO("nav send: %x, %x, %x, %x ,%x, %x, %x", cmd[3], cmd[4],cmd[5], cmd[6], cmd[7], cmd[8], cmd[9]);
                r.sleep();
        }
}

/***************************************************************************************************************************************************************************
 * 帧头  帧头  标志位	 tx[0]	  tx[1]	      tx[2]	     tx[3]	 tx[4]   tx[5]	     tx[6]	     tx[7]	     tx[8]	    tx[9]   tx[10]	     tx[11]     校验	    共16字节
 * 0xff	0xfe  ‘e’	Y_Angle_H8	Y_Angle_L8	Y_小数点后	Z_w_H8	Z_w_L8	motorA_H8	motorA_L8	motorB_H8	motorB_L8  方向     舵机角度_H8	  舵机角度_L8 异或校验
 * 检验位：只校验数据域rx[0~10],校验方式为：异或校验
 * 编码器方向位：rx[8] = 0x0F (A、B=0);
 *             rx[8] = 0x05 (A、B>0);
 *             rx[8] = 0x0A (A、B<0);
 *****************************************************************************************************************************************************************************/
bool MotorAdaptor::calcVelocity(uint8_t *rx, double &vehicle_linear, double &vehicle_angle, double &Yaw,double diff)
{
        if(rx[15]==check_data(rx,3,15))                                        //数据帧头正确且校验正确
        {
                uint16_t U_Velocity_A = 0;
                uint16_t U_Velocity_B = 0;
                uint16_t UServo_Ang = 0;
                int16_t Velocity_A = 0x00;
                int16_t Velocity_B = 0x00;

                double v_left;
                double v_right;
                double Servo_Ang;
                double ICR;

                UServo_Ang = ((UServo_Ang|rx[13])<<8)|rx[14]; // 下位机上传舵机角度的范围是[45,135]度

                Servo_Ang = (double)( PI/2.0 - ( ( UServo_Ang / 100.0 ) * (PI/180.0) ) ); // 舵机角度转换成弧度 ，左偏右偏的角度的弧度值
                ICR = car_length_ / tan( Servo_Ang );                   // 向右偏转为负的，向左偏转为正的。 计算整个小车的转弯半径

                U_Velocity_A = ((U_Velocity_A|rx[8])<<8)|rx[9]; //存储在无符号16位中间变量里面
                U_Velocity_B = ((U_Velocity_B|rx[10])<<8)|rx[11];

                Velocity_A = (int16_t)U_Velocity_A;          //存储在有符号16位中间变量里面
                Velocity_B = (int16_t)U_Velocity_B;

                if(rx[12]== 0x05)      // 小车前进
                {
                        v_left = (double)((Velocity_A + rear_odom_correct_param) / 1000.0); //计算左右轮的速度 m/s   + 补偿的速度值
                        v_right = (double)((Velocity_B + rear_odom_correct_param) / 1000.0); //计算左右轮的速度 m/s  + 补偿的速度值
                }
                else if(rx[12] == 0x0A) // 小车后退
                {
                        v_left = -(double)((Velocity_A + rear_odom_correct_param) / 1000.0); //计算左右轮的速度 m/s  + 补偿的速度值
                        v_right = -(double)((Velocity_B + rear_odom_correct_param) / 1000.0); //计算左右轮的速度 m/s + 补偿的速度值
                }
                else                 // 小车停止
                {
                        v_left = 0.0;
                        v_right= 0.0;
                }

                vehicle_linear = (v_left + v_right) / 2;     //计算mini小车后两轮中点处的线速度
                if(UServo_Ang == 90)
                        vehicle_angle = 0;
                else
                        vehicle_angle = vehicle_linear / ICR;  //计算mini小车后两轮中点处的角速度

                //  std::cout<<" Recv------> Left encoder: "<<Velocity_A <<" Right encoder: "<<Velocity_B<<" Servo_Ang : "<<Servo_Ang*(180/PI)<<std::endl;
                return 1;
        }
        else     //检验不正确
        {
                // ROS_INFO("Check error!");
                return 0;
        }
}

/*************************************************************************************************************************
* 上位机(rasp)------->下位机(stm32)
* tx[0]  tx[1]	 tx[2]	    tx[3]	    tx[4]	      tx[5]	      tx[6]	      tx[7]          tx[8]          tx[9]   tx[10]
* oxff	  0xfe   tag	   motorA_H8   motorA_L8	 motorB_H8	 motorB_L8	 Servo_angle_H8	 Servo_angle_L8 方向	 校验
* 编码器方向位：rx[8] = 0x0F (A、B=0);
*             rx[8] = 0x05 (A、B>0);
*             rx[8] = 0x0A (A、B<0);
*  **********************************************************************************************************************/
uint8_t MotorAdaptor::calcByteToWrite(int16_t &Encoder_right, int16_t &Encoder_left, int16_t &Steer_angle, Vel &v )
{
        double V_left = 0.0;
        double V_right = 0.0;
        double V_angle = 0.0;

        if(v.angle!=0)                                                  //********* 以一定的角度前进或倒退 ************
        {
                V_angle = std::atan2((v.angle*car_length_),fabs(v.linear)); // 前轮转向角度 atan2(y,x) = 求 y/x 的反正切
                ROS_INFO("Calculate Steer Angle: %lf",V_angle);
                if(V_angle>max_angle_)
                {
                        V_angle = max_angle_;
                        ROS_INFO("Steer angle out range");
                }
                else if(V_angle<-max_angle_)
                {
                        V_angle = -max_angle_;
                        ROS_INFO("Steer angle out range");
                }
                if(v.linear<0)
                {
                        V_left = v.linear*(1+tan(V_angle)/2.0);         // 小车倒车的时候，结算相反带角速度的运动学解算
                        V_right = v.linear*(1-tan(V_angle)/2.0);        // m/s
                        V_angle = -V_angle;
                }
                else
                {
                        V_left = v.linear*(1-tan(V_angle)/2.0);         // 带角速度的运动学解算
                        V_right = v.linear*(1+tan(V_angle)/2.0);        // m/s
                }
        }
        else                                                      // *********  角度为0 直行 *********
        {
                V_angle = 0.0;
                V_left = v.linear;
                V_right = v.linear;                               // m/s
        }


        Encoder_left = (int16_t)fabs(V_left * 1000);              // 此处除以 下位机 30ms读取一次 编码值，然后执行PI
        Encoder_right = (int16_t)fabs(V_right * 1000);             // 计算下位机30ms内应有的速度
        Steer_angle = (int16_t)( 9000 - (V_angle * (180/PI))*100.0);

        // std::cout<<"Send----> Encoder_left: "<<Encoder_left<<"  Encoder_right:  "<<Encoder_right<<" V_angle: "<<V_angle<<std::endl;

        uint8_t motor_direction_tag = 0x00;
        if(v.linear<0)
                motor_direction_tag = 0x0A;                       // 倒车
        else if(v.linear>0)
                motor_direction_tag = 0x05;                       // 前进
        else
                motor_direction_tag = 0x0F;

        return motor_direction_tag;
}

/**************************************************************************************************************************************
* 超声波距离换算函数，stm32传输上来的超声波数据是：原始数据(单位 m)*1000,然后存储为高低八位
* 下位机收到tag=‘u’后，下位机将超声波原始数据(单位 m)*1000，然后分别分为高、低八位打包，通过串口传输。测距范围<4.5m
*  tx[0]	tx[1]	tx[2]   tx[3]   tx[4]	   tx[5]   tx[6]   tx[7]     tx[8]   tx[9]   tx[10]   tx[11]   tx[12]  tx[13]   共10字节
*  0xff	0xfe	‘u’     左_H8	 左_L8	    前左_H8	 前左_L8  前中_H8	前中_L8	 前右_H8 前右_L8   右_H8    右_L8    异或校验
**************************************************************************************************************************************/
uint8_t MotorAdaptor::calUltraTorange(uint8_t *rx, double &Left_Range, double &Front_Left_Range, double &Front_Center_Range, double &Front_Right_Range,double &Right_Range)
{
        uint16_t Temp_Range_L = 0;
        uint16_t Temp_Range_F_L = 0;
        uint16_t Temp_Range_F_C = 0;
        uint16_t Temp_Range_F_R = 0;
        uint16_t Temp_Range_R = 0;

        Temp_Range_L = ((Temp_Range_L|rx[3])<<8)|rx[4];
        Temp_Range_F_L = ((Temp_Range_F_L|rx[5])<<8)|rx[6];  // 移位获得串口传输的超声波数据，无符号16位整型
        Temp_Range_F_C = ((Temp_Range_F_C|rx[7])<<8)|rx[8];
        Temp_Range_F_R = ((Temp_Range_F_R|rx[9])<<8)|rx[10];
        Temp_Range_R = ((Temp_Range_R|rx[11])<<8)|rx[12];

        Left_Range = (float)Temp_Range_L/1000.0;
        Front_Left_Range = (float)Temp_Range_F_L/1000.0;     // 除以1000，转换数据
        Front_Center_Range = (float)Temp_Range_F_C/1000.0;
        Front_Right_Range = (float)Temp_Range_F_R/1000.0;
        Right_Range = (float)Temp_Range_R/1000.0;

        //std::cout<<"Temp_Range_F_L: "<<Temp_Range_F_L<<" Temp_Range_F_C: "<<Temp_Range_F_C<<" Temp_Range_F_R: "<<Temp_Range_F_R<<std::endl;
        //std::cout<<"Ranger_L: "<<Left_Range<<" Front_Ranger_L: "<<Front_Left_Range<<" Front_Ranger_C: "<<Front_Center_Range<<" Front_range_right "<<Front_Right_Range<<" Range_R "<<Right_Range<<std::endl;

}

void MotorAdaptor::execute()
{
        ros::spin();
}

void MotorAdaptor::nav_callback(const geometry_msgs::TwistConstPtr &msg)
{
        recv_nav_tag = true;
        Vel push;
        if(msg->linear.x == 0)
                return ;
        push.linear = msg->linear.x;
        push.angle = msg->angular.z;
        if(nav_queue_.size() > 2)                          //若队列满了，就会移除队头元素，然后
                nav_queue_.pop();
        nav_queue_.push(push);
}

/*************************************
* 函数名: 异或校验函数
* 参数:   需要校验的数组 ， 数组长度
* 返回值： 异或校验数值
*************************************/
uint8_t MotorAdaptor::check_data(uint8_t cmd[],int begin,int end)
{
        int i;
        uint8_t check=cmd[begin];

        for(i = begin+1; i<end; i++)
                check^=cmd[i];

        // ROS_INFO("check: 0x%X",check);
        return check;
}

int main(int argc, char *argv[])
{
        ros::init(argc, argv, "motor_adaptor_node");
        MotorAdaptor ma;
        ma.execute();
        return 0;
}
