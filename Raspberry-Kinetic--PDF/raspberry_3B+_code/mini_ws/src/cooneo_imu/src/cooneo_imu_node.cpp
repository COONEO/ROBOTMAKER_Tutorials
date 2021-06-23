#include <geometry_msgs/Quaternion.h>       //
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


unsigned char tag=0x00;
unsigned char Angle_initialization[3]={0xFF,0xAA,0x52};     //角度初始化校准指令，使Z轴角度归零
unsigned char Acceleration_calibration[3]={0xFF,0xAA,0x67}; //加速度计校准指令，校准加速度零偏
bool DecodeIMUData(double result[], unsigned char chrTemp[]);

int main(int argc, char** argv)
{
    serial::Serial ser;
    std::string port;
    std::string imu_frame_id;
    int baudrate;


    ros::init(argc, argv, "cooneo_imu_node");

    ros::NodeHandle private_node_handle("~");
    private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");
    private_node_handle.param<int>("baudrate", baudrate, 115200);
    private_node_handle.param<std::string>("imu_frame_id", imu_frame_id, "imu");
    

    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_frame_id, 100);
    geometry_msgs::Quaternion Qua;

    // publish imu message
    sensor_msgs::Imu imu;

    ros::Rate r(200); // 1000 hz

    unsigned char chrBuffer[1000];
    unsigned char chrTemp[1000];
    unsigned short usLength=0,usRxLength=0;

    try
    {
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
        ROS_INFO("Serial port open Successful");
        //校准传感器
        ser.write(Angle_initialization,3);
        ser.write(Acceleration_calibration,3);
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << " ");
        return -1;
    }

    while(ros::ok())
    {
        // read string from serial device  
        // std::cout<<ser.available()<<std::endl;    Serial.availabel() 返回（0,1）
        if(ser.available())
        {  
             //原本imu模块每一帧包含3个数据包,频率为100hz，33个字节，但是由于上下位机同步的原因，采集33个时候，容易掉包，改善方法：提高主程序循环频率或增大每次采集的数据包数目。
            usLength = ser.read(chrBuffer,33);   
            std::cout<<usLength<<std::endl;
            if (usLength>0)
            {
                // ser.flushInput();
                usRxLength += usLength;
                while (usRxLength >= 11)
                {
                    memcpy(chrTemp,chrBuffer,usRxLength);
                    if (!((chrTemp[0] == 0x55) & ((chrTemp[1] == 0x51) | (chrTemp[1] == 0x52) | (chrTemp[1] == 0x53))))
                    {
                        //该循环的目的是：当采集的一系列数据的开头和结尾不符合要求的时候，讲整个数组逐步后移，找到这一帧中有用的数据
                        for (int i = 1; i < usRxLength; i++) chrBuffer[i - 1] = chrBuffer[i];
                        usRxLength-=1;
                        std::cout<<"remove back from data queue "<<std::endl;
                        continue;
                    }

                    double Decode_data[3];
                    if(DecodeIMUData(Decode_data, chrTemp))
                    {
                        imu.header.stamp = ros::Time::now();
                        imu.header.frame_id = imu_frame_id;
                        if(tag==0x51)
                        {
                           imu.linear_acceleration.x=Decode_data[0];
                           imu.linear_acceleration.y=Decode_data[1];
                           imu.linear_acceleration.z=Decode_data[2];
                        }
                        else if(tag==0x52)
                        {
                           imu.angular_velocity.x=Decode_data[0];
                           imu.angular_velocity.y=Decode_data[1];
                           imu.angular_velocity.z=Decode_data[2];
                        }
                        else
                        {
                            double Roll,Pitch,Yall;
                            Roll=Decode_data[0];
                            Pitch=Decode_data[1];
                            Yall=Decode_data[2];
                            Qua=tf::createQuaternionMsgFromRollPitchYaw(Roll,Pitch,Yall);
                            imu.orientation=Qua;
                        }
                    }
                    else
                    {
                        std::cout<<"Decode data Failed!"<<std::endl;
                        continue;
                    }
                    for (int i = 11; i < usRxLength; i++)
                    chrBuffer[i - 11] = chrBuffer[i];
                    usRxLength -= 11;
                }
                imu_pub.publish(imu);
                // ser.flushInput();    //清除输入缓存的数据
                
            }
        }
        else  // ser not available
        {
            std::cout<<"not rev data"<<std::endl;
        }
        ros::spinOnce();
        r.sleep();
    }

}


bool DecodeIMUData(double result[], unsigned char chrTemp[])
{
    int scale;
    if( (chrTemp[1]==0x51)||(chrTemp[1]==0x52)||(chrTemp[1]==0x53))
    {
        switch(chrTemp[1])
        {
        case 0x51: 
            scale = 16*9.8; //a
            tag=0x51;
            break;
        case 0x52: 
            scale = 2000*3.1415926/180; //w  rad/s
            tag=0x52;
            break;
        case 0x53: 
            scale = 3.1415926;//Decode_data rad
            tag=0x53;
            break;
        }
        result[0] = (short(chrTemp[3]<<8|chrTemp[2]))/32768.0*scale;
        result[1] = (short(chrTemp[5]<<8|chrTemp[4]))/32768.0*scale;
        result[2] = (short(chrTemp[7]<<8|chrTemp[6]))/32768.0*scale;
        // for(int i=0;i<3;i++)
        //         std::cout<<result[i]<<" ";
        //     std::cout<<std::endl;

        // double T = (short(chrTemp[9]<<8|chrTemp[8]))/340.0+36.25;
        // printf("aresult = %4.3f\t%4.3f\t%4.3f\t\r\n",result[0],result[1],result[2]);
        return true;
    }
    return false;
}
