#include "LaserUart.h"
#include "sys/time.h"
#include <time.h>
#include <sstream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sys/ioctl.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define pi 3.1415926
namespace carto_release {
namespace LASER {

//using namespace LASER;
//using namespace std;
//using namespace boost;

//LaserUart类初始化
LaserUart::LaserUart():uartShutDownFlag(false), uartFd_laser(-1), uartThread_laser(NULL)
{
    LaserData.timestamp.tv_sec = 0;
    LaserData.timestamp.tv_nsec = 0;
    LaserData.angle_min_ = 0;
    LaserData.angle_max_ = 2*pi;
    LaserData.angle_increment_ = pi/180;
    LaserData.scan_time_ = 0.2;
    LaserData.time_increment_ = 0.2/359;
    LaserData.range_min_ = 0.3;
    LaserData.range_max_ = 3.1;

    for(int i=0; i<360; i++)
    {
        LaserData.Distance[i] = 0.0;
        LaserData.Angle[i] = 0.0;
    }
    for(int i=0; i<42; i++)
    {
        nodeleave[i] = 0;
    }

    mm = 0xA1;
    kk = 0;
    leave = false;
    flag_1 = 0;
    flag_2 = 0;
    count_node = 0;

    data2compensate.sync = 0xFA;
    data2compensate.angle_index = 0xA0;
    data2compensate.RPM = 0;
    data2compensate.intensity_0 = 0;
    data2compensate.distance_0= 0;
    data2compensate.reserved_0= 0;
    data2compensate.intensity_1= 0;
    data2compensate.distance_1= 0;
    data2compensate.reserved_1= 0;
    data2compensate.intensity_2= 0;
    data2compensate.distance_2= 0;
    data2compensate.reserved_2= 0;
    data2compensate.intensity_3= 0;
    data2compensate.distance_3= 0;
    data2compensate.reserved_3= 0;
    data2compensate.intensity_4= 0;
    data2compensate.distance_4= 0;
    data2compensate.reserved_4= 0;
    data2compensate.intensity_5= 0;
    data2compensate.distance_5= 0;
    data2compensate.reserved_5= 0;
    data2compensate.checksum= 0;

    //Use to handle laser data!
    if(!uartInit("/dev/ttyUSB0", 230400, 8, 1, 'n', 'n'))
    {
        cerr<<"Open Laser Collecter uart failed!"<<endl;
    }
}

LaserUart::~LaserUart()
{
    LaserUart::uartClose();
}

//串口初始化
bool LaserUart::uartInit(const char* comPort, int baudRate, int dataBits, int stopBits, const char parity,const char streamControl)
{
    uartFd_laser = open(comPort, O_RDWR|O_NOCTTY|O_NONBLOCK);//O_NDELAY);
    
    //uartFd = open(comPort, O_RDWR|O_NOCTTY);//O_NDELAY);
    int fd_laser = uartFd_laser;

    //save uart sets to local viriable
    uartProtocol.baudRate = baudRate;
    uartProtocol.dataBits = dataBits;
    uartProtocol.stopBits = stopBits;
    uartProtocol.parity = parity;
    uartProtocol.streamControl = streamControl;

    if(uartFd_laser == -1)
    {
        cerr<<"open port "<<comPort<<" failed"<<endl;
        return false;
    }

    //config uart protocol
    int   i;
    int   speed_arr[] = { B230400, B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = { 230400, 115200,  19200,  9600,  4800,  2400,  1200,  300};
    struct termios options;

    //存储目前的序列埠设定
    if  ( tcgetattr( fd_laser,&options)  !=  0)
    {
        cerr<<"error get uart attrib"<<endl;
        return false;
    }

    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
        if  (baudRate == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch(streamControl)
    {

    case 'n' ://不使用流控制
    case 'N':
        options.c_cflag &= ~CRTSCTS;
        break;

    case  'h'://使用硬件流控制
    case 'H':
        options.c_cflag |= CRTSCTS;
        break;
    case 's' ://使用软件流控制
    case 'S':
        options.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (dataBits)
    {
    case 5    :
        options.c_cflag |= CS5;
        break;
    case 6    :
        options.c_cflag |= CS6;
        break;
    case 7    :
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr,"Unsupported data size\n");
        return false;
    }
    //设置校验位
    switch (parity)
    {
    case 'n':
    case 'N': //无奇偶校验位。
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        break;
    case 'o':
    case 'O'://设置为奇校验
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E'://设置为偶校验
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        break;
    case 's':
    case 'S': //设置为空格
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        fprintf(stderr,"Unsupported parity\n");
        return false;
    }
    // 设置停止位
    switch (stopBits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB; break;
    case 2:
        options.c_cflag |= CSTOPB; break;
    default:
        fprintf(stderr,"Unsupported stop bits\n");
        return false;
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//original data input
    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON); //iflag not lflag

    //设置等待时间和最小接收字符
//    options.c_cc[VTIME] = 0; /* 读取一个字符等待1*(1/10)s */
//    options.c_cc[VMIN] = 0; /* 读取字符的最少个数为1 */ //13*2 byte

    //如果发生数据溢出，接收数据，但是不再读取, 刷新收到的数据但是不读

    tcflush(fd_laser,TCIFLUSH);
    if (fcntl(fd_laser, F_SETFL, FNDELAY) != 0)
    {
        uartClose();
        return false;
    }
    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd_laser,TCSANOW,&options) != 0)
    {
       perror("com set error!\n");
       return false;
    }

    return true;
}

void LaserUart::uartClose(void)
{
    {
        boost::unique_lock<boost::mutex> lk(uartShutDownMutex);
        uartShutDownFlag = true;
        if(uartFd_laser != -1)
        {
            close(uartFd_laser);
        }
        lk.unlock( );
    }
    if(uartThread_laser != NULL)
        uartThread_laser->join();

}

void LaserUart::datacollect(void)
{
    uartThread_laser = boost::make_shared<boost::thread>(boost::bind(&LaserUart::datareceive_laser, this));
}

void LaserUart::datareceive_laser(void)
{
    lds_response_measurement_node_t      local_buf[60];
    memset(local_buf, 0, sizeof(local_buf));
    char buff[1]={'b'};
    int fd_laser = uartFd_laser;
    
    write(fd_laser,buff,1);
    while(1)
    {
//	   {
//            boost::unique_lock<boost::mutex> lk(uartShutDownMutex);
            if(uartShutDownFlag == true)
            {
//                lk.unlock();
                break;
            }
//        }
         int sequence = 0;
         flag_1 = 0;
         flag_2 = 0;
	     flag_a0 = false;
         count_node = 0;
         lds_response_measurement_node_t node;
         //set the 360 points pre values!
         for(int i=0; i<60; i++)
         {
             data2compensate.angle_index = 0xA0 + i;
             local_buf[i] = data2compensate;
         }
         data2compensate.angle_index = 0xA0;

         clock_gettime(CLOCK_REALTIME, &start_laser);    
         while ( !waitNode(&node))
         {
                count_node += 1;
                sequence = node.angle_index - 0xA0;
                local_buf[sequence] = node;
         }
         clock_gettime(CLOCK_REALTIME, &end_laser);
         {
               // only publish the data when it contains a full 360 degree scan
               boost::unique_lock<boost::mutex> lk(receivedateMutex);
                //clock_gettime(CLOCK_REALTIME, &start_laser);
                memcpy(_cached_scan_node_buf, local_buf, 60*sizeof(lds_response_measurement_node_t));
                lk.unlock();
         }
         flag_a0 = true;
         usleep(1000);
       }
}

int LaserUart::waitNode(lds_response_measurement_node_t *node)
{
       int  recvPos = 0;
       _u8  recvBuffer[sizeof(lds_response_measurement_node_t)];
       _u8 *nodeBuffer = (_u8*)node;

       bool flag_odom = false;
       int remainSize = 0;
       if(leave == true)
       {
           recvPos = kk;
       }

       while(1)
       {
//		  {
//            boost::unique_lock<boost::mutex> lk(uartShutDownMutex);
            if(uartShutDownFlag == true)
            {
//                lk.unlock();
                break;
            }
//          }
           remainSize = sizeof(lds_response_measurement_node_t) - recvPos;
           int recvSize;
           waitfordata(remainSize, &recvSize);
           if (recvSize > remainSize) recvSize = remainSize;
           recvdata(recvBuffer, recvSize);

           if(leave == true)
           {
               memcpy(&recvBuffer[kk],recvBuffer,recvSize);
               memcpy(recvBuffer,nodeleave,kk);

               kk = 0;
               leave = false;
               recvSize = sizeof(lds_response_measurement_node_t);
               recvPos = 0;
           }

           for (int pos = 0; pos < recvSize; ++pos)
           {
                _u8 currentByte = recvBuffer[pos];
                //cerr<<"I know you!"<<endl;
                switch (recvPos) {
                case 0:
               {
                   if (currentByte == 0xFA){}
                   else continue;
                }
                break;
                case 1:
                {
                       if(currentByte >=0xA0 && currentByte <= 0xDB)
                       {
                            if(currentByte == 0xA0)
                            {
                                flag_1 += 1;
                                if(flag_1 == 2)
                                {
                                    kk = recvSize;
                                    leave = true;

                                    nodeleave[0] = 0xFA;
                                    nodeleave[1] = 0xA0;
                                    for(int count=0; count<kk; count++)
                                    {
                                         nodeleave[count+2] =recvBuffer[pos++];
                                    }
                                    return 1;
                                }

                            }

                            if(currentByte == 0xA1)
                            {
                                flag_2 += 1;
                                if(flag_2 == 2)
                                {
                                    kk = recvSize - 2;
                                    leave = true;
                                    nodeleave[0] = 0xFA;
                                    nodeleave[1] = 0xA1;
                                    for(int count=0; count<kk; count++)
                                    {
                                         nodeleave[count+2] =recvBuffer[pos++];
                                    }
                                    return 1;
                                }
                            }
                       }

                       else
                       {
                           recvPos = 0;
                           continue;
                       }
                   }
                   break;
               }
               nodeBuffer[recvPos++] = currentByte;
           }

           if (recvPos == sizeof(lds_response_measurement_node_t))
           {
                return 0;
           }
        }
}

int LaserUart::waitfordata(int data_count, int * returned_size)
{
    int length = 0;
    if (returned_size==NULL) returned_size=(int *)&length;
    *returned_size = 0;

    fd_set input_set;
    FD_ZERO(&input_set);
    
        FD_SET(uartFd_laser, &input_set);
        if ( ioctl(uartFd_laser, FIONREAD, returned_size) == -1) return -1;
        if (*returned_size >= data_count)
        {
            return 0;
        }

        while(1)
        {
            if(uartShutDownFlag == true)
            {
                break;
            }
            assert (FD_ISSET(uartFd_laser, &input_set));
            if ( ioctl(uartFd_laser, FIONREAD, returned_size) == -1) return -1;
            if (*returned_size >= data_count)
            {
                return 0;
            }
            else
            {
                usleep(3000);
            }
        }
    return -1;
}

int LaserUart::recvdata(unsigned char * data, int size)
{
    read(uartFd_laser, data, size);
}

void LaserUart::grabScanData(lds_response_measurement_node_t* nodebuffer, int & count)
{
    while(!flag_a0) {}
    flag_a0 = false;
    {
       boost::unique_lock<boost::mutex> lk(LaserDataMutex);
       memcpy(nodebuffer, _cached_scan_node_buf, count*sizeof(lds_response_measurement_node_t));
       lk.unlock( );
    }

}

void LaserUart::ascendScanData(lds_response_measurement_node_t* nodebuffer, int  count)
{
    int i = 0;
    //Tune head
    boost::unique_lock<boost::mutex> lk(LaserDataMutex);
    for (i = 0; i < count; i++)
    {
        LaserData.Distance[359- 6*i] = nodebuffer[i].distance_0/1000.0;
        LaserData.Distance[359- 6*i- 1] = nodebuffer[i].distance_1/1000.0;
        LaserData.Distance[359- 6*i- 2] = nodebuffer[i].distance_2/1000.0;
        LaserData.Distance[359- 6*i- 3] = nodebuffer[i].distance_3/1000.0;
        LaserData.Distance[359- 6*i- 4] = nodebuffer[i].distance_4/1000.0;
        LaserData.Distance[359- 6*i- 5] = nodebuffer[i].distance_5/1000.0;
    }
    LaserData.timestamp = start_laser;
//    cerr<<"I have receive these data"<<endl;
    lk.unlock( );
}

Laser_Data LaserUart::getCurrentLaser(void)
{
    boost::unique_lock<boost::mutex> lk(LaserDataMutex);
    return LaserData;
    lk.unlock( );
}

}// namespace LASER
}// namespace carto_release

