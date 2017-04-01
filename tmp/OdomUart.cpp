#include "OdomUart.h"
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
#include<sys/ioctl.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define pi 3.1415926
namespace carto_release {
namespace ODOM {

//using namespace std;

//OdomUart类初始化
OdomUart::OdomUart():uartShutDownFlag(false), uartFd_odom(-1), uartThread_odom(NULL)
{
    OdomData.X = 0.0;
    OdomData.Y = 0.0;
    OdomData.Theta = 0.0;
    OdomData. frameIndex= -1;
    OdomData. timestamp.tv_sec= 0;
    OdomData. timestamp.tv_nsec= 0;

    //Use to handle odom data!
    if(!uartInit("/dev/ttyUSB1", 230400, 8, 1, 'n', 'n'))
    {
        cerr<<"Open Odom Collecter uart failed!"<<endl;
    }
}

OdomUart::~OdomUart()
{
    OdomUart::uartClose();
}

//串口初始化
bool OdomUart::uartInit(const char* comPort, int baudRate, int dataBits, int stopBits, const char parity,const char streamControl)
{

    uartFd_odom = open(comPort, O_RDWR|O_NOCTTY|O_NONBLOCK);//O_NDELAY);
    
    //uartFd = open(comPort, O_RDWR|O_NOCTTY);//O_NDELAY);
    int fd_odom = uartFd_odom;

    //save uart sets to local viriable
    uartProtocol.baudRate = baudRate;
    uartProtocol.dataBits = dataBits;
    uartProtocol.stopBits = stopBits;
    uartProtocol.parity = parity;
    uartProtocol.streamControl = streamControl;

    if(uartFd_odom == -1)
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


    if  ( tcgetattr(fd_odom,&options)  !=  0)
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

        tcflush(fd_odom,TCIFLUSH);
        if (fcntl(fd_odom, F_SETFL, FNDELAY) != 0)
        {
            uartClose();
            return false;
        }
        //激活配置 (将修改后的termios数据设置到串口中）
        if (tcsetattr(fd_odom,TCSANOW,&options) != 0)
        {
            perror("com set error!\n");
            return false;
        }
    
    return true;
}

void OdomUart::uartClose(void)
{
    {
        unique_lock<mutex> lck(uartShutDownMutex);
        uartShutDownFlag = true;

        if(uartFd_odom != -1)
        {
            close(uartFd_odom);
        }
    }

    if(uartThread_odom != NULL)
        uartThread_odom->join();
}

void OdomUart::datacollect(void)
{
    uartThread_odom = boost::make_shared<boost::thread>(boost::bind(&OdomUart::datareceive_odom, this ) );
}

void OdomUart::datareceive_odom(void)
{
    lds_response_measurement_node_odom_t local_buf_odom[1];
    memset(local_buf_odom, 0, sizeof(local_buf_odom));
    //cerr<<"hihihihi"<<endl;

    while(1)
    {
	  {
            unique_lock<mutex> lck(uartShutDownMutex);
            if(uartShutDownFlag == true) break;
        }
         flag_tt = false;
         lds_response_measurement_node_odom_t node_odom;
         clock_gettime(CLOCK_REALTIME, &start_odom);
         
         waitNode_odom(&node_odom);
         //node_odom.timestamp = start_odom;
         local_buf_odom[0] = node_odom;

         //if (local_buf_odom[0].sync_1 == 0x5A && local_buf_odom[0].sync_2 == 0xA5 &&  local_buf_odom[0].sync_3 == 0xFF &&  local_buf_odom[0].sync_4 == 0xFF )
         {
                unique_lock<mutex> lck(receivedateMutex);
                //cerr<<"hihi"<<endl;
                memcpy(_cached_scan_node_buf_odom, local_buf_odom, sizeof(lds_response_measurement_node_odom_t)); 
                flag_tt = true;               
         }
         usleep(100);
    }
}

int OdomUart::waitNode_odom(lds_response_measurement_node_odom_t *node_odom)
{
       int  recvPos = 0;
       _u8  recvBuffer[sizeof(lds_response_measurement_node_odom_t)];
       _u8 *nodeBuffer_odom = (_u8*)node_odom;

       
       while(1)
       {
		   {
            unique_lock<mutex> lck(uartShutDownMutex);
            if(uartShutDownFlag == true) break;
           }
           int remainSize = sizeof(lds_response_measurement_node_odom_t) - recvPos;
           int recvSize;

           waitfordata(remainSize,  &recvSize);
           if (recvSize > remainSize) recvSize = remainSize;
           recvdata(recvBuffer, recvSize);
           //cerr<<"hihihihi"<<endl;
           for (int pos = 0; pos < recvSize; ++pos)
           {
               _u8 currentByte = recvBuffer[pos];
               switch (recvPos) {
               case 0: // expect the sync bit and its reverse in this byte
                   {
                       //_u8 tmp = (currentByte>>1);
                       if ( currentByte == 0x5A) {
                           // pass
                       } else {
                           continue;
                       }
                   }
                   break;
               case 1: // expect the highest bit to be 1
                   {
                       if (currentByte == 0xA5 ) {
                           // pass
                       } else {
                           recvPos = 0;
                           continue;
                       }
                   }
                   break;
               }
               nodeBuffer_odom[recvPos++] = currentByte;
           }

           if (recvPos == sizeof(lds_response_measurement_node_odom_t))
           {
               return 0;
           }
        }
}

int OdomUart::waitfordata(int data_count, int * returned_size)
{
    int length = 0;
    if (returned_size==NULL) returned_size=(int *)&length;
    *returned_size = 0;

    fd_set input_set;
    FD_ZERO(&input_set);

    FD_SET(uartFd_odom, &input_set);
    if ( ioctl(uartFd_odom, FIONREAD, returned_size) == -1) return -1;
    if (*returned_size >= data_count)
    {
         return 0;
    }
    while(1)
    {
         assert (FD_ISSET(uartFd_odom, &input_set));
         if ( ioctl(uartFd_odom, FIONREAD, returned_size) == -1) return -1;
         if (*returned_size >= data_count)
         {
              return 0;
         }
         else
         {
              usleep(300);
         }
    }
    return -1;
}

int OdomUart::recvdata(unsigned char * data, int size)
{
   read(uartFd_odom, data, size);
}

void OdomUart::grabScanData(lds_response_measurement_node_odom_t* nodebuffer_odom, int & count_odom)
{
    //while(!flag_tt){}
    //flag_tt = false;
    {
       unique_lock<mutex> lck(OdomDataMutex);
       memcpy(nodebuffer_odom, _cached_scan_node_buf_odom, count_odom*sizeof(lds_response_measurement_node_odom_t));
    }
}

void OdomUart::ascendScanData(lds_response_measurement_node_odom_t* nodebuffer_odom, int  count_odom)
{
    int i = 0;
    //Tune head
    unique_lock<mutex> lck(OdomDataMutex);

    OdomData.X = nodebuffer_odom[0].X/1000.;
    OdomData.Y = nodebuffer_odom[0].Y/-1000.;
    OdomData.Theta = nodebuffer_odom[0].Theta/-1000.;
    OdomData.timestamp = start_odom;
}

Odom_Data OdomUart::getCurrentOdom(void)
{
    unique_lock<mutex> lck(OdomDataMutex);
    return OdomData;
}

}
}
