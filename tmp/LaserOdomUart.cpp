#include "LaserOdomUart.h"
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
namespace LASER_ODOM{
using namespace std;

//LaserOdomUart类初始化
LaserOdomUart::LaserOdomUart():uartShutDownFlag(false), uartFd_laser(-1),   uartFd_odom(-1), uartThread_laser(NULL), uartThread_odom(NULL)
{
    OdomData.X = 0.0;
    OdomData.Y = 0.0;
    OdomData.Theta = 0.0;
    OdomData. frameIndex= -1;
    OdomData. timestamp.tv_sec= 0;
    OdomData. timestamp.tv_nsec= 0;
    theta[0] = 0;
    theta[1] = 0;

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
    if(!uartInit("/dev/ttyUSB0", 230400, 8, 1, 'n', 'n', 0))
    {
        cerr<<"Open Laser Collecter uart failed!"<<endl;
    }

    //Use to handle odom data!
    if(!uartInit("/dev/ttyUSB1", 230400, 8, 1, 'n', 'n', 1))
    {
        cerr<<"Open Odom Collecter uart failed!"<<endl;
    }
}

LaserOdomUart::~LaserOdomUart()
{
    LaserOdomUart::uartClose();
}

//串口初始化
bool LaserOdomUart::uartInit(const char* comPort, int baudRate, int dataBits, int stopBits, const char parity,const char streamControl, int flag_lo)
{
    if(!flag_lo)
    {
        uartFd_laser = open(comPort, O_RDWR|O_NOCTTY|O_NONBLOCK);//O_NDELAY);
    }
    else
    {
        uartFd_odom = open(comPort, O_RDWR|O_NOCTTY|O_NONBLOCK);//O_NDELAY);
    }
    //uartFd = open(comPort, O_RDWR|O_NOCTTY);//O_NDELAY);
    int fd_laser = uartFd_laser;
    int fd_odom = uartFd_odom;

    //save uart sets to local viriable
    uartProtocol.baudRate = baudRate;
    uartProtocol.dataBits = dataBits;
    uartProtocol.stopBits = stopBits;
    uartProtocol.parity = parity;
    uartProtocol.streamControl = streamControl;

    if(uartFd_laser == -1&& !flag_lo)
    {
        cerr<<"open port "<<comPort<<" failed"<<endl;
        return false;
    }

    if(uartFd_odom == -1&& flag_lo)
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
    if  ( tcgetattr( fd_laser,&options)  !=  0 && !flag_lo)
    {
        cerr<<"error get uart attrib"<<endl;
        return false;
    }

    if  ( tcgetattr(fd_odom,&options)  !=  0 && flag_lo)
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
    if(!flag_lo)
    {
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
    }
    else
    {
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
    }
    return true;
}

void LaserOdomUart::uartClose(void)
{
    {
        unique_lock<mutex> lck(uartShutDownMutex);
        uartShutDownFlag = true;
        if(uartFd_laser != -1)
        {
            close(uartFd_laser);
        }

        if(uartFd_odom != -1)
        {
            close(uartFd_odom);
        }
    }
    if(uartThread_laser != NULL)
        uartThread_laser->join();
    if(uartThread_odom != NULL)
        uartThread_odom->join();
}

void LaserOdomUart::datacollect(void)
{
    uartThread_laser = boost::make_shared<boost::thread>(boost::bind(&LaserOdomUart::datareceive_laser, this ) );
//    uartThread_odom = boost::make_shared<boost::thread>(boost::bind(&LaserOdomUart::datareceive_odom, this ) );
}

void LaserOdomUart::datareceive_laser(void)
{
    lds_response_measurement_node_t      local_buf[60];
    lds_response_measurement_node_odom_t local_buf_odom[1];

    memset(local_buf, 0, sizeof(local_buf));
    memset(local_buf_odom, 0, sizeof(local_buf_odom));
    int sequence = 0;
    flag_1 = 0;
    flag_2 = 0;
    flag_a0 = false;
    lds_response_measurement_node_t node;
    lds_response_measurement_node_odom_t node_odom;

    char buff[1]={'b'};
    int fd_laser = uartFd_laser;
    write(fd_laser,buff,1);
    while(1)
    {
		 {
            unique_lock<mutex> lck(uartShutDownMutex);
            if(uartShutDownFlag == true) break;
          }

        waitNode_odom(&node_odom);
        local_buf_odom[0] = node_odom;

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
                sequence = node.angle_index - 0xA0;
                local_buf[sequence] = node;
        }
        {
               // only publish the data when it contains a full 360 degree scan
                unique_lock<mutex> lck(receivedateMutex);
                memcpy(_cached_scan_node_buf, local_buf, 60*sizeof(lds_response_measurement_node_t));
                memcpy(_cached_scan_node_buf_odom, local_buf_odom, sizeof(lds_response_measurement_node_odom_t));
          }
          flag_a0 = true;
          usleep(1000);
       }
}

void LaserOdomUart::datareceive_odom(void)
{
    lds_response_measurement_node_odom_t local_buf_odom[1];
    memset(local_buf_odom, 0, sizeof(local_buf_odom));
//    struct timeval start;
    while(1)
    {
  //
		  {
            unique_lock<mutex> lck(uartShutDownMutex);
            if(uartShutDownFlag == true) break;
          }
         lds_response_measurement_node_odom_t node_odom;


         waitNode_odom(&node_odom);
         local_buf_odom[0] = node_odom;

         {
                unique_lock<mutex> lck(receivedateMutex);
                memcpy(_cached_scan_node_buf_odom, local_buf_odom, sizeof(lds_response_measurement_node_odom_t));                
         }
       }
}

int LaserOdomUart::waitNode(lds_response_measurement_node_t *node)
{
       int  recvPos = 0;
       _u8  recvBuffer[sizeof(lds_response_measurement_node_t)];
       _u8 *nodeBuffer = (_u8*)node;
//       _u8 *nodeBuffer_odom = (_u8*)node_odom;

       bool flag_odom = false;
       int remainSize = 0;
       if(leave == true)
       {
           recvPos = kk;
       }

       while(1)
       {
		  {
            unique_lock<mutex> lck(uartShutDownMutex);
            if(uartShutDownFlag == true) break;
          }
           remainSize = sizeof(lds_response_measurement_node_t) - recvPos;
           int recvSize;
           waitfordata(remainSize, &recvSize, 0);
           if (recvSize > remainSize) recvSize = remainSize;
           recvdata(recvBuffer, recvSize, 0);

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
                   if (currentByte == 0xFA)
                   {
                       flag_odom = false;
                   }
                   else if (currentByte == 0x5A)
                   {
                      // cerr<<"I want you!"<<endl;
                       flag_odom = true;
                   }
                   else continue;
                }
                break;
                case 1:
                {
                       if(currentByte >=0xA0 && currentByte <= 0xDB && flag_odom == false)
                       {
                            if(currentByte == 0xA0)
                            {
                                flag_1 += 1;
		/*		if(flag_1 == 1)
				{
				    flag_a0 = true;
               		            clock_gettime(CLOCK_REALTIME, &start_laser);
               		            //node -> timestamp = start_laser;				
				}*/
                                if(flag_1 == 2)
                                {
                                    kk = recvSize;
                                    leave = true;
                                    //cerr<<"I do know!!!!!"<<endl;
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
                                    //cerr<<"I do know it"<<endl;
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
                       else if(currentByte >=0xA1 && currentByte <= 0xDB)
                       {
                           if(currentByte == mm)
                           {                       
                               mm += 1;
                               if(mm == 0xDC)
                               {
                                   mm = 0xA1;
                               }
                           }
                           else
                           {
                                //hh = currentByte - mm;
                                mm = currentByte + 1;
                                if(mm == 0xDC)
                                {
                                    mm = 0xA1;
                                }
                           }
                       }
                       else if(currentByte == 0xA5 && flag_odom)
                       {
                        /*       nodeBuffer_odom[0] = 0x5A;
                               nodeBuffer_odom[1] = 0xA5;
                               for(int count=2; count<15; count++)
                               {
                                    nodeBuffer_odom[count] = recvBuffer[++pos];
                               }
                               kk = recvSize - 15;
                               leave = true;

                               for(int count=0; count<kk; count++)
                               {
                                    nodeleave[count] = recvBuffer[++pos];
                               }
                          */
                               return 1;
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

int LaserOdomUart::waitNode_odom(lds_response_measurement_node_odom_t *node_odom)
{
       int  recvPos = 0;
       _u8  recvBuffer[sizeof(lds_response_measurement_node_odom_t) - 8];
       _u8 *nodeBuffer_odom = (_u8*)node_odom;

       while(1)
       {
		  {
            unique_lock<mutex> lck(uartShutDownMutex);
            if(uartShutDownFlag == true) break;
          }
           int remainSize = sizeof(lds_response_measurement_node_odom_t) - 8 - recvPos;
           int recvSize;
           waitfordata(remainSize,  &recvSize, 1);
           if (recvSize > remainSize) recvSize = remainSize;
           recvdata(recvBuffer, recvSize, 1);

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

           if (recvPos == (sizeof(lds_response_measurement_node_odom_t) - 8))
           {
               clock_gettime(CLOCK_REALTIME, &start_odom);
               node_odom -> timestamp = start_odom;
               return 0;
           }
        }
}

int LaserOdomUart::waitfordata(int data_count, int * returned_size, int lo_flag)
{
    int length = 0;
    if (returned_size==NULL) returned_size=(int *)&length;
    *returned_size = 0;

    fd_set input_set;
    FD_ZERO(&input_set);
    if(!lo_flag)
    {
        FD_SET(uartFd_laser, &input_set);
        if ( ioctl(uartFd_laser, FIONREAD, returned_size) == -1) return -1;
        if (*returned_size >= data_count)
        {
            return 0;
        }

        while(1)
        {
            assert (FD_ISSET(uartFd_laser, &input_set));
            if ( ioctl(uartFd_laser, FIONREAD, returned_size) == -1) return -1;
            if (*returned_size >= data_count)
            {
                return 0;
            }
        }
    }
    else
    {
        FD_SET(uartFd_odom, &input_set);
         tcflush(uartFd_odom,TCIFLUSH);
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
        }
    }

    /*
    int max_fd;
    fd_set input_set;
    struct timeval timeout_val;

    // Initialize the input set
    FD_ZERO(&input_set);
    FD_SET(uartFd, &input_set);
    max_fd = uartFd + 1;

    // Initialize the timeout structure
    timeout_val.tv_sec = 0;
    timeout_val.tv_usec = 30000; //30ms


    if ( ioctl(uartFd, FIONREAD, returned_size) == -1) return -1;
    if (*returned_size >= data_count)
    {
        return 0;
    }

    while (1)
    {
        // Do the select
        int n = ::select(max_fd, &input_set, NULL, NULL, &timeout_val);

        if (n < 0)
        {
            // select error
            return -1;
        }
        else if (n == 0)
        {
            // time out
            return -1;
        }
        else
        {
            // data avaliable
            assert (FD_ISSET(uartFd, &input_set));
            if ( ioctl(uartFd, FIONREAD, returned_size) == -1) return -1;
            if (*returned_size >= data_count)
            {
                return 0;
            }
        }
    }
    */
    return -1;
}

int LaserOdomUart::recvdata(unsigned char * data, int size, int lo_flag)
{
    if(!lo_flag)
    {
          read(uartFd_laser, data, size);
    }
    else
    {
        read(uartFd_odom, data, size);
    }
}

void LaserOdomUart::grabScanData(lds_response_measurement_node_t* nodebuffer, int & count, lds_response_measurement_node_odom_t* nodebuffer_odom, int & count_odom)
{
    while(!flag_a0) {}
    flag_a0 = false;
    unique_lock<mutex> lck(LaserDataMutex);
    memcpy(nodebuffer, _cached_scan_node_buf, count*sizeof(lds_response_measurement_node_t));
    memcpy(nodebuffer_odom, _cached_scan_node_buf_odom, count_odom*sizeof(lds_response_measurement_node_odom_t));
    //_cached_scan_node_count = 0;
    //_cached_scan_node_odom_count = 0;
   // return 0;
}

void LaserOdomUart::ascendScanData(lds_response_measurement_node_t* nodebuffer, int  count, lds_response_measurement_node_odom_t* nodebuffer_odom, int  count_odom)
{
    /*
    //int i = 0;
    int tmp = 0;
    //Tune head
    unique_lock<mutex> lck(LaserDataMutex);
    
      OdomData.X = nodebuffer_odom[0].X/1000.;
      OdomData.Y = nodebuffer_odom[0].Y/-1000.;
      OdomData.Theta = nodebuffer_odom[0].Theta/-1000.;
      theta[0] = theta[1];
      theta[1] =  OdomData.Theta;
      //cerr << "  theta[0] = "<<   theta[0]<<"--------------------  theta[1] = " <<   theta[1]<<endl;
      OdomData.timestamp = nodebuffer_odom[0].timestamp;
      float diff = (theta[1] - theta[0])*1.8/(3.15159*3590);
      //cerr << "diff = " << diff << endl;
     for (int i = 0; i < count; i++)
     {
		tmp = (6+diff)*i;
		if(tmp < 0 )
		{
			tmp = 0;
		}
		else if(tmp > 355)
		{
			//cerr << diff << "--------------------tmp" << tmp<<endl;
			tmp = 355;
		}
        LaserData.Distance[359-tmp] = nodebuffer[i].distance_0;
        LaserData.Distance[359-tmp-1] = nodebuffer[i].distance_1;
        LaserData.Distance[359-tmp-2] = nodebuffer[i].distance_2;
        LaserData.Distance[359-tmp-3] = nodebuffer[i].distance_3;
        LaserData.Distance[359-tmp+4] = nodebuffer[i].distance_4;
        LaserData.Distance[359-tmp-5] = nodebuffer[i].distance_5;
        if(tmp > 355)
         {
			break;
		 }
       }
      LaserData.timestamp = start_laser;
      */
    unique_lock<mutex> lck(receivedateMutex);
    OdomData.X = nodebuffer_odom[0].X/1000.;
    OdomData.Y = nodebuffer_odom[0].Y/-1000.;
    OdomData.Theta = nodebuffer_odom[0].Theta/-1000.;
    for (int i = 0; i < count; i++)
    {
        LaserData.Distance[359- 6*i] = nodebuffer[i].distance_0/1000.0;
        LaserData.Distance[359- 6*i- 1] = nodebuffer[i].distance_1/1000.0;
        LaserData.Distance[359- 6*i- 2] = nodebuffer[i].distance_2/1000.0;
        LaserData.Distance[359- 6*i- 3] = nodebuffer[i].distance_3/1000.0;
        LaserData.Distance[359- 6*i- 4] = nodebuffer[i].distance_4/1000.0;
        LaserData.Distance[359- 6*i- 5] = nodebuffer[i].distance_5/1000.0;
    }
    LaserData.timestamp = start_laser;
}

Laser_Data LaserOdomUart::getCurrentLaser(void)
{
    unique_lock<mutex> lck(LaserDataMutex);
    return LaserData;
}

Odom_Data LaserOdomUart::getCurrentOdom(void)
{
    unique_lock<mutex> lck(OdomDataMutex);
    return OdomData;
}

}
}
