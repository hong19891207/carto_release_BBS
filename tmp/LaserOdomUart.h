//by fabregas
//read wheel encoder data to give slam a pridictor

#ifndef LASER_ODOM_UART_H
#define LASER_ODOM_UART_H

#include <mutex>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

#include <condition_variable>
#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <unistd.h>     /*Unix 标准函数定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX 终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <string>
#include <iostream>

#include <vector>
#include <deque>
#include <list>
#include <fstream>
#include <stdint.h>
//#include <pthread.h>

namespace carto_release {
using namespace std;

typedef int8_t         _s8;
typedef uint8_t        _u8;

typedef int16_t        _s16;
typedef uint16_t       _u16;

typedef int32_t        _s32;
typedef uint32_t       _u32;

typedef int64_t        _s64;
typedef uint64_t       _u64;

typedef struct _lds_response_measurement_node_t {
    _u8    sync;
    _u8    angle_index;
    _u16   RPM;
    _u16   intensity_0;
    _u16   distance_0;
    _u16   reserved_0;
    _u16   intensity_1;
    _u16   distance_1;
    _u16   reserved_1;
    _u16   intensity_2;
    _u16   distance_2;
    _u16   reserved_2;
    _u16   intensity_3;
    _u16   distance_3;
    _u16   reserved_3;
    _u16   intensity_4;
    _u16   distance_4;
    _u16   reserved_4;
    _u16   intensity_5;
    _u16   distance_5;
    _u16   reserved_5;
    _u16   checksum;
    //timespec timestamp;
}__attribute__((packed)) lds_response_measurement_node_t;

typedef struct _lds_response_measurement_node_odom_t {
    _u8    sync_1;      // syncbit:1;syncbit_inverse:1;quality:6;
    _u8    sync_2;
    _u8    index;
    _s32    X;
    _s32    Y;
    _s16    Theta;
    _u8     sync_3;
    _u8     sync_4;
    timespec timestamp;
}__attribute__((packed)) lds_response_measurement_node_odom_t;

namespace LASER_ODOM
{

typedef struct UART_PROTOCOL{
    int baudRate;
    int dataBits;
    int stopBits;
    char parity;
    char streamControl; //
}UartProtocol;

//[ x, y, theta] data structure
typedef struct ODOM_DATA{
    timespec timestamp;
    int frameIndex;
    float X;
    float Y;
    float Theta;
}Odom_Data;

typedef struct LASER_DATA{
    timespec timestamp;
    float angle_min_;
    float angle_max_;
    float angle_increment_;
    float scan_time_;
    float time_increment_;
    float range_min_;
    float range_max_;

    float Distance[360];
    float Angle[360];
}Laser_Data;

class LaserOdomUart
{
public:
    LaserOdomUart();
    ~LaserOdomUart();

    //uart open port and init protocol, portNum, baudrate, databits, stopbits, parity, streamcontrol
    bool uartInit(const char*, int, int, int, const char, const char, int);

    //close uartFd
    void uartClose(void);

    void datacollect(void);
    void datareceive_laser(void);
    void datareceive_odom(void);

    int waitNode(lds_response_measurement_node_t *);
    int waitNode_odom(lds_response_measurement_node_odom_t *);
    int waitfordata(int , int * , int);
    int recvdata(unsigned char * , int , int);

    void grabScanData(lds_response_measurement_node_t * , int & , lds_response_measurement_node_odom_t * , int &  );
    void ascendScanData(lds_response_measurement_node_t * ,int count ,lds_response_measurement_node_odom_t * ,int );

    Odom_Data getCurrentOdom(void);
    Laser_Data getCurrentLaser(void);
    struct timespec start_laser;
    struct timespec start_odom;
    struct timespec start_laser_1;
    struct timespec end_odom[60];
    struct timespec end_laser_l[60];
    int _odom_count;
    float  odom_last_time ;

protected:
    UartProtocol uartProtocol; //save current uart prococol
    Laser_Data LaserData;
    Odom_Data OdomData;

    int uartFd_laser;
    int uartFd_odom;
    int _cached_scan_node_count;
    int _cached_scan_node_odom_count;
    int mm;
    _u8 nodeleave[42];
    bool leave;
    bool flag_a0;
    int kk;
    int flag_1;
    int flag_2;
    bool flag_tt;
    bool flag_odom_ok;
    float theta[2];

    
    lds_response_measurement_node_t       _cached_scan_node_buf[180];
    lds_response_measurement_node_t       data2compensate;
    lds_response_measurement_node_odom_t       _cached_scan_node_buf_odom[10];
    boost::shared_ptr<boost::thread>  uartThread_laser;
    boost::shared_ptr<boost::thread>  uartThread_odom;
    std::mutex LaserDataMutex;
    std::mutex OdomDataMutex;
    std::mutex uartShutDownMutex;
    std::mutex receivedateMutex;
    std::mutex receivedateodomMutex;
    bool uartShutDownFlag;
};

}
}

#endif
