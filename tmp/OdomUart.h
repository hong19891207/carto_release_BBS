//by fabregas
//read wheel encoder data to give slam a pridictor

#ifndef ODOM_UART_H
#define ODOM_UART_H

#include <mutex>
//#include <thread>

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

typedef struct _lds_response_measurement_node_odom_t {
    _u8    sync_1;      // syncbit:1;syncbit_inverse:1;quality:6;
    _u8    sync_2;
    _u8    index;
    _s32    X;
    _s32    Y;
    _s16    Theta;
    _u8     sync_3;
    _u8     sync_4;
//    timespec timestamp;
}__attribute__((packed)) lds_response_measurement_node_odom_t;

namespace ODOM
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

class OdomUart
{
public:
    OdomUart();
    ~OdomUart();

    //uart open port and init protocol, portNum, baudrate, databits, stopbits, parity, streamcontrol
    bool uartInit(const char*, int, int, int, const char, const char);

    //close uartFd
    void uartClose(void);

    void datacollect(void);
    void datareceive_odom(void);

    int waitNode_odom(lds_response_measurement_node_odom_t *);
    int waitfordata(int, int *);
    int recvdata(unsigned char *, int);
    void grabScanData(lds_response_measurement_node_odom_t * , int &  );
    void ascendScanData(lds_response_measurement_node_odom_t * ,int );

    Odom_Data getCurrentOdom(void);
    struct timespec start_odom;
protected:
    UartProtocol uartProtocol; //save current uart prococol
    Odom_Data OdomData;

    int uartFd_odom;
    bool flag_tt;
    

    lds_response_measurement_node_odom_t       _cached_scan_node_buf_odom[10];

    boost::shared_ptr<boost::thread>  uartThread_odom;

    std::mutex OdomDataMutex;
    std::mutex uartShutDownMutex;
    std::mutex receivedateMutex;
    bool uartShutDownFlag;
};

}
}//

#endif
