#include<iostream>
#include <ros/ros.h>

#include <mongoose/mongoose.h>
#include <stdint.h>
#include <assert.h>
#include <fstream>

#include <netinet/in.h>
#include <stdio.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string>
#include <math.h>
#include "cJSON/cJSON.h"

using namespace std;
using std::string;

#define ROW_HEIGHT 30

// mongoose 的 ws client 示例是基于事件循环然后回调

// ws 客户端状态
enum WsState {
    WS_CLOSED = 0,
    WS_CONNECTING,
    WS_CONNECTED,
};
static int m_ws_state = WS_CLOSED; // 客户端连接状态，未连接（被关闭），连接中，已连接

// UI win 显示标识
static int m_ui_vehiclestatus = 1;
static int m_ui_position = 1;
static int m_ui_osbgrid = 1;

static uint64_t clock_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static void ev_handler(struct mg_connection *nc, int ev, void *ev_data)
{
    switch (ev) {
    case MG_EV_TIMER: {
        if (m_ws_state != WS_CONNECTED) {
            // 连接超时，设置立即关闭
            ROS_ERROR("Connecting timeout");
        }
        break;
    }
    case MG_EV_CONNECT: {
        int status = *((int *) ev_data);
        if (status != 0) {
            ROS_ERROR("Connection error: %d %s", status, strerror(status));
            m_ws_state = WS_CLOSED;
        } else {
            // 可能 DNS 解析刚完成
            ROS_INFO("Connecting...");
            m_ws_state = WS_CONNECTING;
        }
        break;
    }
    case MG_EV_WEBSOCKET_HANDSHAKE_DONE: {
        struct http_message *hm = (struct http_message *) ev_data;
        if (hm->resp_code == 101) {
            ROS_INFO("Connection connected");
            m_ws_state = WS_CONNECTED;
        } else {
            ROS_ERROR("Connection failed! HTTP code %d", hm->resp_code);
            /* Connection will be closed after this. */
            m_ws_state = WS_CLOSED;
        }
        break;
    }
    case MG_EV_WEBSOCKET_FRAME: {
        struct websocket_message *wm = (struct websocket_message *) ev_data;
        ROS_DEBUG("Received frame size=%u", wm->size);
        // 解析 RobotResponse

    
            
        
        break;
    }
    case MG_EV_CLOSE: {
        if (m_ws_state == WS_CONNECTED) {
            ROS_INFO("Connection disconnected");
        } else {
            ROS_INFO("Connecting close");
        }
        m_ws_state = WS_CLOSED;
        break;
    }
    }
}

static void ws_connect()
{
}



static char *get_home_dir()
{
#ifdef _WIN32
#define ENV_HOME "HOMEPATH"
#else
#define ENV_HOME "HOME"
#endif

    static char _buf[256];
    if (strlen(_buf) == 0) {
        char *p = getenv(ENV_HOME);
        if (p) {
#ifdef _WIN32
            char *q = getenv("HOMEDRIVE");
            snprintf(_buf, sizeof(_buf), "%s%s\\", q ? q : "C:", p);
#else
            snprintf(_buf, sizeof(_buf), "%s/", p);
#endif
        }
    }
    return _buf;
}


static double round_fx(double f, int afterdot)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "%.*f", afterdot, f);
    double ret = atof(buf);
    // check zero
    if (fabs(ret) <= pow(10, -afterdot)) {
        return 0;
    } else {
        return ret;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "iau_ros_hmi_test");
    ros::NodeHandle nh;
    std::ifstream OpenFile;
    OpenFile.open("./djs/vessel_gps.json");
    string str_file;
    std::string line;
    while (std::getline(OpenFile, line))
    {
        str_file+=line;
    }
        std::cout<<"!!! "<<str_file<<std::endl;
        OpenFile.close();
    sleep(0.1);
    std::ifstream OpenFile1;
    OpenFile1.open("./djs/rov_pose.json");
    string str_file1;
    std::string line1;
    while (std::getline(OpenFile1, line1))
    {
        str_file1+=line1;
    }
        OpenFile1.close();
        std::cout<<"!!! "<<str_file1<<std::endl;
    sleep(0.1);

    std::ifstream OpenFile2;
    OpenFile2.open("./djs/USBL.json");
    string str_file2;
    std::string line2;
    while (std::getline(OpenFile2, line2))
    {
        str_file2+=line2;
    }
        OpenFile2.close();
        std::cout<<"!!! "<<str_file2<<std::endl;
    sleep(0.1);

    std::ifstream OpenFile3;
    OpenFile3.open("./djs/poindclouds.json");
    string str_file3;
    std::string line3;
    while (std::getline(OpenFile3, line3))
    {
        str_file3+=line3;
    }
        OpenFile3.close();
        std::cout<<"!!! "<<str_file3<<std::endl;
    sleep(0.1);


    ROS_INFO("get");
    //vessel_pose = cJSON_Parse(str_file.c_str());
    static struct mg_mgr _mgr;
    static struct mg_connection *_connect = NULL;
    static bool _mg_init = false;

    // 初始化 _mgr
    if (!_mg_init) {
        _mg_init = true;
        mg_mgr_init(&_mgr, NULL);
    }

    static char svr_addr[32] = "ws://127.0.0.1:4300";

	if (m_ws_state == WS_CLOSED) {
        // 创建连接
        _connect = mg_connect_ws(&_mgr, ev_handler, svr_addr, "ws_chat", NULL);
        if (_connect == NULL) {
            ROS_ERROR("Invalid URL '%s'", svr_addr);
        } else {
            ROS_INFO("Create connection OK");
            // 设置超时
            mg_set_timer(_connect, mg_time() + 3);
            m_ws_state = WS_CONNECTING;
        }
    } else if (m_ws_state == WS_CONNECTING) {
        // 不做任何操作
        ROS_INFO("Still connectiing...");
    } else if (m_ws_state == WS_CONNECTED) {
    	// 断开，发送 CLOSE
    	mg_send_websocket_frame(_connect, WEBSOCKET_OP_CLOSE, NULL, 0);
    }

    static uint64_t _last = 0;
    bool is_add = true;
//1
                unsigned char _packbuf[1024 * 100];
                unsigned char *str_data = (unsigned char*)(str_file.c_str());

                // str_data = (unsigned char *)cJSON_PrintUnformatted(vessel_pose);
                size_t sz = sizeof(str_data);
                assert (sz <= sizeof(_packbuf) && "RobotResponse size too big(>100kb");
        ROS_INFO("copy no");

                memcpy(&_packbuf,(const char*)str_data,strlen((const char*)str_data));
                std::cout<<_packbuf<<std::endl;
        ROS_INFO("copy yes");
//2
                unsigned char _packbuf1[1024 * 100];
                unsigned char *str_data1 = (unsigned char*)(str_file1.c_str());

                // str_data = (unsigned char *)cJSON_PrintUnformatted(vessel_pose);
                size_t sz1 = sizeof(str_data1);
                assert (sz1 <= sizeof(_packbuf1) && "RobotResponse size too big(>100kb");
        ROS_INFO("copy no");

                memcpy(&_packbuf1,(const char*)str_data1,strlen((const char*)str_data1));


//3
                unsigned char _packbuf2[1024 * 100];
                unsigned char *str_data2 = (unsigned char*)(str_file2.c_str());

                // str_data = (unsigned char *)cJSON_PrintUnformatted(vessel_pose);
                size_t sz2 = sizeof(str_data2);
                assert (sz2 <= sizeof(_packbuf2) && "RobotResponse size too big(>100kb");
        ROS_INFO("copy no");
//4
                unsigned char _packbuf3[1024 * 100];
                unsigned char *str_data3 = (unsigned char*)(str_file3.c_str());

                // str_data = (unsigned char *)cJSON_PrintUnformatted(vessel_pose);
                size_t sz3 = sizeof(str_data3);
                assert (sz3 <= sizeof(_packbuf3) && "RobotResponse size too big(>100kb");
        ROS_INFO("copy no");
                memcpy(&_packbuf3,(const char*)str_data3,strlen((const char*)str_data3));
    int num = 0;
    while(ros::ok()){
         // poll and request
         mg_mgr_poll(&_mgr, 5);
        // 连接情况下周期发送 RobotRequest
        if (m_ws_state == WS_CONNECTED) {
            uint64_t now = clock_ms();
            // 10 hz 频率
            if(now - _last > 1000)
            {
                _last = now;
                  ////////
                //str_data->SerializePartialToArray(_packbuf,sizeof(_packbuf));
                if(num == 0)
                {
                    mg_send_websocket_frame(_connect, WEBSOCKET_OP_BINARY, _packbuf, strlen((const char*)str_data));

                    num++;
                }
                if(num == 1)
                {
                    mg_send_websocket_frame(_connect, WEBSOCKET_OP_BINARY, _packbuf1, strlen((const char*)str_data1));
                    num++;
                }
                if(num == 2)
                {
                    mg_send_websocket_frame(_connect, WEBSOCKET_OP_BINARY, _packbuf2, strlen((const char*)str_data2));
                    num++;
                }
                if(num == 3)
                {
                    mg_send_websocket_frame(_connect, WEBSOCKET_OP_BINARY, _packbuf3, strlen((const char*)str_data3));
                    num=0;
                }
            }
    
        }
    }
	return 0;
}

