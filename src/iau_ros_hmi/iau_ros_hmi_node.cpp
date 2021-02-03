#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <iau_ros_hmi/search_point.h>//搜寻信息自定义消息类型
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/range_image/range_image.h>
// #include <pcl/filters/filter.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/common/common.h>
// #include <pcl/registration/icp.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "cJSON/cJSON.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <cstdlib>
#include <string>
#include <mutex>
#include <math.h>
// #include <msgq.h>
#include <mongoose/mongoose.h>
//#include "proj.h"
#include "CoorConv.hpp"

#include<iostream>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "jpg_kit.h"

//PJ *PrOj = NULL;

using namespace std;
using std::string;


#define D2R (M_PI/180.0)
#define PI 3.14159265
#define dist(x0, y0, x1, y1) sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0))
typedef pcl::PointXYZI  PointType;


cJSON *cjson1=NULL;
cJSON *rov_pose = NULL;
cJSON *usbl = NULL;
cJSON *vessel_gps = NULL;
cJSON *sonarcloud = NULL;
cJSON *sonar_image = NULL;


static sig_atomic_t m_signal_received = 0;
static const char * m_http_port = "4300";
static struct mg_serve_http_opts m_http_server_opts;
static double m_base_line = 51.5*1E-2;
double current_time, last_time;
std::mutex rov_mutex;
std::mutex g_position_mutex;
std::mutex g_trail_mutex;

ros::Subscriber search_point_sub;// 搜寻信息
ros::Subscriber ClickPoint_sub;
ros::Subscriber subSimOdometry;
ros::Subscriber Pub_path_signal;
std::vector<geometry_msgs::PoseStamped> global_path;
std_msgs::Bool path_signal;//是否显示路径
geometry_msgs::PoseStamped targetPoint;//点击的目标点
geometry_msgs::PointStamped current_mission_Point;//当前任务点
geometry_msgs::PointStamped GotoPoint;//前往的目标点

int nearInd;//当前位置轨迹点序号
int lastInd;//上次位置轨迹点序号

int current_mission_point = 0; // 当前任务点序号
int next_mission_point = 0;// 下一任务点序号


bool clicked = false;
bool arrived = false;
bool arrived2 = false;
bool first_odom = true;
bool msg_arrived = false;
nav_msgs::Path path;
std::vector<geometry_msgs::Point> global_point;

image_transport::Publisher sonarImage_pub;

ros::Publisher pubOdometry;
ros::Publisher pubOdometry2;
ros::Publisher pubSonarPoint;
ros::Publisher pubpath;
ros::Publisher pubpath3;
ros::Publisher spline_pub;
ros::Publisher pubNextPoint;
ros::Publisher markerArrayPub;
ros::Publisher target_msg_pub;

double Init_lon = 101.00000000;//经度
double Init_lat =  20.00000000;
int UTM_zone = 50;
// PJ_COORD Inita;
// PJ_COORD Initb;
    UTMCoord InitPose;

tf::StampedTransform RosPoseTrans;
tf::StampedTransform VesselPoseTrans;
nav_msgs::Odometry RosPose;
nav_msgs::Odometry VesselPose;
pcl::PointCloud<PointType>::Ptr Cloud;
struct ROVPose{

    string headname;
    double timestamp;
    string X_Velocity_Bottom;
    string Y_Velocity_Bottom;
    string Z_Velocity_Bottom;
    string rov_Roll;
    string rov_Pitch;
    string rov_Heading;
    string rov_Depth;
    string rov_Altitude;


};
struct USBL{

    string headname;
    double timestamp;
    string TPcode;
    string longitude;
    string N_S;
    string latitude;
    string E_W;
    string roll;
    string pitch;
    string heading;
    string x_coordinate;
    string y_coordinate;
    string depth;
};
struct VesselGPS{
    string headname;
    string timestamp;
    string longitude;
    string N_S;
    string latitude;
    string E_W;
    string heading;
};

static void signal_handler(int sig_num)
{
    signal(sig_num, signal_handler);  // Reinstantiate signal handler
    m_signal_received = sig_num;
    //printf("sig_num = %d\n",m_signal_received);
}

// 缓存的系统各个 message
static uint64_t  m_position_clk;

static uint64_t clock_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static int is_websocket(const struct mg_connection *nc)
{
    return nc->flags & MG_F_IS_WEBSOCKET;
}


//发布声呐图像
void PublishSonarImage(cJSON *sonar_image)
{
    string str =  cJSON_GetObjectItem(sonar_image,"image")->valuestring;

    cv::Mat dst;
    vector <unsigned char> img_data;

    int len = str.size() + 1;
    char* buf = new char[len*2+1];
    int lengh = mg_base64_decode((const unsigned char*)str.data(), str.size(), buf);
    std::string out_str(buf, lengh);

    delete[] buf;

    // Base64Decode(str, &out_str);
    img_data.assign(out_str.begin(), out_str.end());
    dst = cv::imdecode(img_data, CV_LOAD_IMAGE_COLOR);
    cv::imwrite("/home/ubuntu/114.jpg", dst);

   
   
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
    sonarImage_pub.publish(msg);
    ROS_INFO("SonarImage pub");
}

int get_near_traj(std::vector<geometry_msgs::PoseStamped>& _local_plan, double cx, double cy)
{
    
    double closest_distance = std::numeric_limits<double>::max();
    int closest_waypoint_index = -1;
    geometry_msgs::PoseStamped add_point;
    add_point.pose.position.x = cx;
    add_point.pose.position.y = cy;
    //遵循轨迹顺序行走,重线不碍事.
    for(int i = 0; i < _local_plan.size(); i++)
    { 
        double traj_x = _local_plan[i].pose.position.x;
        double traj_y = _local_plan[i].pose.position.y;  
        double current_distance = dist(cx,cy,traj_x,traj_y);  
        if(current_distance < closest_distance)
        {
        closest_distance = current_distance;
        closest_waypoint_index = i;
        } 
    }
    
    return closest_waypoint_index;
}

//发布声呐点云
void PublishPointCloud(cJSON *pcloud)
{

    Cloud->clear();
    PointType point;
    if(NULL != pcloud)
    {
        cJSON *sonar = pcloud->child;
    
        while( sonar != NULL ){

            point.x = cJSON_GetObjectItem( sonar , "x_coordinate")->valuedouble;
            point.y = cJSON_GetObjectItem( sonar , "y_coordinate")->valuedouble;
            point.z = cJSON_GetObjectItem( sonar , "z_coordinate")->valuedouble;
            point.intensity = cJSON_GetObjectItem( sonar , "intensities")->valuedouble;
            
            Cloud->push_back(point);
            sonar = sonar->next;
        }
    }
    sensor_msgs::PointCloud2 SensorCloud;
    pcl::toROSMsg(*Cloud, SensorCloud);
    SensorCloud.header.stamp = ros::Time::now();
    SensorCloud.header.frame_id = "/rexrov/base_stabilized";
    pubSonarPoint.publish(SensorCloud);
    ROS_INFO("cloud pub");
}

//发布水下机器人位置
void PublishRobotPose(cJSON *rov_pose, cJSON *usbl)
{   
    //std::lock_guard(rov_mutex);
    ROVPose rovpose;
    rovpose.headname = cJSON_GetObjectItem(rov_pose,"headname")->valuestring;
    rovpose.timestamp = cJSON_GetObjectItem(rov_pose,"timestamp")->valuedouble;
    rovpose.X_Velocity_Bottom = cJSON_GetObjectItem(rov_pose,"X_Velocity_Bottom")->valuestring;
    rovpose.Y_Velocity_Bottom = cJSON_GetObjectItem(rov_pose,"Y_Velocity_Bottom")->valuestring;
    rovpose.Z_Velocity_Bottom = cJSON_GetObjectItem(rov_pose,"Z_Velocity_Bottom")->valuestring;
    rovpose.rov_Roll = cJSON_GetObjectItem(rov_pose,"rov_Roll")->valuestring;
    rovpose.rov_Pitch = cJSON_GetObjectItem(rov_pose,"rov_Pitch")->valuestring;
    rovpose.rov_Heading = cJSON_GetObjectItem(rov_pose,"rov_Heading")->valuestring;
    rovpose.rov_Depth = cJSON_GetObjectItem(rov_pose,"rov_Depth")->valuestring;
    rovpose.rov_Altitude = cJSON_GetObjectItem(rov_pose,"rov_Altitude")->valuestring;
    USBL Usbl;
    Usbl.headname = cJSON_GetObjectItem(usbl,"headname")->valuestring;
    Usbl.timestamp = cJSON_GetObjectItem(usbl,"timestamp")->valuedouble;
    Usbl.TPcode = cJSON_GetObjectItem(usbl,"TPcode")->valuestring;

    Usbl.longitude = cJSON_GetObjectItem(usbl,"longitude")->valuestring;
    Usbl.N_S = cJSON_GetObjectItem(usbl,"N_S")->valuestring;
    Usbl.latitude = cJSON_GetObjectItem(usbl,"latitude")->valuestring;
    Usbl.E_W = cJSON_GetObjectItem(usbl,"E_W")->valuestring;
    Usbl.roll = cJSON_GetObjectItem(usbl,"roll")->valuestring;
    Usbl.pitch = cJSON_GetObjectItem(usbl,"pitch")->valuestring;
    Usbl.heading = cJSON_GetObjectItem(usbl,"heading")->valuestring;
    Usbl.x_coordinate = cJSON_GetObjectItem(usbl,"x_coordinate")->valuestring;
    Usbl.y_coordinate = cJSON_GetObjectItem(usbl,"y_coordinate")->valuestring;
    Usbl.depth = cJSON_GetObjectItem(usbl,"depth")->valuestring;

    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(atof(rovpose.rov_Roll.c_str()), atof(rovpose.rov_Pitch.c_str()), atof(rovpose.rov_Heading.c_str()));

    // PJ_COORD a = proj_coord(atof(Usbl.longitude.c_str()),atof(Usbl.latitude.c_str()),0,0);
    // PJ_COORD b = proj_trans(PrOj, PJ_FWD, a);
    // ROS_INFO("lon:%.8lf lat:%.8lf", a.lp.lam, a.lp.phi);
    // // 输出高斯正算值
    // ROS_INFO("x: %.3lf, y: %.3lf", b.enu.e, b.enu.n);
    UTMCoord UsbPose;
    LatLonToUTM(atof(Usbl.latitude.c_str())/180*PI,atof(Usbl.longitude.c_str()),UTM_zone,UsbPose);
    RosPose.header.stamp = ros::Time::now();
    RosPose.pose.pose.position.x = UsbPose.x - InitPose.x;
    RosPose.pose.pose.position.y = UsbPose.y - InitPose.y;
    RosPose.pose.pose.position.z = atof(Usbl.depth.c_str()) / 10.0; //  竖直方向深度缩小10倍
    RosPose.pose.pose.orientation.x = geoQuat.x;
    RosPose.pose.pose.orientation.y = geoQuat.y;
    RosPose.pose.pose.orientation.z = geoQuat.z;
    RosPose.pose.pose.orientation.w = geoQuat.w;

    pubOdometry.publish(RosPose);
    
    tf::TransformBroadcaster tfBroadcaster;

    RosPoseTrans.stamp_ = RosPose.header.stamp;
    RosPoseTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    RosPoseTrans.setOrigin(tf::Vector3(RosPose.pose.pose.position.x, RosPose.pose.pose.position.y, RosPose.pose.pose.position.z));
    tfBroadcaster.sendTransform(RosPoseTrans);


    rov_pose=NULL;
    usbl=NULL;
        // vessel_gps=NULL;


}
//仿真位置
void SimPose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{

    geometry_msgs::PoseStamped robPoint;//机器人位置
    robPoint.pose.position.x = msg->pose.pose.position.x;
    robPoint.pose.position.y = msg->pose.pose.position.y;
    robPoint.pose.position.z = msg->pose.pose.position.z;
    // 机器人姿态
    robPoint.pose.orientation.x = msg->pose.pose.orientation.x;
    robPoint.pose.orientation.y = msg->pose.pose.orientation.y;
    robPoint.pose.orientation.z = msg->pose.pose.orientation.z;
    robPoint.pose.orientation.w = msg->pose.pose.orientation.w;

    double roll,pitch,yaw;
    tf::Quaternion RQ2;  
    tf::quaternionMsgToTF(robPoint.pose.orientation,RQ2);  
    tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);  // roll 是机器人艏向角(弧度)

    double  heading = -yaw + 90.0 / 180.0 * 3.141592653;


    nav_msgs::Path path3;
    // path2.header.stamp = ros::Time::now();
    // path2.header.frame_id = "world";
    // path2.poses.clear();
    path3.header.stamp = ros::Time::now();
    path3.header.frame_id = "world";

    if(first_odom)
    {
        nearInd = next_mission_point;
        // nearInd = get_near_traj(global_path,robPoint.pose.position.x,robPoint.pose.position.y);
        first_odom = false;
        lastInd = 0;
    }

    double dd = dist(robPoint.pose.position.x,robPoint.pose.position.y,global_path[next_mission_point].pose.position.x,global_path[next_mission_point].pose.position.y);
    double angle = acos(((global_path[next_mission_point].pose.position.x - robPoint.pose.position.x)*sin(heading) + (global_path[next_mission_point].pose.position.y - robPoint.pose.position.y)*cos(heading))/dd)/ PI * 180.0 ;  // 向量点积 计算夹角
    double zuoyou = (global_path[next_mission_point].pose.position.x - robPoint.pose.position.x)*cos(heading) - sin(heading)*(global_path[next_mission_point].pose.position.y - robPoint.pose.position.y); // 向量叉积 计算目标点在左边还是右边
    if (zuoyou > 0 ){
        angle = -angle;
    }

    // double dd = dist(robPoint.pose.position.x,robPoint.pose.position.y,global_path[nearInd+1].pose.position.x,global_path[nearInd+1].pose.position.y);
    // GotoPoint.point.z = dd;
    if(dd<=2.0)//到达指定路径位置,开始扫描
    {
        arrived=true;
    }
    if(arrived)
    {
        std::cout<<"arrived"<<std::endl;
        current_mission_point = next_mission_point;
        next_mission_point++;
        if (current_mission_point > 0 ){
            path.poses.erase(path.poses.begin());
            // global_point.erase(global_point.begin());
        }
        
        arrived = false;
        arrived2 = false;
        clicked = false;

    }

   
   if (current_mission_point > global_path.size()){
            GotoPoint.point.x = GotoPoint.point.y = GotoPoint.point.z = 9527.00;
            pubNextPoint.publish(GotoPoint);

            exit(0);
   }


    if(clicked)
    
    {
        path3.poses.clear();

        dd = dist(robPoint.pose.position.x,robPoint.pose.position.y,targetPoint.pose.position.x,targetPoint.pose.position.y);
        angle = acos(((targetPoint.pose.position.x - robPoint.pose.position.x)*sin(heading) + (targetPoint.pose.position.y - robPoint.pose.position.y)*cos(heading))/dd)/ PI * 180.0 ;
        zuoyou = (targetPoint.pose.position.x - robPoint.pose.position.x)*cos(heading) - sin(heading)*(targetPoint.pose.position.y - robPoint.pose.position.y);// 向量叉积 计算目标点在左边还是右边
        if (zuoyou > 0 ){
            angle = -angle;
    }
        if(dd < 2)//到达目标点,执行任务
        {
            arrived2 = true;
        }
    
        if(arrived2)
        {

            path3.poses.push_back(robPoint);
            path3.poses.push_back(global_path[next_mission_point]);
            pubpath3.publish(path3);
            clicked = false;
            arrived2=false;
        }
        else
        {
            path3.poses.push_back(robPoint);
            path3.poses.push_back(targetPoint);
            path3.poses.push_back(global_path[next_mission_point]);
            pubpath3.publish(path3);

            //发布目标点文字
            visualization_msgs::Marker marker;
            marker.header.frame_id="world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "target_text";
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.id =0;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.lifetime = ros::Duration(1.0);

            marker.scale.z = 3;
            marker.color.b = 0;
            marker.color.g = 0;
            marker.color.r = 255;
            marker.color.a = 1;

            geometry_msgs::Pose pose;
            pose.position.x =  targetPoint.pose.position.x;
            pose.position.y =  targetPoint.pose.position.y;
            pose.position.z =  targetPoint.pose.position.z+3;
            ostringstream str;
            str<<"Target";
            marker.text=str.str();
            marker.pose=pose;
            target_msg_pub.publish(marker);
        }
        // GotoPoint.point.x =dd; // range
        // GotoPoint.point.y=(yaw - atan((targetPoint.pose.position.x - robPoint.pose.position.x) /( targetPoint.pose.position.y - robPoint.pose.position.y)) )/ PI * 180; // angle

        // GotoPoint.point.x=targetPoint.pose.position.x;
        // GotoPoint.point.y=targetPoint.pose.position.y;
    }
    else
    {
        if(msg_arrived==true&&path_signal.data == 1){//接收到了搜寻信息并且按过发布路径按钮后执行

            path3.poses.push_back(robPoint);
            // GotoPoint.point.x = global_path[next_mission_point].pose.position.x;
            // GotoPoint.point.y = global_path[next_mission_point].pose.position.y;
            path3.poses.push_back(global_path[next_mission_point]);
            std::cout<<"next_mission_point="<<next_mission_point<<std::endl;
            pubpath3.publish(path3);
            }
            // GotoPoint.point.x =dd; // range
            // GotoPoint.point.y=(yaw - atan(( global_path[nearInd+1].pose.position.x - robPoint.pose.position.x) /(global_path[nearInd+1].pose.position.y - robPoint.pose.position.y)) )/ PI * 180; // angle

    }
    GotoPoint.point.x = float(dd);
    GotoPoint.point.y = float(angle);
    GotoPoint.point.z = float(yaw);

    pubNextPoint.publish(GotoPoint);
}
//发布船的位置
void Publish_Vessel_GPS(cJSON *vessel_gps)
{   

    //std::lock_guard(rov_mutex);
    VesselGPS vessel;
    vessel.headname = cJSON_GetObjectItem(vessel_gps,"headname")->valuestring;
    vessel.timestamp = cJSON_GetObjectItem(vessel_gps,"timestamp")->valuedouble;
    vessel.longitude = cJSON_GetObjectItem(vessel_gps,"longitude")->valuestring;
    vessel.N_S = cJSON_GetObjectItem(vessel_gps,"N_S")->valuestring;
    vessel.latitude = cJSON_GetObjectItem(vessel_gps,"latitude")->valuestring;
    vessel.E_W = cJSON_GetObjectItem(vessel_gps,"E_W")->valuestring;
    vessel.heading = cJSON_GetObjectItem(vessel_gps,"heading")->valuestring;

    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromYaw(atof(vessel.heading.c_str())/180*PI);

    // PJ_COORD a = proj_coord(atof(vessel.longitude.c_str()),atof(vessel.latitude.c_str()),0,0);
    // PJ_COORD b = proj_trans(PrOj, PJ_FWD, a);
    // ROS_INFO("lon:%.8lf lat:%.8lf", a.lp.lam, a.lp.phi);
    // // 输出高斯正算值
    // ROS_INFO("x: %.3lf, y: %.3lf", b.enu.e, b.enu.n);
    UTMCoord VesPose;
    LatLonToUTM(atof(vessel.latitude.c_str())/180*PI,atof(vessel.longitude.c_str())/180*PI,UTM_zone,VesPose);
    VesselPose.header.stamp = ros::Time::now();
    VesselPose.pose.pose.position.x = VesPose.x - InitPose.x;
    VesselPose.pose.pose.position.y = VesPose.y - InitPose.y;
    VesselPose.pose.pose.position.z = 0;
    VesselPose.pose.pose.orientation.x = geoQuat.x;
    VesselPose.pose.pose.orientation.y = geoQuat.y;
    VesselPose.pose.pose.orientation.z = geoQuat.z;
    VesselPose.pose.pose.orientation.w = geoQuat.w;
    pubOdometry2.publish(VesselPose);
    ROS_INFO("x: %.3lf, y: %.3lf", VesselPose.pose.pose.position.x,VesselPose.pose.pose.position.y);
    
    tf::TransformBroadcaster VesseltfBroadcaster;

    VesselPoseTrans.stamp_ = VesselPose.header.stamp;
    VesselPoseTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    VesselPoseTrans.setOrigin(tf::Vector3(VesselPose.pose.pose.position.x, VesselPose.pose.pose.position.y, VesselPose.pose.pose.position.z));
    VesseltfBroadcaster.sendTransform(VesselPoseTrans);

    vessel_gps=NULL;
}
//rviz publish point 按钮点回调
void ClickPoint_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    targetPoint.pose.position.x = msg->point.x;
    targetPoint.pose.position.y = msg->point.y;
    targetPoint.pose.position.z = msg->point.z;

    // ROS_INFO("click point: %f, %f",ClickPoint_x ,ClickPoint_y);
    clicked = true;
}

void Pub_path_callback(const std_msgs::Bool::ConstPtr& msg){
    path_signal.data = msg->data;
}


void publish_markers(ros::Publisher *thisPub, std::vector<geometry_msgs::Point> &points, ros::Time thisStamp, std::string thisFrame) {


    /////////////////////////////////////////////
    visualization_msgs::MarkerArray markerArray;
    /////////////////////////////////////////////




    visualization_msgs::MarkerArray marker_array;

    marker_array.markers.resize(1);

    visualization_msgs::Marker& t_marker = marker_array.markers[0];
    t_marker.header.frame_id = thisFrame;
    t_marker.header.stamp = thisStamp;
    t_marker.ns = "path";
    t_marker.id = 1;
    t_marker.points.reserve(points.size());
    t_marker.colors.reserve(points.size());

    t_marker.pose.position.z = 0.0f;
    t_marker.pose.orientation.w = 1.0f;
    t_marker.lifetime = ros::Duration(1.0);
    t_marker.pose.position.z = 0;
    t_marker.pose.orientation.w = 1.0;
    t_marker.scale.x = t_marker.scale.y = t_marker.scale.z = 2.0;
    t_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    t_marker.action = visualization_msgs::Marker::ADD;

    for(int j = 0; j< points.size(); ++j) {
        geometry_msgs::Point point;
        point.x = points[j].x;
        point.y = points[j].y;
        point.z = points[j].z;
        t_marker.points.push_back(point);
        std_msgs::ColorRGBA rgba;
        rgba.r = 1.0f;
        rgba.g = 0.0f;
        rgba.b = 0.0f;
        rgba.a = 0.6f;
        t_marker.colors.push_back(rgba);

        //发布搜寻任务点文字
        /////////////////////////////////////////////
        visualization_msgs::Marker marker;
        marker.header.frame_id=thisFrame;
        marker.header.stamp = thisStamp;
        marker.ns = "basic_shapes";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id =j;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.lifetime = ros::Duration(1.0);

        marker.scale.z = 3;
        marker.color.b = 0;
        marker.color.g = 0;
        marker.color.r = 255;
        marker.color.a = 1;

        geometry_msgs::Pose pose;
        pose.position.x =  points[j].x;
        pose.position.y =  points[j].y;
        pose.position.z =  points[j].z+3;
        ostringstream str;
        str<<"Point:"<<j+1;
        marker.text=str.str();
        marker.pose=pose;
        markerArray.markers.push_back(marker);
        /////////////////////////////////////////////

    }        

    if(thisPub->getNumSubscribers() != 0)
        thisPub->publish(marker_array);
        markerArrayPub.publish(markerArray);
}

double get_new_lat(double old_lat, double coef){
    return (old_lat + coef);
}

// / cos(start_lat * (PI / 180))
double get_new_long(double old_long, double coef,double start_lat){
    return (old_long + coef);
}

void Generate_target_point(double center_point_lon, double center_point_lat, double half_length, std::vector<std::vector<double> > &point, int radius) {

    double start_lat = center_point_lat - half_length;
    double start_long = center_point_lon - half_length;
    double end_lat = center_point_lat + half_length;
    double end_long = center_point_lon + half_length;

    double coef = 2 * radius;

    std::vector<double> first_row_lats;
    std::vector<double> second_row_lats;
    double current_lat1 = start_lat;
    double current_lat2 = start_lat + radius;
    while (current_lat1 < end_lat){
        first_row_lats.push_back(current_lat1);
        second_row_lats.push_back(current_lat2);
        current_lat1 = get_new_lat(current_lat1, coef);
        current_lat2 = get_new_lat(current_lat2, coef);
    }

    cout<<"first_row_lats"<<endl;
     for(double i : first_row_lats){
         cout.precision(10);
         cout<<i<<endl;
     }
     cout<<"second_row_lats"<<endl;
     for(double i : second_row_lats){
         cout.precision(10);
         cout<<i<<endl;
     }
//     / cos(start_lat * 0.018)
    std::vector<double> first_row_longs;
    std::vector<double> second_row_longs;
    double current_long1 = start_long;
    double current_long2 = start_long + radius  ;
    while (current_long1 < end_long){
        first_row_longs.push_back(current_long1);
        second_row_longs.push_back(current_long2);
        current_long1 = get_new_long(current_long1, coef, start_lat);
        current_long2 = get_new_long(current_long2, coef, start_lat);
    }

    cout<<"first_row_longs"<<endl;
     for(double i : first_row_longs){
         cout.precision(10);
         cout<<i<<endl;
     }
     cout<<"second_row_longs"<<endl;
     for(double i : second_row_longs){
         cout.precision(10);
         cout<<i<<endl;
     }

    for (int i=0; i<first_row_longs.size(); i++){

        for (auto frla : first_row_lats){
            //std::cout<<frlo<<"\t"<<frla<<std::endl;
            point.push_back({first_row_longs[i],frla});
        }

        for (auto it = second_row_lats.rbegin(); it != second_row_lats.rend(); it++){
            //std::cout<<srlo<<"\t"<<(*it)<<std::endl;
            point.push_back({second_row_longs[i],(*it)});
        }
    }
}

void  pubtopic()
{
     if(cjson1 == NULL)
	{
		//ROS_WARN("json pack into cjson error...");
	}
	else
	{
	//获取字段值
	//cJSON_GetObjectltem返回的是一个cJSON结构体所以我们可以通过函数返回结构体的方式选择返回类型！
        string Request = cJSON_GetObjectItem(cjson1,"request")->valuestring;
        // rov_pose=NULL;
        // usbl=NULL;
        // vessel_gps=NULL;
        if(Request == "rov_pose")
        {
            ROS_INFO("rov_pose get");
            rov_pose = cJSON_GetObjectItem(cjson1,"rov_pose"); 
    
        }else if(Request == "usbl")
        {
            ROS_INFO("usbl get");
            usbl = cJSON_GetObjectItem(cjson1,"usbl");

        }else if(Request == "vessel_gps")
        {
            ROS_INFO("vessel_gps get");
            vessel_gps = cJSON_GetObjectItem(cjson1,"vessel_gps");
            if(vessel_gps != NULL)
            {
                std::lock_guard<std::mutex> lock(g_position_mutex);
                Publish_Vessel_GPS(vessel_gps);
            }
        }
        else if(Request == "sonar_image")
        {
             ROS_INFO("sonar_image get");   
             sonar_image = cJSON_GetObjectItem(cjson1,"sonar_image");
             PublishSonarImage(sonar_image);

        }
        else if(Request == "pointclouds")
        {
            ROS_INFO("pointclouds get");
            sonarcloud = cJSON_GetObjectItem(cjson1,"pointclouds");
            PublishPointCloud(sonarcloud);


        }else
        {
            ROS_WARN(" illegal request! ");
        }
        if(rov_pose!=NULL && usbl != NULL)
        {
            std::lock_guard<std::mutex> lock(rov_mutex);
            PublishRobotPose(rov_pose, usbl);
        }

    }    
}
//websocket
static void ev_handler(struct mg_connection *nc, int ev, void *ev_data)
{
    char addr[1024*100];
    mg_sock_addr_to_str(&nc->sa, addr, sizeof(addr),
                        MG_SOCK_STRINGIFY_IP | MG_SOCK_STRINGIFY_PORT);
    switch (ev) {
    case MG_EV_WEBSOCKET_HANDSHAKE_DONE: {
        ROS_INFO("accept client %s connected", addr);
        break;
    }
    case MG_EV_WEBSOCKET_FRAME: {
        struct websocket_message *wm = (struct websocket_message *) ev_data;
        ROS_INFO("receive client %s frame data len=%u", addr, wm->size);
        string str_data((char *)wm->data, wm->size);//str_data.c_str()
        //std::cout<<"#"<<str_data<<"#"<<std::endl;

        cjson1 = cJSON_Parse(str_data.c_str());
        pubtopic();
        break;
    }
    case MG_EV_HTTP_REQUEST: {
        mg_serve_http(nc, (struct http_message *) ev_data, m_http_server_opts);
        break;
    }
    case MG_EV_CLOSE: {
        if (is_websocket(nc)) {
            ROS_INFO("client %s disconnected", addr);
            // 平板断开链接时，发送急停
        }
        break;
    }
    }
   
}

//搜寻信息回调
void SearchPoint_callback(iau_ros_hmi::search_point msg)
{
    
    Init_lon = msg.mission_point_lon;
    Init_lat = msg.mission_point_lat;
    UTM_zone = msg.UTM_zone;

     LatLonToUTM(Init_lat/180*PI,Init_lon/180*PI,UTM_zone,InitPose);


    global_path.clear();
    global_point.clear();
    path.poses.clear();


    std::vector<geometry_msgs::PoseStamped>().swap(global_path);
    std::vector<geometry_msgs::Point>().swap(global_point);


//    global_path.reserve(msg.k* msg.k);
//    global_point.reserve(msg.k* msg.k);


    vector<vector<double> > pathpoints;
    //pathpoints.reserve(msg.k* msg.k);

ROS_INFO("1");
    Generate_target_point(msg.lon, msg.lat, msg.search_size, pathpoints, msg.r);
ROS_INFO("2");
    for (auto i : pathpoints){
        for (auto j : i){
            cout<<j;
        }
        cout<<endl;
    }

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    geometry_msgs::PoseStamped point_;
    geometry_msgs::Point _point;

    for(int i = 0; i< pathpoints.size(); i++)
    {
     _point.x = point_.pose.position.x = pathpoints[i][0];
     _point.y = point_.pose.position.y = pathpoints[i][1];
     _point.z = point_.pose.position.z = -20; // 初始化 path 高度

     global_path.push_back(point_);
     global_point.push_back(_point);
     path.poses.push_back(point_);
    }
    first_odom = true;
    msg_arrived = true;
}














int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "iau_ros_hmi");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    Cloud.reset(new pcl::PointCloud<PointType>());

    
    //读取文件中的轨迹
    // std::ifstream OpenFile;
    // OpenFile.open("./djs/point.txt");
    // geometry_msgs::PoseStamped point_;
    // geometry_msgs::Point _point;
    // std::string line;
    // const char * split = ",";
    // while (std::getline(OpenFile, line))
    // {
    //     char *p = strtok((char *)line.c_str(), split);
    //     double b_lat = atof(p);
    //     p  =strtok(NULL, split);
    //     double b_lon = atof(p);
    //     ROS_INFO("pathpoint: %f,%f",point_.pose.position.x,point_.pose.position.y);
    //     // PJ_COORD a = proj_coord(point_.pose.position.x,point_.pose.position.y,0,0);
    //     // PJ_COORD b = proj_trans(PrOj, PJ_FWD, a);
    //     // ROS_INFO("lon:%.8lf lat:%.8lf", a.lp.lam, a.lp.phi);
    //     // // 输出高斯正算值
    //     // ROS_INFO("x: %.3lf, y: %.3lf", b.enu.e, b.enu.n);
    //     UTMCoord trajPose;
    //     LatLonToUTM(b_lat/180*PI,b_lon/180*PI,49,trajPose);
    //     _point.x = point_.pose.position.x = trajPose.x - InitPose.x;
    //     _point.y = point_.pose.position.y = trajPose.y - InitPose.y;
    //     global_path.push_back(point_);
    //     global_point.push_back(_point);
    // }

    geometry_msgs::PoseStamped point_init;
    point_init.pose.position.x =  std::numeric_limits<float>::max();
    point_init.pose.position.y =  std::numeric_limits<float>::max();
    point_init.pose.position.z =  std::numeric_limits<float>::max();
    global_path.push_back(point_init);

    //自动生成的轨迹
    //vector<vector<float> > pathpoints;
    //Generate_target_point(0, 0, 100, 3, pathpoints);

//    path.header.stamp = ros::Time::now();
//    path.header.frame_id = "world";
//    geometry_msgs::PoseStamped point_;
//    geometry_msgs::Point _point;
//    for(int i = 0; i< pathpoints.size(); i++)
//    {
//     _point.x = point_.pose.position.x = pathpoints[i][0];
//     _point.y = point_.pose.position.y = pathpoints[i][1];
//     _point.z = point_.pose.position.z = -20; // 初始化 path 高度


//     global_path.push_back(point_);
//     global_point.push_back(_point);
//     path.poses.push_back(point_);
//    }
    search_point_sub = nh.subscribe("/search_point_msg", 5, &SearchPoint_callback);
    ClickPoint_sub = nh.subscribe("/clicked_point", 5, &ClickPoint_callback);
    subSimOdometry = nh.subscribe("/rexrov/pose_gt",5,&SimPose_callback);
    Pub_path_signal = nh.subscribe("/plan_bool",5,&Pub_path_callback);

    target_msg_pub = nh.advertise<visualization_msgs::Marker>("/target_text", 10);
    markerArrayPub = nh.advertise<visualization_msgs::MarkerArray>("/mission_point_text", 10);
    pubNextPoint = nh.advertise<geometry_msgs::PointStamped>("/goto_point", 5);
    pubOdometry = nh.advertise<nav_msgs::Odometry>("/odom/rov", 10);
    pubOdometry2 = nh.advertise<nav_msgs::Odometry>("/odom/vessel", 10);
    pubSonarPoint = nh.advertise<sensor_msgs::PointCloud2>("/sonar_cloud", 10);
    pubpath = nh.advertise<nav_msgs::Path>("/GlobalPath", 3);
    pubpath3 = nh.advertise<nav_msgs::Path>("/targetPath", 3);
    spline_pub = nh.advertise<visualization_msgs::MarkerArray>("/path_spline",10);

    image_transport::ImageTransport it(nh);
    sonarImage_pub = it.advertise("/sonar_image", 10);

    RosPose.header.frame_id = "/rexrov/base_footprint";
    RosPose.child_frame_id = "/rexrov/base_stabilized";
    RosPoseTrans.frame_id_ = "/rexrov/base_footprint";
    RosPoseTrans.child_frame_id_ = "/rexrov/base_stabilized";

    VesselPose.header.frame_id = "/world";
    VesselPose.child_frame_id = "/vessel_link";
    VesselPoseTrans.frame_id_ = "/world";
    VesselPoseTrans.child_frame_id_ = "/vessel_link";    
    // 创建 websocket server
    // 参考 mongoose 库用法
    // websocket server: https://github.com/cesanta/mongoose/blob/master/examples/websocket_chat/websocket_chat.c
    struct mg_mgr mgr;
    struct mg_connection *nc;

    signal(SIGTERM, signal_handler);
    signal(SIGINT,  signal_handler);

    mg_mgr_init(&mgr, NULL);// 创建并初始化事件管理器

    nc = mg_bind(&mgr, m_http_port, ev_handler);
    mg_set_protocol_http_websocket(nc);// 创建监听链接

    m_http_server_opts.document_root = ".";  // Serve current directory
    m_http_server_opts.enable_directory_listing = "no";

    ROS_INFO("start websocket server on port %s", m_http_port);
    uint64_t last_ = 0;
    // 接收数据
	// ros::Rate loop_rate(20);
    while (m_signal_received == 0 && ros::ok()) {

        ros::spinOnce();
        
        mg_mgr_poll(&mgr, 5); // 创建事件循环，1毫秒用于 mongoose 处理 
        // loop_rate.sleep(); 
        uint64_t now = clock_ms();
        if(now - last_ > 1000)
        {
            last_ = now;
            if (path_signal.data == 1){
                pubpath.publish(path);
                publish_markers(&spline_pub, global_point, ros::Time::now(),"world");
            }
                  
        }
    }
    mg_mgr_free(&mgr);
	
    return 0;
}
