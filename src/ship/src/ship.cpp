#include "ros/ros.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Polygon.h"
#include "nav_msgs/Odometry.h"
#include "vector"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

float center_x=0;
float center_y=0;
float rol=0;
float p1dx,p1dy,p2dx,p2dy,p3dx,p3dy,p4dx,p4dy,p5dx,p5dy;
using namespace std;


#define pi 3.141592653

void VesselPose_callback(nav_msgs::Odometry msgs){
    center_x=msgs.pose.pose.position.x;
    center_y=msgs.pose.pose.position.y;
    double r,p,y;
    tf::Quaternion RQ2;
    tf::quaternionMsgToTF(msgs.pose.pose.orientation,RQ2);
    tf::Matrix3x3(RQ2).getRPY(r,p,y);
    rol=-y+pi/2;
    cout<<rol*(180/pi)<<endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ship");  //初始化ros，并命令节点名。
  ros::NodeHandle n;        //初始化节点，调用ros api接口句柄。
  ros::Publisher markerPub = n.advertise<visualization_msgs::Marker>("/ship_text", 10);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PolygonStamped>("ship_polygon", 1000);
  ros::Subscriber vessel_pose_sub = n.subscribe<nav_msgs::Odometry>("/odom/vessel", 5, &VesselPose_callback);
  ros::Rate loop_rate(5);  //设置发布频率
  std::vector<geometry_msgs::Point32> h;

  geometry_msgs::PolygonStamped msg;

  geometry_msgs::Point32 p1;
  geometry_msgs::Point32 p2;
  geometry_msgs::Point32 p3;
  geometry_msgs::Point32 p4;
  geometry_msgs::Point32 p5;

  while (ros::ok()){    //检测节点是否正常运行

    h.clear();

    p1dx=center_x+62.85;
    p1dy=center_y+0;
    p1.z=0;
    p1.x=(p1dx-center_x)*cos(rol)-(p1dy-center_y)*sin(rol)+center_x;
    p1.y=(p1dx-center_x)*sin(rol)+(p1dy-center_y)*cos(rol)+center_y;
    h.push_back(p1);

    p2dx=center_x+40;
    p2dy=center_y+12.5;
    p2.z=0;
    p2.x=(p2dx-center_x)*cos(rol)-(p2dy-center_y)*sin(rol)+center_x;
    p2.y=(p2dx-center_x)*sin(rol)+(p2dy-center_y)*cos(rol)+center_y;
    h.push_back(p2);

    p3dx=center_x+(-62.85);
    p3dy=center_y+12.5;
    p3.z=0;
    p3.x=(p3dx-center_x)*cos(rol)-(p3dy-center_y)*sin(rol)+center_x;
    p3.y=(p3dx-center_x)*sin(rol)+(p3dy-center_y)*cos(rol)+center_y;
    h.push_back(p3);

    p4dx=center_x+(-62.85);
    p4dy=center_y+(-12.5);
    p4.z=0;
    p4.x=(p4dx-center_x)*cos(rol)-(p4dy-center_y)*sin(rol)+center_x;
    p4.y=(p4dx-center_x)*sin(rol)+(p4dy-center_y)*cos(rol)+center_y;
    h.push_back(p4);

    p5dx=center_x+40;
    p5dy=center_y+(-12.5);
    p5.z=0;
    p5.x=(p5dx-center_x)*cos(rol)-(p5dy-center_y)*sin(rol)+center_x;
    p5.y=(p5dx-center_x)*sin(rol)+(p5dy-center_y)*cos(rol)+center_y;
    h.push_back(p5);

    msg.header.frame_id = "world";    //填充要发布的消息
    msg.header.stamp = ros::Time::now();
    msg.polygon.points = h;

    //////////////////////////////////////////
    visualization_msgs::Marker marker;

    marker.header.frame_id="/world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "ship";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id =0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.scale.z = 3;
    marker.color.b = 0;
    marker.color.g = 0;
    marker.color.r = 255;
    marker.color.a = 1;
    marker.lifetime = ros::Duration(1.0);

    geometry_msgs::Pose pose;
    pose.position.x = center_x;
    pose.position.y = center_y;
    pose.position.z = 3;
    std::ostringstream str;
    str<<"SHEN QIAN HAO";
    marker.text=str.str();
    marker.pose=pose;
    //////////////////////////////////////////


    ros::spinOnce();    //处理回调函数，会返回，这里也可以不加，因为此节点没有回调函数。

    chatter_pub.publish(msg);   //发布消息
    markerPub.publish(marker);

    loop_rate.sleep();  //用于控制发布频率。
  }
  return 0;
}
