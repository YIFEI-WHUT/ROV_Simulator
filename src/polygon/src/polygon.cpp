#include "ros/ros.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Polygon.h"
#include "polygon/search_point.h"
#include <visualization_msgs/Marker.h>
#include "vector"
#include <iostream>

float r=0;


void SearchPoint_callback(polygon::search_point msgs){
  int temp = msgs.r;
  r = float(temp);
  //std::cout<<"r="<<r<<std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "polygon");  //初始化ros，并命令节点名。
  ros::NodeHandle n;        //初始化节点，调用ros api接口句柄。
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PolygonStamped>("/rexrov/sonar_polygon", 1000);
  ros::Publisher markerPub = n.advertise<visualization_msgs::Marker>("/rexrov/text", 10);
  ros::Subscriber search_point_sub = n.subscribe<polygon::search_point>("/search_point_msg", 5, &SearchPoint_callback);
  ros::Rate loop_rate(5);  //设置发布频率
  std::vector<geometry_msgs::Point32> h;

  geometry_msgs::PolygonStamped msg;

  geometry_msgs::Point32 sonar_p;
  geometry_msgs::Point32 p1;
  geometry_msgs::Point32 p2;


  visualization_msgs::Marker marker;

  marker.header.frame_id="/rexrov/base_stabilized";
  marker.header.stamp = ros::Time::now();
  marker.ns = "rov";
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
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 3;
  std::ostringstream str;
  str<<"ROV";
  marker.text=str.str();
  marker.pose=pose;


  while (ros::ok()){    //检测节点是否正常运行
    ros::spinOnce();    //处理回调函数，会返回，这里也可以不加，因为此节点没有回调函数。

    if (r != 0){
        h.clear();
        sonar_p.x=1.3;
        sonar_p.y=0;
        sonar_p.z=0.5;
        h.push_back(sonar_p);

        p1.y=-r/2*sqrt(3);
        p1.x=sonar_p.x+r/2;
        p1.z=0.5;
        h.push_back(p1);

        float y;
        for (float x=-r/2*sqrt(3);x<=r/2*sqrt(3);x=x+r/20){
           y=sqrt(r*r-x*x);
           geometry_msgs::Point32 p;
           p.x=sonar_p.x+y;
           p.y=x;
           p.z=0.5;
           h.push_back(p);
           //std::cout<<"x="<<x<<std::endl;
           //std::cout<<"r/20="<<r/20<<std::endl;
           //std::cout<<"r="<<r<<std::endl;
        }

        p2.y=r/2*sqrt(3);
        p2.x=sonar_p.x+r/2;
        p2.z=0.5;
        h.push_back(p2);

        msg.header.frame_id = "/rexrov/base_stabilized";    //填充要发布的消息
        msg.header.stamp = ros::Time::now();
        msg.polygon.points = h;

        chatter_pub.publish(msg);   //发布消息
        markerPub.publish(marker);
    }

    loop_rate.sleep();  //用于控制发布频率。
  }
  return 0;
}
