#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <sort_track/IntList.h>
#include <cmath>
#include <vector> 
using namespace std;
// global variable
geometry_msgs::Point p_central, p_bbox;
visualization_msgs::Marker bbox_line, central_point;
vector<visualization_msgs::Marker> bbox_line_vector;
ros::Publisher marker_pub;
ros::Subscriber bbox_sub;
// define bbox's height/length/width and offset from vertices to central point
float h = 2.0;
float l = 0.8;
float w = 0.8;
float offset_x[8]={-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2,w/2};
float offset_y[8]={-l/2,l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2};
float offset_z[8]={h/2,h/2,h/2,h/2,-h/2,-h/2,-h/2,-h/2};
// float x1,x2,y1,y2,z1,z2;

void bbox_proc(const std_msgs::Float32MultiArray& msg)// msg:[x_h,y_h,depth,track_id]
{ 
  // set human central point
  central_point.points.clear();// set current human bbox central point
  bbox_line.points.clear();// set current human bbox_line
  int len = msg.data.size();
  for(uint32_t i = 0; i < len/4; ++i){
    p_central.x = msg.data.at(4*i+2);//depth
    p_central.y = -msg.data.at(4*i);//x_h
    p_central.z = -msg.data.at(4*i+1);//y_h
    central_point.points.push_back(p_central);
  
    // Create the vertices for the points and lines
    // (1)plot upper plant
    for (uint32_t j = 0; j < 4; ++j)
    {
      p_bbox.x = p_central.x + offset_x[j];
      p_bbox.y = p_central.y + offset_y[j];
      p_bbox.z = p_central.z + offset_z[j];
      bbox_line.points.push_back(p_bbox);
      if(j<3){
      p_bbox.x = p_central.x + offset_x[j+1];
      p_bbox.y = p_central.y + offset_y[j+1];
      p_bbox.z = p_central.z + offset_z[j+1];
      }
      else{
      p_bbox.x = p_central.x + offset_x[0];
      p_bbox.y = p_central.y + offset_y[0];
      p_bbox.z = p_central.z + offset_z[0];
      }      
      bbox_line.points.push_back(p_bbox);

    }
    // (2)plot down plant
    for (uint32_t j = 4; j < 8; ++j)
    {
      p_bbox.x = p_central.x + offset_x[j];
      p_bbox.y = p_central.y + offset_y[j];
      p_bbox.z = p_central.z + offset_z[j];
      //p_bbox.z = 0;
      bbox_line.points.push_back(p_bbox);
      if(j<7){
      p_bbox.x = p_central.x + offset_x[j+1];
      p_bbox.y = p_central.y + offset_y[j+1];
      p_bbox.z = p_central.z + offset_z[j+1];
      //p_bbox.z = 0;
      }
      else{
      p_bbox.x = p_central.x + offset_x[4];
      p_bbox.y = p_central.y + offset_y[4];
      p_bbox.z = p_central.z + offset_z[4];
      //p_bbox.z = 0;
      }
      bbox_line.points.push_back(p_bbox);

    }
    // (3)plot vertical lines
    for (uint32_t j = 0; j < 4; ++j)
    {
      p_bbox.x = p_central.x + offset_x[j];
      p_bbox.y = p_central.y + offset_y[j];
      p_bbox.z = p_central.z + offset_z[j];
      bbox_line.points.push_back(p_bbox);
      p_bbox.z = p_bbox.z - h;
      bbox_line.points.push_back(p_bbox);
    }
    marker_pub.publish(bbox_line);
  }
  marker_pub.publish(central_point);

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "bbox");
  ros::NodeHandle n;
  // configuration
  bbox_line.header.frame_id = "/camera_link";
  bbox_line.header.stamp = ros::Time::now();
  bbox_line.ns = "bbox_points";
  bbox_line.action = visualization_msgs::Marker::ADD;
  bbox_line.pose.orientation.w = 1.0;

  central_point.header.frame_id = "/camera_link";
  central_point.header.stamp = ros::Time::now();
  central_point.ns = "central_point";
  central_point.action = visualization_msgs::Marker::ADD;
  central_point.pose.orientation.w = 1.0;

  bbox_line.id = 0;
  central_point.id = 1;

  bbox_line.type = visualization_msgs::Marker::LINE_LIST;
  central_point.type = visualization_msgs::Marker::POINTS;

  // POINTS markers use x and y scale for width/height respectively
  central_point.scale.x = 0.02;
  central_point.scale.y = 0.02;
  central_point.scale.z = 0.02;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  bbox_line.scale.x = 0.01;

  // Line strip is blue
  bbox_line.color.b = 1.0;
  bbox_line.color.a = 1.0;

  // Points are green
  central_point.color.g = 1.0f;
  central_point.color.a = 1.0;

  // define pub and sub
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  bbox_sub = n.subscribe("sort_track_deep", 100, bbox_proc);
  ros::Rate r(30);
  
  ros::spin();
  // while (ros::ok())
  // { 
    

  //   // marker_pub.publish(central_point);
  //   // marker_pub.publish(bbox_line);
  //   ros::spinOnce();

  //   r.sleep();
  // }
  return 0;
}

