/*
 * ScaleCorrector.h
 *
 *  Created on: Jun 2, 2018
 *      Author: jarek
 */

#ifndef SRC_SCALECORRECTOR_H_
#define SRC_SCALECORRECTOR_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class ScaleCorrector {

 public:

  ScaleCorrector(const float distance_to_drive_);
  void  setCameraStartingPoint(const float x_, const float y_, const float z_);
  bool  gotCamPosition();
  bool  isReady();
  void  calculateScale(const float x_, const float y_, const float z_);
  bool  isScaleUpdated();
  float getScale();
  virtual ~ScaleCorrector();

 private:

  struct Point {
    float x, y, z;
  } start_pt_odom, cur_pt_odom, start_pt_cam, end_pt_cam;

  double calculateDist(const Point* pt1, const Point* pt2, int flag_);
  void odomCallback(const nav_msgs::Odometry& msg_);
  ros::NodeHandle nh;
  std::string odom_topic;
  ros::Subscriber odom_sub;

  bool got_cam_starting_position;
  bool first_check;
  bool scale_ready, scale_updated;
  float dist_driven, dist_to_drive, scale;
  const int DIM_2 = 0;
  const int DIM_3 = 1;


};

#endif /* SRC_SCALECORRECTOR_H_ */
