/*
 * ScaleCorrector.cpp
 *
 *  Created on: Jun 2, 2018
 *      Author: jarek
 */

#include <include/ScaleCorrector.h>


ScaleCorrector::ScaleCorrector(const float distance_to_drive_)
: dist_driven(0.00),
  dist_to_drive(distance_to_drive_),
  scale(1.00),
  first_check(true),
  scale_ready(false),
  got_cam_starting_position(false),
  scale_updated(false),
  odom_topic("/odom")
{
  nh.param<std::string>("/orb_slam2_ros/map_scale/odom_topic", odom_topic, odom_topic);
}

void ScaleCorrector::setCameraStartingPoint(const float x_, const float y_, const float z_) {

  // camera coordinate system isn't orientated the same as odom
  start_pt_cam.x =  z_; // x_;
  start_pt_cam.y = -x_; // y_;
  start_pt_cam.z = -y_; // z_;
  odom_sub = nh.subscribe(odom_topic, 3, &ScaleCorrector::odomCallback, this);
  got_cam_starting_position = true;

}

bool ScaleCorrector::gotCamPosition() {
  return got_cam_starting_position;
}

bool ScaleCorrector::isReady() {

  dist_driven = calculateDist(&start_pt_odom, &cur_pt_odom, DIM_2);

  if ( dist_driven > (5 * dist_to_drive) ) {
    // something went wrong, get position once again
    got_cam_starting_position = false;
    first_check = true;
    return scale_ready;

  } else if ( dist_driven >= dist_to_drive ) {
    scale_ready = true;
    odom_sub.shutdown();

  }

  ROS_INFO("Robot moved %.2f m out of %.2f m", dist_driven, dist_to_drive);
  return scale_ready;

}

void ScaleCorrector::calculateScale(const float x_, const float y_, const float z_) {

  if ( !scale_ready ) {
    return;
  }

  // camera coordinate system isn't orientated the same as odom (odom is ground truth here)
  end_pt_cam.x =  z_; // x_;
  end_pt_cam.y = -x_; // y_;
  end_pt_cam.z = -y_; // z_;

  double dist_map =  calculateDist(&start_pt_cam,  &end_pt_cam,  DIM_3);
  double dist_odom = calculateDist(&start_pt_odom, &cur_pt_odom, DIM_3);

  if ( dist_map > 0.01 && dist_odom > 0.01 ) {
    scale = float(dist_odom/dist_map);
    ROS_INFO("Map scale succesfully calculated as %.2f", scale);
  }
  scale_updated = true;
  return;

}

bool ScaleCorrector::isScaleUpdated() {
  return scale_updated;
}

float ScaleCorrector::getScale() {
  return scale;
}

double ScaleCorrector::calculateDist(const Point* pt1, const Point* pt2, int flag_) {

  // flag_ = 0 means 2D points (odom)
  // flag_ = 1 means 3D points
  double result;
  // switching from float to double here to avoid overflow
  // thresholds because small odometry calculations make result as big as e+10
  if ( flag_ == 1 ) {
    float x_squared = (pt1->x - pt2->x) * (pt1->x - pt2->x);
    float y_squared = (pt1->y - pt2->y) * (pt1->y - pt2->y);
    float z_squared = (pt1->z - pt2->z) * (pt1->z - pt2->z);
    result = double(sqrt(x_squared+y_squared+z_squared));
  } else if ( flag_ == 0 ){
    float x_squared = (pt1->x - pt2->x) * (pt1->x - pt2->x);
    float y_squared = (pt1->y - pt2->y) * (pt1->y - pt2->y);
    result = double(sqrt(x_squared+y_squared));
  }
  return result;
}


void ScaleCorrector::odomCallback(const nav_msgs::Odometry& msg_) {

  if ( first_check ) {

    start_pt_odom.x = msg_.pose.pose.position.x;
    start_pt_odom.y = msg_.pose.pose.position.y;
    start_pt_odom.z = msg_.pose.pose.position.z;
    cur_pt_odom = start_pt_odom;
    first_check = false;

  } else {

    cur_pt_odom.x = msg_.pose.pose.position.x;
    cur_pt_odom.y = msg_.pose.pose.position.y;
    cur_pt_odom.z = msg_.pose.pose.position.z;
  }

  return;
}

ScaleCorrector::~ScaleCorrector() {
  // TODO Auto-generated destructor stub
}

