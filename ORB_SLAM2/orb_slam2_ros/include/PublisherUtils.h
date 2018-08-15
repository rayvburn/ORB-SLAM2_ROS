/*
 * PublisherUtils.h
 *
 *  Created on: Jun 1, 2018
 *      Author: jarek
 */

#ifndef SRC_PUBLISHERUTILS_H_
#define SRC_PUBLISHERUTILS_H_

#include "PublisherUtils_impl.h"
#include <sensor_msgs/PointCloud2.h>
#include <tf/LinearMath/Transform.h>
#include <orb_slam2_ros/ORBState.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "ROSPublisher.h"

namespace PublisherUtils {

const orb_slam2_ros::ORBState toORBStateMessage(ORB_SLAM2::Tracking::eTrackingState trackingState);
geometry_msgs::PoseStamped    getPoseStamped(const tf::Transform* tf_, const std::string* frame_id_);
sensor_msgs::PointCloud2      convertToPCL2( const std::vector<ORB_SLAM2::MapPoint*> &map_points, const double &map_scale, const float &camera_height);
template<typename Q> Q        convertToQuaternion(const cv::Mat& rot);
template<typename T> T        getROSParam(ros::NodeHandle nh, std::string param_name, T default_value);void transformPoint(cv::Mat& pos_, const double &scale_, bool scale_only_, const unsigned char frame_flag_, const float &camera_height_);
tf::Quaternion                quaternionFromRPY(float roll_, float pitch_, float yaw_);
tf::Transform                 createTF(tf::Vector3 vect_, tf::Quaternion quat_);
const char*                   stateDescription(orb_slam2_ros::ORBState orb_state);
cv::Mat                       computeCameraTransform(const cv::Mat& Twc, const double scale = 1.00);
void                          grayscaleToFile(const std::string& filename, const cv::Mat& img);
bool                          isBigEndian();
int                           erodeNaN(cv::Mat &matrix, int n);

// ------------------------------------------------------------ //

const bool IS_BIG_ENDIAN = PublisherUtils::isBigEndian();

/*
 * Some hard-coded rotation matrices in homogeneous coordinates
 */
/*
 * ------------------- Rotation around X axis
 */
const cv::Mat R_90X = (cv::Mat_<float>(4,4) <<  1, 0, 0, 0,
                                                0, 0,-1, 0,
                                                0, 1, 0, 0,
                                                0, 0, 0, 1);

const cv::Mat R_180X = (cv::Mat_<float>(4,4) <<  1, 0, 0, 0,
                                                 0,-1, 0, 0,
                                                 0, 0,-1, 0,
                                                 0, 0, 0, 1);

const cv::Mat R_270X = (cv::Mat_<float>(4,4) << 1, 0, 0, 0,
                                                0, 0,+1, 0,
                                                0,-1, 0, 0,
                                                0, 0, 0, 1);
/*
 * ------------------- Rotation around Y axis
 */
const cv::Mat R_90Y = (cv::Mat_<float>(4,4) <<  0, 0, 1, 0,
                                                0, 1, 0, 0,
                                               -1, 0, 0, 0,
                                                0, 0, 0, 1);

const cv::Mat R_270Y = (cv::Mat_<float>(4,4) << 0, 0,-1, 0,
                                                0, 1, 0, 0,
                                               +1, 0, 0, 0,
                                                0, 0, 0, 1);
/*
 * ------------------- Rotation around Z axis
 */
const cv::Mat R_90Z = (cv::Mat_<float>(4,4) <<  0, 1, 0, 0,
                                               -1, 0, 0, 0,
                                                0, 0, 1, 0,
                                                0, 0, 0, 1);

const cv::Mat R_270Z = (cv::Mat_<float>(4,4) << 0,+1, 0, 0,
                                               -1, 0, 0, 0,
                                                0, 0, 1, 0,
                                                0, 0, 0, 1);

const cv::Mat R_0 = (cv::Mat_<float>(4,4) <<    1, 0, 0, 0,
                                                0, 1, 0, 0,
                                                0, 0, 1, 0,
                                                0, 0, 0, 1);
static const float DEG_RAD_MULT = 2 * M_PI / 360;

}

#endif /* SRC_PUBLISHERUTILS_H_ */
