//
// Created by sebastiano on 8/18/16.
//

#include <include/ROSPublisher.h>
#include "FrameDrawer.h"
#include "Tracking.h"
#include "LoopClosing.h"
#include "utils.h"
#include "System.h"

#include <thread>
#include <sstream>
#include <cassert>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Path.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>

#include <orb_slam2_ros/ORBState.h>
#include <cv_bridge/cv_bridge.h>

#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <chrono>
#include <stdint.h>

using namespace ORB_SLAM2;

ROSPublisher::ROSPublisher(Map *map, double frequency, ros::NodeHandle nh) :
    IMapPublisher(map),
    drawer_(GetMap()),
    nh_(std::move(nh)),
    pub_rate_(frequency),
    lastBigMapChange_(-1),
    octomap_tf_based_(false),
    octomap_(PublisherUtils::getROSParam<float>(nh, "/orb_slam2_ros/octomap/resolution", 0.1)),
    pointcloud_chunks_stashed_(0),
    clear_octomap_(false),
    localize_only(false),
    map_scale_(1.50),
    perform_scale_correction_(true),
    scaling_distance_(1.00),
    camera_height_(0.205),
    camera_height_mult_(1.0),
    camera_height_corrected_(camera_height_*camera_height_mult_),
    publish_octomap_(false), publish_projected_map_(true), publish_gradient_map_(false)
{

    initializeParameters(nh);
    orb_state_.state = orb_slam2_ros::ORBState::UNKNOWN;

    // initialize publishers
    map_pub_         = nh_.advertise<sensor_msgs::PointCloud2>("map", 3);
    map_updates_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_updates", 3);
    image_pub_       = nh_.advertise<sensor_msgs::Image>("frame", 5);
    state_pub_       = nh_.advertise<orb_slam2_ros::ORBState>("info/state", 10);
    state_desc_pub_  = nh_.advertise<std_msgs::String>("info/state_description", 10);
    kp_pub_          = nh_.advertise<std_msgs::UInt32>("info/frame_keypoints", 1);
    kf_pub_          = nh_.advertise<std_msgs::UInt32>("info/map_keyframes", 1);
    mp_pub_          = nh_.advertise<std_msgs::UInt32>("info/matched_points", 1);
    loop_close_pub_  = nh_.advertise<std_msgs::Bool>("info/loop_closed", 2);
    trajectory_pub_  = nh_.advertise<nav_msgs::Path>("cam_path", 2);

    // initialize subscribers
    mode_sub_       = nh_.subscribe("switch_mode",    1, &ROSPublisher::localizationModeCallback,   this);
    clear_path_sub_ = nh_.subscribe("clear_cam_path", 1, &ROSPublisher::clearCamTrajectoryCallback, this);

    if (octomap_enabled_)
    {
      if ( publish_octomap_ ) {
        octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap", 3);
      }
      if ( publish_projected_map_ ) {
        projected_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("projected_map", 5, 10);
        projected_morpho_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("projected_morpho_map", 5, 10);
      }
      if ( publish_gradient_map_ ) {
        gradient_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("gradient_map", 5, 10);
      }
    }

    if ( perform_scale_correction_ ) { // TODO: make available only for monocular cameras

      try {

        // only "z" translation component is used (to put PCL/octomap higher/lower)
        tf::StampedTransform tf_target;
        tf::Vector3 cam_base_translation_tf_;

        tf_listener_.waitForTransform(camera_frame_, base_frame_, ros::Time::now(), ros::Duration(1.0));
        tf_listener_.lookupTransform (camera_frame_, base_frame_, ros::Time(0), tf_target);

        cam_base_translation_tf_.setX(tf_target.getOrigin().x());
        cam_base_translation_tf_.setY(tf_target.getOrigin().y());
        cam_base_translation_tf_.setZ(tf_target.getOrigin().z());
        cam_base_translation_tf_.setW(tfScalar(1.0));

        // rotation base -> camera applied to corresponding translation from real world
        tf::Transform tf = PublisherUtils::createTF(cam_base_translation_tf_,
                                                    PublisherUtils::quaternionFromRPY(0.0, 0.0, 0.0));

        tf::Transform tf_correction = PublisherUtils::createTF(tf::Vector3(0.0, 0.0, 0.0),
                                                               PublisherUtils::quaternionFromRPY(90.0, 0.0, -90.0));
        tf = tf_correction * tf;

        camera_height_ = tf.getOrigin().z();
        ROS_INFO("camera height: %.1f", camera_height_);

      } catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(3.0).sleep();
      }

    } else {

      ROS_INFO("camera height: %.1f", camera_height_);

    }

    // used because of scale correction coefficient non-ideal estimation
    camera_height_corrected_ = camera_height_ * camera_height_mult_;

}

void ROSPublisher::initializeParameters(ros::NodeHandle &nh) {

  // freq and image_topic are defined in ros_mono.cc
  nh.param<float>("/orb_slam2_ros/topic/orb_state_republish_rate", orb_state_republish_rate_, 1);

  // odom topic defined in ScaleCorrector.cpp
  nh.param<bool>("/orb_slam2_ros/map_scale/perform_correction",        perform_scale_correction_,  true);
  nh.param<float>("/orb_slam2_ros/map_scale/scaling_distance",         scaling_distance_,          1.000);
  nh.param<float>("/orb_slam2_ros/map_scale/set_manually",             map_scale_,                 1.500);
  nh.param<float>("/orb_slam2_ros/map_scale/camera_height",            camera_height_,             0.205);
  nh.param<float>("/orb_slam2_ros/map_scale/camera_height_multiplier", camera_height_mult_,        1.000);

  nh.param<std::string>("/orb_slam2_ros/frame/map_frame",          map_frame_,          ROSPublisher::DEFAULT_MAP_FRAME);
  nh.param<std::string>("/orb_slam2_ros/frame/map_frame_adjusted", map_frame_adjusted_, "/orb_slam2/odom");
  nh.param<std::string>("/orb_slam2_ros/frame/camera_frame",       camera_frame_,       ROSPublisher::DEFAULT_CAMERA_FRAME);
  nh.param<std::string>("/orb_slam2_ros/frame/base_frame",         base_frame_,         "/orb_slam2/base_link");

  nh.param<bool>("/orb_slam2_ros/octomap/enabled",                octomap_enabled_,        true);
  nh.param<bool>("/orb_slam2_ros/octomap/publish_octomap",        publish_octomap_,        false);
  nh.param<bool>("/orb_slam2_ros/octomap/publish_projected_map",  publish_projected_map_,  true);
  nh.param<bool>("/orb_slam2_ros/octomap/publish_gradient_map",   publish_gradient_map_,   false);

  nh.param<bool>("/orb_slam2_ros/octomap/rebuild",  octomap_rebuild_, false);
  nh.param<float>("/orb_slam2_ros/octomap/rate",    octomap_rate_,    1.0);
  // resolution is set default in constructor

  nh.param<double>("/orb_slam2_ros/occupancy/projected_map/min_height", projection_min_height_,  -10.0);
  nh.param<double>("/orb_slam2_ros/occupancy/projected_map/max_height", projection_max_height_,  +10.0);

  nh.param<int>   ("/orb_slam2_ros/occupancy/projected_map/morpho_oprations/erode_se_size",  erode_se_size_,  3);
  nh.param<int>   ("/orb_slam2_ros/occupancy/projected_map/morpho_oprations/erode_nb",       erode_nb_,       1);
  nh.param<int>   ("/orb_slam2_ros/occupancy/projected_map/morpho_oprations/open_se_size",   open_se_size_,   3);
  nh.param<int>   ("/orb_slam2_ros/occupancy/projected_map/morpho_oprations/open_nb",        open_nb_,        1);
  nh.param<int>   ("/orb_slam2_ros/occupancy/projected_map/morpho_oprations/close_se_size",  close_se_size_,  3);
  nh.param<int>   ("/orb_slam2_ros/occupancy/projected_map/morpho_oprations/close_nb",       close_nb_,       1);
  nh.param<int>   ("/orb_slam2_ros/occupancy/projected_map/morpho_oprations/erode2_se_size", erode2_se_size_, 3);
  nh.param<int>   ("/orb_slam2_ros/occupancy/projected_map/morpho_oprations/erode2_nb",      erode2_nb_,      1);

  nh.param<float> ("/orb_slam2_ros/occupancy/height_gradient_map/max_height",   gradient_max_height_,   0);
  nh.param<int>   ("/orb_slam2_ros/occupancy/height_gradient_map/nb_erosions",  gradient_nb_erosions_,  1);
  nh.param<float> ("/orb_slam2_ros/occupancy/height_gradient_map/low_slope",    gradient_low_slope_,    M_PI / 4.0);
  nh.param<float> ("/orb_slam2_ros/occupancy/height_gradient_map/high_slope",   gradient_high_slope_,   M_PI / 3.0);

  std::cout << endl;
  std::cout << "ROS Publisher parameters" << endl;
  std::cout << "TOPIC" << endl;
  std::cout << "- orb_state_republish_rate:  " << orb_state_republish_rate_ << std::endl;
  std::cout << "MAP SCALE" << endl;
  std::cout << "- perform_correction:  " << perform_scale_correction_ << std::endl;
  std::cout << "- set_manually:  " << map_scale_ << std::endl;
  std::cout << "- camera_height:  " << camera_height_ << std::endl;
  std::cout << "- camera_height_multiplier:  " << camera_height_mult_ << std::endl;
  std::cout << "FRAME" << endl;
  std::cout << "- map_frame:  " << map_frame_ << std::endl;
  std::cout << "- map_frame_adjusted:  " << map_frame_adjusted_ << std::endl;
  std::cout << "- camera_frame:  " << camera_frame_ << std::endl;
  std::cout << "- base_frame:  " << base_frame_ << std::endl;
  std::cout << "OCTOMAP" << endl;
  std::cout << "- octomap/enabled:  " << octomap_enabled_ << std::endl;
  std::cout << "- octomap/publish_octomap:  " << publish_octomap_ << std::endl;
  std::cout << "- octomap/publish_projected_map:  " << publish_projected_map_ << std::endl;
  std::cout << "- octomap/publish_gradient_map:  " << publish_gradient_map_ << std::endl;
  std::cout << "- octomap/rebuild:  " << octomap_rebuild_ << std::endl;
  std::cout << "- octomap/rate:  " << octomap_rate_ << std::endl;
  std::cout << "OCCUPANCY/PROJECTED_MAP" << endl;
  std::cout << "- projected_map/min_height:  " << projection_min_height_ << std::endl;
  std::cout << "- projected_map/max_height:  " << projection_max_height_ << std::endl;
  std::cout << "OCCUPANCY/PROJECTED_MAP/MORPHO" << endl;
  std::cout << "- open_se_size:  " << open_se_size_ << std::endl;
  std::cout << "- open_nb:  " << open_nb_ << std::endl;
  std::cout << "- close_se_size:  " << close_se_size_ << std::endl;
  std::cout << "- close_nb:  " << close_nb_ << std::endl;
  std::cout << "- erode_se_size:  " << erode_se_size_ << std::endl;
  std::cout << "- erode_nb:  " << erode_nb_ << std::endl;
  std::cout << "OCCUPANCY/GRADIENT_MAP" << endl;
  std::cout << "- max_height:  " << gradient_max_height_ << std::endl;
  std::cout << "- nb_erosions:  " << gradient_nb_erosions_ << std::endl;
  std::cout << "- low_slope:  " << gradient_low_slope_ << std::endl;
  std::cout << "- high_slope:  " << gradient_high_slope_ << std::endl;
  std::cout << endl;

  // DEPRECATED
  // nh.param<bool>("/orb_slam2_ros/octomap/tf_based", octomap_tf_based_, false);
  // nh.param<bool>("/orb_slam2_ros/frame/align_map_to_cam_frame",   align_map_to_cam_frame_, true);
  // nh.param<bool>("/orb_slam2_ros/frame/adjust_map_frame",          adjust_map_frame_,      false);
  // nh.param<float>("/orb_slam2_ros/topic/loop_close_republish_rate_", loop_close_republish_rate_, ROSPublisher::LOOP_CLOSE_REPUBLISH_RATE);

}

/*
 * Either appends all GetReferenceMapPoints to the pointcloud stash or clears the stash and re-fills it
 * with GetAllMapPoints, in case there is a big map change in ORB_SLAM 2 or all_map_points is set to true.
 */
void ROSPublisher::stashMapPoints(bool all_map_points)
{
    std::vector<MapPoint*> map_points;

    pointcloud_map_points_mutex_.lock();

    if (all_map_points || GetMap()->GetLastBigChangeIdx() > lastBigMapChange_)
    {
        map_points = GetMap()->GetAllMapPoints();
        lastBigMapChange_ = GetMap()->GetLastBigChangeIdx();
        clear_octomap_ = true;
        pointcloud_map_points_.clear();
        pointcloud_chunks_stashed_ = 1;

    } else {

        map_points = GetMap()->GetReferenceMapPoints();
        pointcloud_chunks_stashed_++;
    }

    for (MapPoint *map_point : map_points) {
        if (map_point->isBad()) {
            continue;
        }
        cv::Mat pos = map_point->GetWorldPos();
        PublisherUtils::transformPoint(pos, map_scale_, true, 1, camera_height_corrected_);
        pointcloud_map_points_.push_back(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
    }

    pointcloud_map_points_mutex_.unlock();
}

/*
 * Octomap worker thread function, which has exclusive access to the octomap. Updates and publishes it.
 */

void ROSPublisher::octomapWorker()
{

    static std::chrono::system_clock::time_point this_cycle_time;

    octomap::pose6d frame;
    octomap::point3d origin = { 0.0, 0.0, 0.0 };
    bool got_tf = false;

    // wait until ORB_SLAM 2 is up and running
    ROS_INFO("octomapWorker thread: waiting for ORBState OK");

    while (orb_state_.state != orb_slam2_ros::ORBState::OK)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    ROS_INFO("octomapWorker thread: starting to work (ORBState is OK)");

    // main thread loop
    while (!isStopped())
    {
        this_cycle_time = std::chrono::system_clock::now();

        if ( !got_tf ) {

          try {

            tf::StampedTransform transform_in_target_frame;
            tf_listener_.waitForTransform(base_frame_, camera_frame_, ros::Time(0), ros::Duration(1.0));
            tf_listener_.lookupTransform( base_frame_, camera_frame_, ros::Time(0), transform_in_target_frame);
            static const tf::Transform octomap = PublisherUtils::createTF(tf::Vector3(tfScalar(0.0),
                                                                                      tfScalar(0.0),
                                                                                      tfScalar(camera_height_corrected_)),
                                                                          transform_in_target_frame.getRotation() );
            frame = octomap::poseTfToOctomap(octomap);
            got_tf = true;

          } catch (tf::TransformException &ex) {

            frame = octomap::pose6d(0, 0, 0, 0, 0, 0);
            got_tf = false;

          }

        }

        if (got_tf || octomap_rebuild_ )
        {
          // clear whenever TF mode changes
          clear_octomap_ |= (got_tf != octomap_tf_based_);

          if (clear_octomap_)
          {
            // WARNING: causes ugly segfaults in octomap 1.8.0
            octomap_.clear();
            ROS_INFO("octomapWorker: octomap cleared, rebuilding...");

            /*
             * TODO: if pointcloud is supposed to be a lidar scan result, this is problematic
             * (multiple hits on one beam/previous hits getting overwritten etc.)
             *
             */
            stashMapPoints(true);     // stash whole map
            clear_octomap_ = false;   // TODO: mutex?
          }

          pointcloud_map_points_mutex_.lock();
          octomap_.insertPointCloud(pointcloud_map_points_, origin, frame);

          pointcloud_map_points_.clear();
          int pointcloud_chunks_stashed = pointcloud_chunks_stashed_;
          pointcloud_chunks_stashed_ = 0;
          pointcloud_map_points_mutex_.unlock();

          octomap_tf_based_ = got_tf;

          if ( publish_octomap_ ) {
            //ROS_INFO("Publishing Octomap...");
            publishOctomap();
            //ROS_INFO("Octomap published");
          }
          if ( publish_projected_map_ ) {
            //ROS_INFO("Publishing Projected map...");
            publishProjectedMap();
            //ROS_INFO("Projected map published");
          }
          if ( publish_gradient_map_ ) {
            //ROS_INFO("Publishing Gradient map...");
            publishGradientMap();
            //ROS_INFO("Gradient map published");
          }

          ROS_INFO("octomapWorker: finished cycle integrating %i pointcloud chunks.", pointcloud_chunks_stashed);
        }
        else
        {

          ROS_INFO("octomapWorker thread: missing camera TF, losing %i pointcloud chunks.", pointcloud_chunks_stashed_);
          pointcloud_map_points_mutex_.lock();
          pointcloud_map_points_.clear();
          pointcloud_chunks_stashed_ = 0;
          pointcloud_map_points_mutex_.unlock();

        }

        std::this_thread::sleep_until(this_cycle_time + std::chrono::milliseconds((int) (1000. / octomap_rate_)));
    }

    ROS_INFO("octomapWorker thread: stopped");
}

/*
 * Creates a 2D Occupancy Grid from the Octomap.
 */
void ROSPublisher::octomapCutToOccupancyGrid(const octomap::OcTree& octree, nav_msgs::OccupancyGrid& map, nav_msgs::OccupancyGrid& map_erode, const double minZ_, const double maxZ_ )
{

    static const uint8_t a = 1;
    static const uint8_t b = 0;
    static const uint8_t c = 2;

    map.info.resolution = octree.getResolution();
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    octree.getMetricMin(minX, minY, minZ);
    octree.getMetricMax(maxX, maxY, maxZ);
    ROS_DEBUG("Octree min %f %f %f", minX, minY, minZ);
    ROS_DEBUG("Octree max %f %f %f", maxX, maxY, maxZ);
    minZ = std::max(minZ_, minZ);
    maxZ = std::min(maxZ_, maxZ);

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey, maxKey, curKey;

    if (!octree.coordToKeyChecked(minPt, minKey))
    {
        ROS_ERROR("Could not create OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
        return;
    }
    if (!octree.coordToKeyChecked(maxPt, maxKey))
    {
        ROS_ERROR("Could not create OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
        return;
    }

    map.info.width = maxKey[b] - minKey[b] + 1;
    map.info.height = maxKey[a] - minKey[a] + 1;

    // might not exactly be min / max:
    octomap::point3d origin =   octree.keyToCoord(minKey, octree.getTreeDepth());

    /*
     * Aligns base_link with origin of map frame, but is not correct in terms of real environment
     * (real map's origin is in the origin of camera's origin)
     * map.info.origin.position.x = origin.x() - octree.getResolution() * 0.5 - cam_base_translation_.at<float>(0);
     * map.info.origin.position.y = origin.y() - octree.getResolution() * 0.5 - cam_base_translation_.at<float>(1);
     *
     */

    map.info.origin.position.x = origin.x() - octree.getResolution() * 0.5;
    map.info.origin.position.y = origin.y() - octree.getResolution() * 0.5;

    map.info.origin.orientation.x = 0.;
    map.info.origin.orientation.y = 0.;
    map.info.origin.orientation.z = 0.;
    map.info.origin.orientation.w = 1.;

    // Allocate space to hold the data
    map.data.resize(map.info.width * map.info.height, -1);

    // Matrix of map's size is inited with unknown (-1) value at each point
    for(std::vector<int8_t>::iterator it = map.data.begin(); it != map.data.end(); ++it) {
       *it = -1;
    }

    map_erode = map;  // plain copy of one struct - copy needed for second map version

    /*
     * Matrix for morphological operations
    ** Matrix's type set to Unsigned Char - additional loop for assigning matrix's
    ** values to map.data vector is still a must
    ** Another thing is that morphological operations behave strange with negative values inside a matrix
    ** It is probably because erode, close and open aren't supposed to deal with negative (-1) values inside matrix
    */

    // values in the matrix
    static const unsigned char MAT_UNKNOWN  = 10;
    static const unsigned char MAT_NON_OCC  = 50;
    static const unsigned char MAT_OCCUPIED = 100;

    cv::Mat map_data_matrix;
    map_data_matrix.create(map.info.height, map.info.width, CV_8U); // ensures that the matrix is continuous
    map_data_matrix.setTo(MAT_UNKNOWN);

    // map creation time
    ros::Time t_start = ros::Time::now();
    unsigned i, j;
    // iterate over all keys:
    for (curKey[a] = minKey[a], j = 0; curKey[a] <= maxKey[a]; ++curKey[a], ++j)
    {
        // pointer to the current row start
        uchar* mat_ptr = map_data_matrix.ptr<uchar>(j);

        for (curKey[b] = minKey[b], i = 0; curKey[b] <= maxKey[b]; ++curKey[b], ++i)
        {
            for (curKey[c] = minKey[c]; curKey[c] <= maxKey[c]; ++curKey[c])
            {   //iterate over height

                octomap::OcTreeNode* node = octree.search(curKey);
                if (node)
                {
                  // creates map data
                  bool occupied = octree.isNodeOccupied(node);
                  if(occupied) {

                      map.data[map.info.width * j + i] = 100;
                      mat_ptr[i] = MAT_OCCUPIED;
                      break;

                  } else {
                      map.data[map.info.width * j + i] = 0;
                      mat_ptr[i] = MAT_NON_OCC;
                  }
                }
            }
        }
    }

    /*
    ** Application of a morphological operations to map - they clear
    ** single points that are incorrectly interpreted as occupied
    */

    if ( projected_morpho_map_pub_.getNumSubscribers() > 0 ) {

      /* ERODE */
      if ( erode_nb_ > 0 ) {
        cv::erode(map_data_matrix,
                  map_data_matrix,
                  cv::getStructuringElement(cv::MorphShapes::MORPH_RECT,
                                            cv::Size(erode_se_size_,erode_se_size_),
                                            cv::Point(-1,-1)),
                  cv::Point(-1,-1),
                  erode_nb_,
                  cv::BORDER_CONSTANT,
                  cv::morphologyDefaultBorderValue());
      }

      /* OPEN */
      if ( open_nb_ > 0 ) {
        cv::morphologyEx(map_data_matrix,
                         map_data_matrix,
                         cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, // MORPH_CROSS,
                                                   cv::Size(open_se_size_,open_se_size_),
                                                   cv::Point(-1,-1)),
                         cv::Point(-1,-1),
                         open_nb_,
                         cv::BORDER_CONSTANT,
                         cv::morphologyDefaultBorderValue());
      }

      /* CLOSE */
      if ( close_nb_ > 0 ) {
        cv::morphologyEx(map_data_matrix,
                         map_data_matrix,
                         cv::MORPH_CLOSE,
                         cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, // MORPH_CROSS,
                                                   cv::Size(close_se_size_,close_se_size_),
                                                   cv::Point(-1,-1)),
                         cv::Point(-1,-1),
                         close_nb_,
                         cv::BORDER_CONSTANT,
                         cv::morphologyDefaultBorderValue());
      }

      /* ERODE */
      if ( erode2_nb_ > 0 ) {
        cv::erode(map_data_matrix,
                  map_data_matrix,
                  cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE,
                                            cv::Size(erode2_se_size_,erode2_se_size_),
                                            cv::Point(-1,-1)),
                  cv::Point(-1,-1),
                  erode2_nb_,
                  cv::BORDER_CONSTANT,
                  cv::morphologyDefaultBorderValue());
      }

      // nav_msgs/OccupancyGrid msg out of map after morphological operations
      for ( int j = 0; j < map_erode.info.height; j++) {

        // pointer to the current row start
        uchar* mat_ptr = map_data_matrix.ptr<uchar>(j);

        for ( int i = 0; i < map_erode.info.width; i++ ) {
          switch( mat_ptr[i] )
          {
            case MAT_UNKNOWN:
                map_erode.data[map_erode.info.width * j + i] = -1;
                break;
            case MAT_NON_OCC:
                map_erode.data[map_erode.info.width * j + i] = 0;
                break;
            case MAT_OCCUPIED:
                map_erode.data[map_erode.info.width * j + i] = 100;
                break;
          }
        }
      }
    }

    ros::Duration t_stop = (ros::Time::now() - t_start);
    ROS_INFO("Occupancy grid: %d x %d created in %.3f sec",  map_erode.info.width,
                                                             map_erode.info.height,
                                                             t_stop.toSec() );
}

/*
 * Constructs a 2-dimensional OccupancyGrid from an Octomap by evaluating its heightmap gradients.
 */
void ROSPublisher::octomapGradientToOccupancyGrid(const octomap::OcTree& octree, nav_msgs::OccupancyGrid& map, float max_height, int nb_erosions, float low_slope, float high_slope)
{
    // get tree dimensions
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    octree.getMetricMin(min_x, min_y, min_z);
    octree.getMetricMax(max_x, max_y, max_z);
    octomap::point3d min_point(min_x, min_y, min_z);
    octomap::point3d max_point(max_x, max_y, max_z);

    // fill in map dimensions
    map.info.resolution = octree.getResolution();
    map.info.width = (max_point.x() - min_point.x()) / map.info.resolution + 1;
    map.info.height = (max_point.y() - min_point.y()) / map.info.resolution + 1;

    map.info.origin.position.x = min_point.x() - map.info.resolution * 0.5;
    map.info.origin.position.y = min_point.y() - map.info.resolution * 0.5;

    map.info.origin.orientation.x = 0.;
    map.info.origin.orientation.y = 0.;
    map.info.origin.orientation.z = 0.;
    map.info.origin.orientation.w = 1.;

    // create CV matrix of proper size with 1 channel of 32 bit floats and init values to NaN for "unknown"
    cv::Mat height_map(map.info.height, map.info.width, CV_32FC1, NAN);

    // iterate over tree leafs to create height map
    octomap::point3d coord;
    int x, y;
    float z;
    for(octomap::OcTree::leaf_iterator it = octree.begin_leafs(), end=octree.end_leafs(); it != end; ++it)
    {
        if (octree.isNodeOccupied(*it))
        {
            coord = it.getCoordinate();
            x = (coord.x() - min_point.x()) / map.info.resolution;
            y = (coord.y() - min_point.y()) / map.info.resolution;
            z = coord.z(); // z-axis is facing UP
            if (z <= max_height) // only consider voxels up to specified height (e.g. for building indoor maps)
            {
                float current_height = height_map.at<float>(y, x);
                if (current_height != current_height || z > current_height)
                {
                    height_map.at<float>(y, x) = z;
                }
            }
        }
    }

    // fill in small holes
    PublisherUtils::erodeNaN(height_map, nb_erosions);
    // store where height is unknown
    cv::Mat mask_unknown = height_map != height_map; // is NaN

    PublisherUtils::erodeNaN(height_map, 1); // avoid discontinuity (and thus a "wall") around known area

    height_map.setTo(0, height_map != height_map); // get rid of all NaN trouble makers

    // get height gradient
    cv::Mat gradient_x, gradient_y, gradient_map;
    cv::Scharr(height_map, gradient_x, CV_32F, 1, 0, 1. / 16.);
    cv::Scharr(height_map, gradient_y, CV_32F, 0, 1, 1. / 16.);
    cv::addWeighted(cv::abs(gradient_x), 0.5, cv::abs(gradient_y), 0.5, 0, gradient_map); // TODO 0.5 rly?

    // height slope thresholds:
    // values < lower are considered free space
    // values > upper are considered obstacle
    // everything inbetween is literally a gray-zone
    float threshold_lower = sin(low_slope) / cos(low_slope) * map.info.resolution;
    float threshold_upper = sin(high_slope) / cos(high_slope) * map.info.resolution;

    // map data probabilities are in range [0,100].  Unknown is -1.
    gradient_map.setTo(threshold_upper, gradient_map > threshold_upper); // clip obstacles
    gradient_map.setTo(threshold_lower, gradient_map < threshold_lower); // clip free space
    gradient_map = (gradient_map - threshold_lower) / (threshold_upper - threshold_lower) * 100.0; // convert into map data range
    gradient_map.setTo(-1, mask_unknown); //replace NaNs

    // ensure correct size of map data vector
    map.data.resize(map.info.width * map.info.height);
    // fill in map data
    for(y = 0; y < gradient_map.rows; ++y) {
        for(x = 0; x < gradient_map.cols; ++x) {
            map.data[y * map.info.width + x] = gradient_map.at<float>(y, x);
        }
    }
}

/*
 * Publishes ORB_SLAM 2 GetAllMapPoints() as a PointCloud2.
 */
void ROSPublisher::publishMap()
{
    if (map_pub_.getNumSubscribers() > 0)
    {
        sensor_msgs::PointCloud2 msg = PublisherUtils::convertToPCL2(GetMap()->GetAllMapPoints(),
                                                                     map_scale_,
                                                                     camera_height_corrected_);
        msg.header.frame_id = map_frame_;
        map_pub_.publish(msg);

    }
}

/*
 * Publishes ORB_SLAM 2 GetReferenceMapPoints() as a PointCloud2.
 */
void ROSPublisher::publishMapUpdates()
{

    if (map_updates_pub_.getNumSubscribers() > 0)
    {
        /*
        sensor_msgs::PointCloud2 msg = PublisherUtils::convertToPCL2(GetMap()->GetAllMapPoints(),
                                                                     map_scale_,
                                                                     camera_height_corrected_);
         */
        sensor_msgs::PointCloud2 msg = PublisherUtils::convertToPCL2(GetMap()->GetReferenceMapPoints(),
                                                                     map_scale_,
                                                                     camera_height_corrected_);
        msg.header.frame_id = map_frame_;
        map_updates_pub_.publish(msg);
    }
}

/*
 * Publishes ORB_SLAM 2 GetCameraPose() as a TF.
 */
void ROSPublisher::publishCameraPose()
{

    // number of subscribers is unknown to a TransformBroadcaster
    cv::Mat xf = PublisherUtils::computeCameraTransform(GetCameraPose(), map_scale_);

    if (!xf.empty()) {

      /*
       * DESCRIPTION:
       * Map to camera transform is corrected pose of robot
       * in the world frame.
       * Therefore there is need to look for odom->camera transform
       * or just base_link->camera.
       * It will create correction in odometry coordinate system
       * to align it with real world frame.
       *
       */
      try {

          std::string source_frame;
          tf::StampedTransform tf_target;
          source_frame = map_frame_adjusted_;
          /* TODO: add as a parameter
          if ( adjust_map_frame_ ) {
            source_frame = map_frame_adjusted_; // interface?
          } else {
            source_frame = map_frame_;          // base_frame_; - buggy at the moment
          }
          */
          tf_listener_.lookupTransform(camera_frame_,  source_frame,
                                       ros::Time(0), tf_target);

          camera_position_ = {  xf.at<float>(0, 3),
                                xf.at<float>(1, 3),
                                xf.at<float>(2, 3) };

          tf::Quaternion orientation = PublisherUtils::convertToQuaternion<tf::Quaternion>(xf);

          /* ------------------------------------
           * Camera's pose in map coordinate system
           * divided into translation and rotation
           */
          tf::Transform Tmc = PublisherUtils::createTF(camera_position_,
                                                       orientation);

          /* ------------------------------------
           * Camera_optical pose in odom coordinate system
           * divided into translation and rotation
           */
          tf::Transform Toc = PublisherUtils::createTF(tf_target.getOrigin(),
                                                       tf_target.getRotation());

          /* ------------------------------------
           * Additional rotation for interface frame
           * (no translation component) - static tf
           */
          static const tf::Transform Toc_int = PublisherUtils::createTF(tf::Vector3(0.0, 0.0, 0.0),
                                                                        PublisherUtils::quaternionFromRPY(0.0, -90.0, 90.0));
          Toc = Toc * Toc_int; // rotation applied to already transformed system (odom is reference here)

          /* ------------------------------------
           * Additional camera translation (it is
           * mounted on top of the mobile base)
           */
          static const tf::Transform T_d_cam = PublisherUtils::createTF(tf::Vector3(0.0, 0.0, camera_height_),
                                                                        PublisherUtils::quaternionFromRPY(0.0, 0.0, 0.0));

          /* ------------------------------------
           * Final computations
           * TODO: interface frame should be allowed to be connected to /base_link or /odom
           * not only to /odom
           */
          tf::Transform Tmo = Toc.inverse() * Tmc;
          Tmo = T_d_cam * Tmo;

          tf::StampedTransform transform(Tmo, ros::Time::now(), map_frame_,  source_frame);
          camera_tf_pub_.sendTransform(transform);

          // camera trajectory extraction
          cam_pose_ = PublisherUtils::getPoseStamped(&Tmo, &camera_frame_);

          ResetCamFlag();


      } catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(3.0).sleep();
      }
    }
}

/*
 * Publishes the previously built Octomap. (called from the octomap worker thread)
 */
void ROSPublisher::publishOctomap()
{
    if (octomap_pub_.getNumSubscribers() > 0)
    {
        auto t0 = std::chrono::system_clock::now();
        octomap_msgs::Octomap msgOctomap;
        msgOctomap.header.frame_id = map_frame_;
        /* TODO: add as a parameter
        if ( adjust_map_frame_ ) {

          msgOctomap.header.frame_id =  octomap_tf_based_ ?
                                                map_frame_adjusted_ :
                                                map_frame_;

          msgOctomap.header.frame_id = octomap_frame_;
        } else {
          msgOctomap.header.frame_id =  map_frame_;
        }
        */
        msgOctomap.header.stamp = ros::Time::now();
        if (octomap_msgs::binaryMapToMsg(octomap_, msgOctomap))   // TODO: full/binary...?
        {
            auto tn = std::chrono::system_clock::now();
            auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(tn - t0);
            //std::cout << "msg generation time: " << dt.count() << " ms" << std::endl;
            t0 = std::chrono::system_clock::now();
            octomap_pub_.publish(msgOctomap);
            tn = std::chrono::system_clock::now();
            dt = std::chrono::duration_cast<std::chrono::milliseconds>(tn - t0);
            //std::cout << "msg publish time: " << dt.count() << " ms" << std::endl;
        }
    }
}

/*
 * Publishes the ORB_SLAM 2 tracking state as ORBState int and/or as a description string.
 */
void ROSPublisher::publishState(Tracking *tracking)
{

    if (tracking != NULL) {
        // save state from tracking, even if there are no subscribers
        orb_state_ = PublisherUtils::toORBStateMessage(tracking->mState);
    }

    if (state_pub_.getNumSubscribers() > 0)
    {
        // publish state as ORBState int
        orb_state_.header.stamp = ros::Time::now();
        state_pub_.publish(orb_state_);
    }

    if (state_desc_pub_.getNumSubscribers() > 0)
    {
        // publish state as string
        std_msgs::String state_desc_msg;
        // const char* test = PublisherUtils::stateDescription(orb_state_);
        state_desc_msg.data = PublisherUtils::stateDescription(orb_state_); // stateDescription(orb_state_);
        state_desc_pub_.publish(state_desc_msg);
    }

    // last_state_publish_time_ = ros::Time::now();
}

/*
 * Publishes the current ORB_SLAM 2 status image.
 */
void ROSPublisher::publishImage(Tracking *tracking)
{

    if (image_pub_.getNumSubscribers() > 0)
    {

      drawer_.Update(tracking);

      std_msgs::Header hdr;
      cv_bridge::CvImage cv_img {hdr, "bgr8", drawer_.DrawFrame()};

      auto image_msg = cv_img.toImageMsg();
      image_msg->header = hdr;
      image_pub_.publish(*image_msg);

    }
}

/*
 * Creates a 2D OccupancyGrid from the Octomap by performing a cut through a previously specified z interval and publishes it.
 */
void ROSPublisher::publishProjectedMap()
{

    int8_t proj_sub_nr = projected_map_pub_.getNumSubscribers();
    int8_t proj_ero_sub_nr = projected_morpho_map_pub_.getNumSubscribers();

    if ( proj_sub_nr | proj_ero_sub_nr ) {

        static nav_msgs::OccupancyGrid msg;
        static nav_msgs::OccupancyGrid msg_eroded;

        msg.header.frame_id = map_frame_;
        msg_eroded.header.frame_id = map_frame_;
        /*
        if ( adjust_map_frame_ ) {

          msg.header.frame_id =  octomap_tf_based_ ?
                                                map_frame_adjusted_ :
                                                map_frame_;

          msg.header.frame_id = octomap_frame_; == map
          msg_eroded.header.frame_id = octomap_frame_;
        } else {
          msg.header.frame_id =  map_frame_;
          msg_eroded.header.frame_id =  map_frame_;
        }
        */
        msg.header.stamp = ros::Time::now();
        msg_eroded.header.stamp = ros::Time::now();

        octomapCutToOccupancyGrid(octomap_, msg, msg_eroded, projection_min_height_, projection_max_height_);

        // one of maps published
        if ( proj_sub_nr > 0 ) {
          projected_map_pub_.publish(msg);
        } else {
          projected_morpho_map_pub_.publish(msg_eroded);
        }
    }
}

/*
 * Creates a 2D OccupancyGrid from the Octomap by evaluating its heightmap gradients and publishes it.
 */
void ROSPublisher::publishGradientMap()
{

    if (gradient_map_pub_.getNumSubscribers() > 0)
    {
        static nav_msgs::OccupancyGrid msg;
        msg.header.frame_id = map_frame_;
        /*
        if ( adjust_map_frame_ ) {

          msg.header.frame_id =  octomap_tf_based_ ?
                                                map_frame_adjusted_ :
                                                map_frame_;

          msg.header.frame_id = octomap_frame_; == map
        } else {
          msg.header.frame_id =  map_frame_;
        }
        */
        msg.header.stamp = ros::Time::now();

        octomapGradientToOccupancyGrid(octomap_, msg,
                                       gradient_max_height_, gradient_nb_erosions_,
                                       gradient_low_slope_,  gradient_high_slope_);

        gradient_map_pub_.publish(msg);
    }
}

void ROSPublisher::publishCamTrajectory() {

  static nav_msgs::Path msg;

  if ( clear_path_ ) {
    msg.poses.clear();
    clear_path_ = false;
  }

  if ( trajectory_pub_.getNumSubscribers() > 0) {
    msg.header.frame_id = map_frame_;
    msg.header.stamp = ros::Time::now();
    msg.poses.push_back(cam_pose_);
    trajectory_pub_.publish(msg);
  }

}

void ROSPublisher::publishLoopState(const bool &state) {
  std_msgs::Bool msg;
  msg.data = state;
  loop_close_pub_.publish(msg);
}

void ROSPublisher::publishUInt32Msg(const ros::Publisher &pub, const unsigned long &data) {
  std_msgs::UInt32 msg;
  msg.data = data;
  pub.publish(msg);
}

void ROSPublisher::checkMode() {

  static bool mode_last_state = false;        // default start with SLAM mode
  if ( mode_last_state != localize_only )
  {
    if ( localize_only ) {
      GetSystem()->ActivateLocalizationMode();
    } else {
      GetSystem()->DeactivateLocalizationMode();
    }
  }
  mode_last_state = localize_only;            // to prevent from periodic mode activation/deactivation - switch will be applied in case of changing value

}

void ROSPublisher::camInfoUpdater() {

  // these operations are moved from Run() to separate thread, it was crucial in my application
  // to get camera pose updates as frequently as possible

  while (WaitCycleStart()) {
    if ( isCamUpdated() ) {

      static ros::Time last_camera_update;
      float tf_delta = (ros::Time::now() - last_camera_update).toSec();
      last_camera_update = ros::Time::now();

      publishCameraPose();

      // for better visualization only
      if ( tf_delta < 0.75 ) {
        ROS_INFO("Updated camera pose published after %.3f",  tf_delta);
      } else if ( tf_delta < 1.50 ) {
        ROS_WARN("Updated camera pose published after %.3f",  tf_delta);
      } else {
        ROS_ERROR("Updated camera pose published after %.3f", tf_delta);
      }

      publishCamTrajectory();

      if ( ros::Time::now() >= (last_state_publish_time_ +
           ros::Duration(1. / orb_state_republish_rate_)) )
      {
         // it's time to re-publish info
         publishState(NULL);
         checkMode();
         publishUInt32Msg(kf_pub_, drawer_.GetKeyFramesNb());
         publishUInt32Msg(kp_pub_, drawer_.GetKeypointsNb());
         publishUInt32Msg(mp_pub_, drawer_.GetMatchedPointsNb());
         publishLoopState(GetLoopCloser()->isRunningGBA()); // GBA is quite time-consuming task so it will probably be detected here
         last_state_publish_time_ = ros::Time::now();
      }
    }
  }
  ROS_ERROR("InfoUpdater finished");
  SetFinish(true);


}
void ROSPublisher::Run()
{
    using namespace std::this_thread;
    using namespace std::chrono;

    ROS_INFO("ROS publisher started");

    if ( perform_scale_correction_ && GetSystem()->GetSensorType() == ORB_SLAM2::System::eSensor::MONOCULAR) {

      bool scale_correction = false; // flag to check state at the end
      ScaleCorrector scale_corrector(scaling_distance_);
      ROS_INFO("Waiting for initialization...");
      while ( GetSystem()->GetTrackingState() <= ORB_SLAM2::Tracking::NOT_INITIALIZED ) {

        if ( isStopRequested() ) {
          Stop();
          // GetSystem()->Shutdown();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }

      ROS_WARN("Starting the map scale correction procedure...");
      ROS_WARN("You should move the robot constantly in one direction");

      while ( !scale_corrector.isScaleUpdated() ) {
        scale_correction = true;
        // TODO: freezes when trying to shutdown here
        cv::Mat xf = PublisherUtils::computeCameraTransform(GetCameraPose());
        if ( !scale_corrector.gotCamPosition() ) {
          scale_corrector.setCameraStartingPoint(xf.at<float>(0, 3),
                                                 xf.at<float>(1, 3),
                                                 xf.at<float>(2, 3));
        }

        if ( scale_corrector.isReady() ) {
          scale_corrector.calculateScale(xf.at<float>(0, 3),
                                         xf.at<float>(1, 3),
                                         xf.at<float>(2, 3));
        }
        if ( isStopRequested() ) {
          Stop();
          scale_correction = false;
          // GetSystem()->Shutdown();
          ROS_WARN("Scale correction procedure must be stopped");
        }
        if ( GetSystem()->GetTrackingState() == ORB_SLAM2::Tracking::LOST ) {
          ROS_WARN("Scale correction procedure couldn't be fully performed - tracking lost. Try to re-initialize");
          scale_correction = false;
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

      }

      if ( scale_correction ) {
        map_scale_ = scale_corrector.getScale();
        ROS_INFO("Map scale corrected!");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // just to check scale
      }

    }

    /*
     * Moved here from constructor - there is a small map
     * created just before start of the correction procedure
     */
    if (octomap_enabled_) {
      octomap_worker_thread_ = std::thread( [this] { octomapWorker(); } );
    }
    info_updater_thread_ = std::thread( [this] { camInfoUpdater(); } );

    SetFinish(false);
    while (WaitCycleStart()) {

        // only publish map, map updates and camera pose, if camera pose was updated
        // TODO: maybe there is a way to check if the map was updated
        if (isCamUpdated()) {

            publishMap();
            // publishMapUpdates();
            if (octomap_enabled_)
            {
              // stashMapPoints(); // store current reference map points for the octomap worker
              stashMapPoints(false);
            }
        }
    }

    ROS_INFO("ROS publisher finished");
    SetFinish(true);
}

bool ROSPublisher::WaitCycleStart()
{
    if (!IPublisherThread::WaitCycleStart())
        return false;
    pub_rate_.sleep();
    return true;
}

void ROSPublisher::Update(Tracking *tracking)
{
    static std::mutex mutex;
    if (tracking == nullptr)
        return;

    publishState(tracking);

    // TODO: Make sure the camera TF is correctly aligned. See:
    // <http://docs.ros.org/jade/api/sensor_msgs/html/msg/Image.html>

    publishImage(tracking);
}

void ROSPublisher::clearCamTrajectoryCallback(const std_msgs::Bool::ConstPtr& msg) {
  if ( msg->data == true ) {
    clear_path_ = true;
  } else if ( msg->data == false ) {
    clear_path_ = false;
  }
}

void ROSPublisher::localizationModeCallback(const std_msgs::Bool::ConstPtr& msg) {

  if ( msg->data == true ) {
    localize_only = true;
  } else if ( msg->data == false ) {
    localize_only = false;
  }

}
