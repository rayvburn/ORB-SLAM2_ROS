/*
 * PublisherUtils.cc
 *
 *  Created on: Jun 2, 2018
 *      Author: jarek
 */

#include <include/PublisherUtils.h>

sensor_msgs::PointCloud2 PublisherUtils::convertToPCL2( const std::vector<ORB_SLAM2::MapPoint*> &map_points,
                                                        const double &map_scale,
                                                        const float &camera_height)
{

    const std::size_t n_map_points = map_points.size();

    // Kind of a hack, but there aren't much better ways to avoid a copy
    struct point { float x, y, z; };

    std::vector<uint8_t> data_buffer(n_map_points * sizeof(point));
    std::size_t vtop = 0;

    point *dataptr = (point*) data_buffer.data();

    ros::Time tStart = ros::Time::now();
    for (ORB_SLAM2::MapPoint *map_point : map_points) {
        if (map_point->isBad())
            continue;
        cv::Mat pos = map_point->GetWorldPos();
        PublisherUtils::transformPoint(pos, map_scale, false, 0, camera_height);
        dataptr[vtop++] = {
            pos.at<float>(0),
            pos.at<float>(1),
            pos.at<float>(2),
        };

    }
    ros::Duration tStop = (ros::Time::now() - tStart);
    ROS_INFO("sending PointCloud (%lu points), conversion took %.3f sec", n_map_points, tStop.toSec());

    static const char* const names[3] = { "x", "y", "z" };
    static const std::size_t offsets[3] = { offsetof(point, x), offsetof(point, y), offsetof(point, z) };
    std::vector<sensor_msgs::PointField> fields(3);
    for (int i=0; i < 3; i++) {
        fields[i].name = names[i];
        fields[i].offset = offsets[i];
        fields[i].datatype = sensor_msgs::PointField::FLOAT32;
        fields[i].count = 1;
    }

    sensor_msgs::PointCloud2 msg;
    msg.height = 1;
    msg.width = n_map_points;
    msg.fields = fields;
    msg.is_bigendian = PublisherUtils::IS_BIG_ENDIAN;
    msg.point_step = sizeof(point);
    msg.row_step = sizeof(point) * msg.width;
    msg.data = std::move(data_buffer);
    msg.is_dense = true;  // invalid points already filtered out
    return msg;

}

void PublisherUtils::transformPoint( cv::Mat& pos_, const double &scale_, const bool no_rotation_,
                                     const unsigned char frame_flag_,
                                     const float &camera_height_)

   // flag = 0 for PCL, flag = 1 for octomap and camera_optical
{

  /*
   *
   * Procedure explained:
   * cv::Mat T = cv::Mat::zeros(4,4, CV_64F);
   * cv::Mat R = cv::Mat(4,4, CV_64F);
   *
   * 1. Compute of appropriate rotation matrix, to transform frame
   * (which flag points to) into /map frame
   *
   * due to limited computing power there are just simple transformations
   * after knowing what changes there need to be applied to original point
   *
   * 2. Insert translation vector into
   * T = R;
   * for ( int i = 0; i < 4; i++) {
   *    T.at<float>(i,3) = translation.at<float>(i);
   * }
   * cv::Mat pos_hom = (cv::Mat_<float>(4,1) <<  pos_.at<float>(0),
   *                                             pos_.at<float>(1),
   *                                             pos_.at<float>(2),
   *                                             1);
   * pos_hom.rowRange(0,3) *= scale_;
   * pos_hom = T * pos_hom;
   * pos_ = pos_hom.rowRange(0,3);
   *
   */

  if ( !no_rotation_ ) {

    // "z" is empirically adjusted to put the PCL "ground level" just below map.z = 0
    // static const float CAMERA_HEIGHT = 0.5 * camera_height_;
    // already done in ROSPublisher class

    // to store the original vector
    cv::Mat bckp;
    pos_.copyTo(bckp);

    // pointers to vectors
    float* ptr_orig = (float*)pos_.data;
    float* ptr_bckp = (float*)bckp.data;

    // step between 2 float values in memory
    const size_t step = pos_.step / sizeof(float);

    if ( frame_flag_ == 0 ) {
      /*
       * Rotation for PCL -> /map transform is
       * R = PublisherUtils::R_270Z * PublisherUtils::R_270X;
       * below is equal
       */
      ptr_orig[0] = ptr_bckp[2*step] * scale_;
      ptr_orig[1] = ptr_bckp[0] * scale_ * (-1);
      ptr_orig[2] =(ptr_bckp[step] * scale_ * (-1)) + camera_height_;

    } else if ( frame_flag_ == 1 ) {
      /*
       * Rotation for octomap/camera_optical -> /map transform is
       * R = PublisherUtils::R_90X * PublisherUtils::R_90Z;
       * below is equal
       * NOTE! valid only if octomap aligned with map_frame_
       */
      ptr_orig[0] = ptr_bckp[1*step] * scale_ * (-1);
      ptr_orig[1] = ptr_bckp[2*step] * scale_ * (-1);
      ptr_orig[2] =(ptr_bckp[0] * scale_) + camera_height_;

    }  else { return; }

  } else {
    // only scale to adjust
    pos_ *= scale_;
  }

}

cv::Mat PublisherUtils::computeCameraTransform(const cv::Mat& Twc, const double scale)
{

  cv::Mat ret = cv::Mat::eye(4, 4, CV_32F);

  if(!Twc.empty()) {

    auto Rwc = Twc.rowRange(0,3).colRange(0,3).t();
    ret.rowRange(0,3).colRange(0,3) = Rwc;

    // twc, the position - rotation + translation
    ret.rowRange(0,3).col(3) = -Rwc * Twc.rowRange(0, 3).col(3);

    // scale correction
    for ( int i = 0; i < 3; i++) {
      ret.at<float>(i,3) *= scale;
    }
  }
  return ret;
}

geometry_msgs::PoseStamped PublisherUtils::getPoseStamped(const tf::Transform* tf_,
                                                          const std::string* frame_id_) {

    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = *frame_id_;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = tf_->getOrigin().x();
    pose.pose.position.y = tf_->getOrigin().y();
    pose.pose.position.z = tf_->getOrigin().z();

    pose.pose.orientation.x = tf_->getRotation().x();
    pose.pose.orientation.y = tf_->getRotation().y();
    pose.pose.orientation.z = tf_->getRotation().z();
    pose.pose.orientation.w = tf_->getRotation().w();

    return pose;

}

bool PublisherUtils::isBigEndian()
{
    volatile int num = 1;
    return *((char*) &num) == ((char) 1);
}

/*
 * Replaces NaN values with the mean of their 8 neighbors whenever possible.
 * Does it n times. Returns number of eroded NaNs.
 */
int PublisherUtils::erodeNaN(cv::Mat &matrix, int n)
{
    cv::Mat matrix_old = matrix.clone(); // use original matrix when looking up neighbors

    int nb_eroded_cells = 0; // rather for information/debug purposes
    for (int i = 0; i < n; ++i) // erode n times
    {
        for (int x = 0; x < matrix.cols; ++x) // iterate over matrix
        {
            for (int y = 0; y < matrix.rows; ++y)
            {
                float current_value = matrix_old.at<float>(y, x);
                if (current_value != current_value) // is NaN
                {
                    int nb_values = 0;
                    float sum = 0;
                    for (int dx = -1; dx < 2; ++dx) // iterate over neighborhood
                    {
                        for (int dy = -1; dy < 2; ++dy)
                        {
                            if ((x + dx >= 0) && (y + dy >= 0) && (x + dx < matrix.cols) && (y + dy < matrix.rows)) // within matrix bounds
                            {
                                current_value = matrix_old.at<float>(y + dy, x + dx);
                                if (current_value == current_value) // is not NaN
                                {
                                    sum += current_value;
                                    nb_values++;
                                }
                            }
                        }
                    }
                    if (nb_values > 0) // there were non-NaN neighbors
                    {
                        matrix.at<float>(y, x) = sum / nb_values;
                        nb_eroded_cells++;
                    }
                }
            }
        }
    }
    return nb_eroded_cells;
}


/*
 * Writes a fully contrast-scaled version of a 2-dimensional 1-channel matrix into a grayscale image file.
 */
void PublisherUtils::grayscaleToFile(const string& filename, const cv::Mat& img)
{
    double min_value = 0, max_value = 0;
    cv::minMaxLoc(img, &min_value, &max_value, 0, 0, img == img); // img==img masks out NaNs

    cv::Mat out;
    double scale = 255. / (max_value - min_value);
    double shift = scale * -min_value;
    img.convertTo(out, CV_8U, scale, shift);
    out.setTo(128, img != img); // set NaN to gray
    cv::imwrite(filename, out);
}


const char* PublisherUtils::stateDescription(orb_slam2_ros::ORBState orb_state)
{
    switch (orb_state.state) {
        case orb_slam2_ros::ORBState::SYSTEM_NOT_READY: return "System not ready";
        case orb_slam2_ros::ORBState::NO_IMAGES_YET: return "No images yet";
        case orb_slam2_ros::ORBState::NOT_INITIALIZED: return "Not initialized";
        case orb_slam2_ros::ORBState::OK: return "OK";
        case orb_slam2_ros::ORBState::LOST: return "Tracking lost";
    }

    return "???";
}



const orb_slam2_ros::ORBState PublisherUtils::toORBStateMessage(ORB_SLAM2::Tracking::eTrackingState trackingState)
{
    orb_slam2_ros::ORBState state_msg;
    state_msg.state = orb_slam2_ros::ORBState::UNKNOWN;

    switch (trackingState) {
        case ORB_SLAM2::Tracking::SYSTEM_NOT_READY: state_msg.state = orb_slam2_ros::ORBState::SYSTEM_NOT_READY;
                                         break;
        case ORB_SLAM2::Tracking::NO_IMAGES_YET:    state_msg.state = orb_slam2_ros::ORBState::NO_IMAGES_YET;
                                         break;
        case ORB_SLAM2::Tracking::NOT_INITIALIZED:  state_msg.state = orb_slam2_ros::ORBState::NOT_INITIALIZED;
                                         break;
        case ORB_SLAM2::Tracking::OK:               state_msg.state = orb_slam2_ros::ORBState::OK;
                                         break;
        case ORB_SLAM2::Tracking::LOST:             state_msg.state = orb_slam2_ros::ORBState::LOST;
                                         break;
    }

    return state_msg;
}

tf::Quaternion PublisherUtils::quaternionFromRPY(float roll_, float pitch_, float yaw_)
{

  tf::Quaternion quat;
  tfScalar yaw =     yaw_ * PublisherUtils::DEG_RAD_MULT;
  tfScalar pitch = pitch_ * PublisherUtils::DEG_RAD_MULT;
  tfScalar roll =   roll_ * PublisherUtils::DEG_RAD_MULT;
  quat.setEulerZYX(yaw, pitch, roll);
  return quat;

}

tf::Transform PublisherUtils::createTF(tf::Vector3 vect_, tf::Quaternion quat_)
{

  tf::Transform tf;
  tf.setOrigin(vect_);
  tf.setRotation(quat_);
  return tf;

}

