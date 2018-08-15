/*
 * PublisherUtils_impl.h
 *
 *  Created on: Jun 2, 2018
 *      Author: jarek
 */

#ifndef SRC_PUBLISHERUTILS_IMPL_H_
#define SRC_PUBLISHERUTILS_IMPL_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

namespace PublisherUtils {

template<typename Q>
  Q convertToQuaternion(const cv::Mat& rot)
  {

      double trace = rot.at<float>(0,0) + rot.at<float>(1,1) + rot.at<float>(2,2);
      double tmp[4];

      if (trace > 0.0) {
          double s = sqrt(trace + 1.0);
          tmp[3] = s * 0.5;
          s = 0.5 / s;
          tmp[0] = ((rot.at<float>(2,1) - rot.at<float>(1,2)) * s);
          tmp[1] = ((rot.at<float>(0,2) - rot.at<float>(2,0)) * s);
          tmp[2] = ((rot.at<float>(1,0) - rot.at<float>(0,1)) * s);
      } else {
          int i;
          if (rot.at<float>(0, 0) < rot.at<float>(1,1))
              i = rot.at<float>(1,1) < rot.at<float>(2,2) ? 2 : 1;
          else
              i = rot.at<float>(0,0) < rot.at<float>(2,2) ? 2 : 0;
          int j = (i + 1) % 3;
          int k = (i + 2) % 3;

          double s = sqrt(rot.at<float>(i,i) - rot.at<float>(j,j) - rot.at<float>(k,k) + 1.0);
          tmp[i] = s * 0.5;
          s = 0.5 / s;
          tmp[3] = (rot.at<float>(k,j) - rot.at<float>(j,k)) * s;
          tmp[j] = (rot.at<float>(j,i) + rot.at<float>(i,j)) * s;
          tmp[k] = (rot.at<float>(k,i) + rot.at<float>(i,k)) * s;
      }

      return {tmp[0], tmp[1], tmp[2], tmp[3]};
  }

/*
 * Returns a ROS parameter as generic type, defaulting to a given value if it is unspecified.
 */
template<typename T>
  T getROSParam(ros::NodeHandle nh, std::string param_name, T default_value)
  {
      T result;
      nh.param<T>(param_name, result, default_value);
      return result;
  }

}

#endif /* SRC_PUBLISHERUTILS_IMPL_H_ */
