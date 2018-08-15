/*
 * ROSSystemBuilder.h
 *
 *  Created on: Jun 3, 2018
 *      Author: jarek
 */

#ifndef SRC_ROSSYSTEMBUILDER_H_
#define SRC_ROSSYSTEMBUILDER_H_

#include "ROSPublisher.h"
#include "utils.h"

class ROSSystemBuilder : public ORB_SLAM2::System::GenericBuilder {
public:
    ROSSystemBuilder(const std::string& strVocFile,
                     const std::string& strSettingsFile,
                     ORB_SLAM2::System::eSensor sensor,
                     double frequency,
                     ros::NodeHandle nh = ros::NodeHandle(),
                     std::string map_frame = ROSPublisher::DEFAULT_MAP_FRAME,
                     std::string camera_frame = ROSPublisher::DEFAULT_CAMERA_FRAME);

    virtual ORB_SLAM2::IPublisherThread* GetPublisher() override;
    virtual ~ROSSystemBuilder();

private:

    std::unique_ptr<ROSPublisher> mpPublisher;
};


#endif /* SRC_ROSSYSTEMBUILDER_H_ */
