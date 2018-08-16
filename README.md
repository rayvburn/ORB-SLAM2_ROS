# ORB-SLAM2_ROS

This package contains a ROS wrapper for ORB-SLAM2 (https://github.com/raulmur/ORB_SLAM2). 

ORB-SLAM2 provides a VSLAM (Visual Simultaneous Localization and Mapping) as a ROS node. Using ORB-SLAM2, you can create a 3-D feature-based sparse map, 2-D occupancy grid map (like a building floorplan) from camera data collected by a mobile robot.

Original package already added some features to the ORB-SLAM2 (https://gitlab.tubit.tu-berlin.de/breakdowncookie/ORB_SLAM2). It's authors are: Jan Brehmer, Christopher-Eyk Hrabia, Sebastiano Barrera. I modified it in matters of my MSc thesis.

Binary version of vocabulary file was proposed by poine (https://github.com/poine/ORB_SLAM2).

# 1. Documentation

You can find a wide description of mechanisms used in ORB-SLAM2 in the following paper:

##### Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. ORB-SLAM: A Versatile and Accurate Monocular SLAM System. IEEE Transactions on Robotics, vol. 31, no. 5, pp. 1147-1163, 2015

The package above was tested on Ubuntu 16.04 with ROS Kinetic **only with monocular cameras**.

# 2. Hardware

TODO

# 3. Installation, example

The installation process is quite a mess now. Look at the note in the **orb_slam2_lib/Vocabulary** folder. Then try to run the **build_catkin.sh** script after cleaning whole workspace and everything should be good.

    cd ${YOUR_CATKIN_WORKSPACE_LOCATION}
    catkin clean
    cd src/ORB-SLAM2
    ./build_catkin.sh
    
An example of use is as follows:

    roslaunch orb_slam2_ros ${YOUR_CAMERA_SPECIFIC_LAUNCH_FILE}

# 4. Nodes

## 4.1 orb_slam2_ros

The orb_slam2_ros node takes in sensor_msgs/Image messages and builds:
- a point cloud (sensor_msgs/PointCloud2),
- an octomap (octomap_msgs/Octomap),
- a map (nav_msgs/OccupancyGrid).
The map can be retrieved via a ROS topic. 

## 4.1.1 Subscribed topics

TODO

## 4.1.2 Published topics

**cam_path (nav_msgs/Path)** - stores the camera path since initialization (or scale correction finish)


**clear_cam_path (std_msgs/Bool)** - publishing true to this topic clears the camera path buffer


**frame (sensor_msgs/Image)** - the current frame with keypoints marked


**info/frame_keypoints (std_msgs/UInt32)** - a number of keypoints in the current frame


**info/loop_closed (std_msgs/Bool)** - publishes true if GBA is running


**info/map_keyframes (std_msgs/UInt32)** - number of total keyframes in the map


**info/matched_points (std_msgs/UInt32)** - number of matched map points in the current frame


**info/state (orb_slam2_ros/ORBState)** - state of the node


**info/state_description (std_msgs/String)** - text description of the state


**map (sensor_msgs/PointCloud2)** - the point cloud storing all the map points


**map_updates (sensor_msgs/PointCloud2)** - the point cloud storing reference map points


**octomap (octomap_msgs/Octomap)** - the octomap created from the point cloud


**projected_map (nav_msgs/OccupancyGrid)** - a floorplan created from the octomap as an intersection between 2 heights


**projected_morpho_map (nav_msgs/OccupancyGrid)** - the projected map with morphological operations performed on


**switch_mode (std_msgs/Bool)** - publishing true switches to localization mode, false switches back to SLAM mode; SLAM is default value

## 4.1.3 Parameters

**ORB-SLAM2/orb_slam2_ros/settings/orb_slam2_param.yaml**


~**topic/freq (float, default: )** - frequency of SLAM system job

~**topic/image_topic (string, default: )** - image topic name

~**topic/orb_state_republish_rate (float, default: )** - re-publish state @ Hz



~**map_scale/perform_correction (bool, default: true)** - possible to do with wheel encoders and robot description

~**map_scale/odom_topic (string, default: "/odom")** - topic that odometry data are published on (valid if correction set true)

~**map_scale/scaling_distance (float, default: 1.0)** - distance to move according to odom topic, to perform scale estimation 

~**map_scale/set_manually (float, default: 1.5)** - manually set map scale - how visually sparse the map is; no effect if 

~**map_scale/camera_height (float, default: 0.205)** - camera height above base_link frame (it is automatically checked in tf_tree if scale correction is performed)

~**map_scale/camera_height_multiplier (float, default: 1.0)** - just for better visualization (if the PCL is too low) - scale is not always perfect



.. TODO ..

## 4.1.4 Required tf Transforms

interface_frame - the tf from odometry frame to camera_optical frame (check REP103 for a camera_optical frame orientation - http://www.ros.org/reps/rep-0103.html, example is in my package diff_drive_mapping_robot/tf_broadcaster)

## 4.1.5 Provided tf Transforms

map -> interface_frame (map_odom_interface_frame)

## 5. Additional info

The package need exhaustive testing with other camera types. It also doesn't have the save/load map feature. Please notice that octomap parameters have huge impact on performance and morphological operations (erosion, opening, closing) have big influence how the occupancy grid map looks like. Another thing is that height of the camera often drift after some time and it is hard to set the occupancy/projected_map heights to perfectly map the real environment.

What's novel in this package is the map's scale correction procedure (for monocular cameras) that uses simple odometry sensors to calculate the multiplier between real and map distance.


