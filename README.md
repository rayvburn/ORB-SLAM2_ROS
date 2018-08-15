# ORB-SLAM2_ROS

This package contains a ROS wrapper for ORB-SLAM2 (https://github.com/raulmur/ORB_SLAM2). 

ORB-SLAM2 provides a VSLAM (Visual Simultaneous Localization and Mapping) as a ROS node. Using ORB-SLAM2, you can create a 3-D feature-based sparse map, 2-D occupancy grid map (like a building floorplan) from camera data collected by a mobile robot.

Original package already added some features to the ORB-SLAM2 (https://gitlab.tubit.tu-berlin.de/breakdowncookie/ORB_SLAM2). It's authors are: Jan Brehmer, Christopher-Eyk Hrabia, Sebastiano Barrera. I modified it in matters of my MSc thesis.

# 1. Documentation

You can find a wide description of mechanisms used in ORB-SLAM2 in the following paper:

##### Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. ORB-SLAM: A Versatile and Accurate Monocular SLAM System. IEEE Transactions on Robotics, vol. 31, no. 5, pp. 1147-1163, 2015

The package above was tested on Ubuntu 16.04 with ROS Kinetic **only with monocular cameras**.

# 2. Hardware

TODO

# 3. Installation, example

The installation process is quite a mess now. Try to run the **build_catkin.sh** script after cleaning whole workspace and everything should be good.

    cd ${YOUR_CATKIN_WORKSPACE_LOCATION}
    catkin clean
    cd src/ORB-SLAM2
    ./build_catkin.sh
    
An example of use is as follows:

    roslaunch orb_slam2_ros ${YOUR_CAMERA_SPECIFIC_LAUNCH_FILE}

# 4. Nodes

TODO

## 4.1 Subscribed topics

TODO

## 4.2 Published topics

TODO

## 4.3 Parameters

TODO

## 4.4 Required tf Transforms

TODO

## 4.5 Provided tf Transforms

TODO

## 5. Additional info

TODO


