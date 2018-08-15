echo "Building ROS nodes"

cd ~/catkin_ws/src/ORB_SLAM2/orb_slam2_ros
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
