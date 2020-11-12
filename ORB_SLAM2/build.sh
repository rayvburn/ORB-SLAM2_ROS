echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2

# back to the initial directory i.e. {WS}/src/ORB_SLAM2_ROS/ORB_SLAM2 (not necessary)
cd ../..
