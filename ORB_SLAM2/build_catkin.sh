RED='\033[0;31m'  # ${RED}
CYN='\033[0;36m'  # ${CYN}
GRN='\033[1;32m'  # ${GRN}
NC='\033[0m'      # ${NC}

echo " "
echo -e "${GRN}Building ORB_SLAM2 - catkin build${NC}"
echo " "
echo -e "Let this script be the ${CYN}first execution after cleaning workspace${NC} by ${GRN}'catkin clean'${NC} command"
echo -e "${RED}Be careful!${NC} It will rebuild all packages from workspace"
echo " "

echo -e "Trying to install ${GRN}Eigen3${NC} and ${GRN}octomap${NC} packages. You need to write password in order to do that..."
sleep 1
sudo apt install libeigen3-dev
sudo apt install ros-$ROS_DISTRO-octomap

echo "Downloading the vocabulary file..."
wget -O orb_slam2_lib/Vocabulary/ORBvoc.txt.tar.gz https://github.com/raulmur/ORB_SLAM2/raw/master/Vocabulary/ORBvoc.txt.tar.gz
echo " "
echo "Unpacking the vocabulary file..."
sleep 1
cd orb_slam2_lib/Vocabulary/
tar xf ORBvoc.txt.tar.gz
echo "Done"
echo " "
sleep 1
cd ../../../../.. # back to the workspace main folder

catkin build -j2 orb_slam2_lib
source devel/setup.bash

echo " "
echo -e "${CYN}Trying to copy compiled lib to system directory...${NC}"
sleep 2
cd devel/lib
sudo cp liborb_slam2_lib.so /usr/local/lib
cd ../.. # back to the workspace main folder

cd src/ORB-SLAM2_ROS/ORB_SLAM2
./build.sh

sleep 1
cd ../../.. # back to the workspace main folder
source devel/setup.bash
catkin build -j2
source devel/setup.bash
sleep 3
catkin build -j2 orb_slam2_ros orb_slam2_lib
source devel/setup.bash
sleep 2

echo " "
echo -e "${CYN}Converting vocabulary to binary...${NC}"
sleep 1
cd src/ORB-SLAM2_ROS/ORB_SLAM2/orb_slam2_lib
./bin_vocabulary
sleep 1

echo " "
echo " "
echo -e " ${GRN}--------------- HINTS ---------------${NC} "
echo " "
echo -e "If an error ${RED}fatal error: orb_slam2_ros/ORBState.h: No such file or directory${NC} exists, please build catkin workspace manually by typing ${GRN}catkin build${NC} in main catkin_ws folder or run the script once again"
echo " "
echo -e "In case of an error saying, like, ${RED}catkin_ws/devel/lib/orb_slam2_ros/Mono: symbol lookup error: ~/catkin_ws/devel/lib/orb_slam2_ros/Mono: undefined symbol: _ZN9ORB_SLAM211FrameDrawerC1EPNS_3MapE${NC} cleaning workspace (${GRN}catkin clean${NC}) and running this script once again will probably help. This most likely happened due to changes in files in ${GRN}orb_slam2_lib${NC} package."
echo " "
echo -e "If the build process ended without any errors, you're ready to go!"
