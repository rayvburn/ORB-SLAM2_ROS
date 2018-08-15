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
echo -e "Installation will start in ${CYN}5${NC}"
sleep 1
echo -e "Installation will start in ${CYN}4${NC}"
sleep 1
echo -e "Installation will start in ${CYN}3${NC}"
sleep 1
echo -e "Installation will start in ${CYN}2${NC}"
sleep 1
echo -e "Installation will start in ${CYN}1${NC}"
sleep 1
echo " "

echo -e "Trying to install ${GRN}Eigen3${NC}. You need to write password in order to to that..."
sleep 1
sudo apt install libeigen3-dev

echo " "
echo "Unpacking the vocabulary file..."
sleep 1
cd ~/catkin_ws/src/ORB_SLAM2/orb_slam2_lib/Vocabulary/
tar xf ORBvoc.txt.tar.gz
echo "Done"
echo " "
sleep 1
cd ~/catkin_ws
catkin build orb_slam2_lib
cd ~/catkin_ws
source devel/setup.bash

echo " "
echo -e "${CYN}Trying to copy compiled lib to system directory...${NC}"
sleep 2
sudo cp ~/catkin_ws/devel/lib/liborb_slam2_lib.so /usr/local/lib

cd ~/catkin_ws/src/ORB_SLAM2
./build.sh
cd ~/catkin_ws/src/ORB_SLAM2
./build_ros.sh

sleep 1
cd ~/catkin_ws
source devel/setup.bash
catkin build
source devel/setup.bash
sleep 3
catkin build orb_slam2_ros orb_slam2_lib
source devel/setup.bash
sleep 2

echo " "
echo -e "${CYN}Converting vocabulary to binary...${NC}"
sleep 1
cd ~/catkin_ws/src/ORB_SLAM2/orb_slam2_lib
./bin_vocabulary
sleep 1

echo " "
echo " "
echo -e " ${GRN}--------------- HINTS ---------------${NC} "
echo " "
echo -e "If an error ${RED}fatal error: orb_slam2_ros/ORBState.h: No such file or directory${NC} still exists, please build catkin workspace manually by typing ${GRN}catkin build${NC} in main catkin_ws folder or run the script once again"
echo " "
echo -e "In case of an error saying ${RED}catkin_ws/devel/lib/orb_slam2_ros/Mono: symbol lookup error: ~/catkin_ws/devel/lib/orb_slam2_ros/Mono: undefined symbol: _ZN9ORB_SLAM211FrameDrawerC1EPNS_3MapE${NC} cleaning workspace (${GRN}catkin clean${NC}) and running this script once again will probably help. This most likely happened due to changes in files in ${GRN}orb_slam2_lib${NC} package."
echo " "
cd ~/catkin_ws


