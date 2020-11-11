#echo "Configuring and building Thirdparty/DBoW2 ..."

#cd orb_slam2_lib/Thirdparty/DBoW2
#mkdir build
#cd build
#cmake .. -DCMAKE_BUILD_TYPE=Release
#make -j2

#cd ..
#cd ../../g2o

#echo "Configuring and building Thirdparty/g2o ..."

#mkdir build
#cd build
#cmake .. -DCMAKE_BUILD_TYPE=Release
#make -j2

#cd ../../..

#echo "Uncompress vocabulary ..."

#cd Vocabulary
#tar -xf ORBvoc.txt.tar.gz
#cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2

# back to the initial directory i.e. {WS}/src/ORB_SLAM2_ROS/ORB_SLAM2 (not necessary)
cd ../..
