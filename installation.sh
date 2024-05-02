cd ~/catkin_ws/src
git clone https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor
git clone https://github.com/tu-darmstadt-ros-pkg/hector_localization
git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo
git clone https://github.com/tu-darmstadt-ros-pkg/hector_models

sudo apt-get install -y ros-noetic-geographic-* \
ros-noetic-laser-geometry \
ros-noetic-unique-identifier

cd ..
catkin_make
