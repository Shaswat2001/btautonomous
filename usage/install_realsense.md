### Installing Intel Realsense Library for Gazebo and Moveit

Below are mentioned the commands used to setup the realsense camera for pick and place operations with the help of articulated manipulators. 

* Preparation

```
$ export REALSENSE_SOURCE_DIR=$HOME/librealsense/

$ sudo apt-get install guvcview git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev

$ sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

$ git clone https://github.com/IntelRealSense/librealsense.git $REALSENSE_SOURCE_DIR

$ mkdir $REALSENSE_SOURCE_DIR/build

$ cd $REALSENSE_SOURCE_DIR/build
```

* Build (this process will take some time)

```
$ export REALSENSE_INSTALL_PREFIX=/opt/realsense

$ sudo mkdir -p $REALSENSE_INSTALL_PREFIX;

$ sudo chown $USER:$USER -R $REALSENSE_INSTALL_PREFIX

$ cmake ../ -DFORCE_RSUSB_BACKEND=true -DBUILD_PYTHON_BINDINGS=false -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DCMAKE_INSTALL_PREFIX=$REALSENSE_INSTALL_PREFIX

$ make install

$ sudo sh -c "echo $REALSENSE_INSTALL_PREFIX/lib > /etc/ld.so.conf.d/realsense.conf"

$ sudo ldconfig

$ cd ~/realsense

$ sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && udevadm trigger

$ echo "export realsense2_DIR=/opt/realsense/lib/cmake/realsense2" >> ~/.bashrc
```

* Integrating ROS with Realsense

```
$ export REALSENSE_ROS_WS=$HOME/ros_intel/src

$ sudo mkdir -p $REALSENSE_ROS_WS;

$ git clone https://github.com/pal-robotics/ddynamic_reconfigure.git

$ git clone git clone https://github.com/IntelRealSense/realsense-ros.git

$ cd realsense-ros/

$ git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`

$ cd ..

$ catkin_init_workspace

$ cd ..

$ catkin_make clean

$ catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release

$ catkin_make install

$ echo "source ~/ros_intel/devel/setup.bash" >> ~/.bashrc

$ source ~/.bashrc

```