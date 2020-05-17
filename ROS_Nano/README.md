## Packages to install on ROS terminal

1. [joy](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)
    * Make sure gamepad controller (Logitech F710) is toggled to "DirectInput" mode

2. [ZED ros wrapper](https://github.com/stereolabs/zed-ros-wrapper)
    * To change **resolution** and **frame rate** when launching, go to\
    ~/*catkin_ws/src/zed-ros-wrapper/zed_wrapper/params/**common.yaml***

    * ZED camera resolution [0: HD2K, 1: HD1080, 2: HD720, 3: VGA] 


3. [rosserial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

4. darknet_ros
    * See */darknet_ros* folder

5. pid
    * See */pid* folder

6. learnjoy_python
    * See */learnjoy_python* folder


Make sure to run `catkin_make ` to build the packages and ensure python scripts are executable using `chmod a+x`

## To launch nodes

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch learnjoy_python joy_control_3.launch
```
To skip having to `source devel/setup.bash` every time a new terminal is opened, add it to .bashrc file

```
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

