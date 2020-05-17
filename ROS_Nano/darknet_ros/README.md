# How to set up darknet_ros package

1.  [Install darknet on ROS](https://github.com/leggedrobotics/darknet_ros)

2.  Use your own detection objects (List below shows where each of the files should go)

    * yolov3-tiny-obj_final.weights\
    *catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights*

    * yolov3-tiny-obj.cfg\
    *catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/cfg*

    * yolov3_cheeyung.yaml\
    *catkin_ws/src/darknet_ros/darknet_ros/config*

    * darknet_ros.launch\
    *catkin_ws/src/darknet_ros/darknet_ros/launch*

3.  /corridor_State node

    * findcorridorbox_v4.py\
    *catkin_ws/src/darknet_ros/darknet_ros*
    
