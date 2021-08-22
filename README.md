# human_tracking_3D
human detection and tracking in 3D space based on Yolov3 and deepsort

![image](https://github.com/EcustBoy/human_tracking_3D/main/docs/bbox.gif )
![image](https://github.com/EcustBoy/human_tracking_3D/main/docs/pointcloud_bbox.gif )

## requirements
* ros-kinetic
* usb camera or intel realsense d435i camera
* python2.7

## installation
* download darknet_ros package
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
cd ../
catkin build darknet_ros
```
* download deepsort-yolo package and rviz visualization package
```
cd ~
git clone git@github.com:EcustBoy/human_tracking_3D.git
cp -r ./human_tracking_3D/* ~/catkin_ws/src
cd ~/catkin_ws/src
catkin build sort_track
cd ../
catkin_make
```

## run
* launch camera device

#### for usb camera
```
roslaunch usb_cam usb_cam-test.launch
```
#### for d435i camera
```
roslaunch realsense2_camera rs_aligned_depth.launch
```
* launch darknet_ros
```
roslaunch darknet_ros yolo_v3-tiny.launch
```
make sure the camera publish topic name is same as darknet_ros subscribe topic name

* launch deepsort node
```
roslaunch sort_track sort_deep.launch
```
* calculate human position in 3D space (core)
```
cd ~/catkin_ws/src/sort-deepsort-yolov3-ROS/sort_track/src
pyhton2 ./my_track_deep.py
```
launch node which subscribes yolo detection results and calculates human 3D position coordinate based on camera equation
publish (x,y,depth,id) for each human

* visualize in rviz
```
rosrun visualize_bbox visualize_bbox
``` 
you can see the real-time human detection results(bounding box) in rviz
