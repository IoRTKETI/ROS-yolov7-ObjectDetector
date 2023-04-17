# ROS-yolov7_ObjectDetector
This is a ROS package developed for **object detection in ROS topic images**. It's based on yolov7 detectors and you can also use custom models if you want. The available weights are **pt files** and **trt files**.

<div align="center">
    <a href="./">
        <img src="./yolov7/scripts/runs/detect/test.JPG" width="59%"/>
    </a>
</div>


## Preferences
we tested in
**ROS melodic** and
**ROS noetic** (recommand)

``` shell
# clone to the ros catkin workspace
git clone https://github.com/IoRTKETI/ROS-yolov7-ObjectDetector

# catkin make
cd ~/(your ros workspace PATH) && catkin_make

```

## run
``` shell
# use roslaunch
roslaunch yolov7 yolov7_object_detection.launch

or

# use rosrun
roscore

# another terminal
rosrun yolov7 yolov7_object_detection.launch
```

