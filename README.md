# insta360x4_webcam

本项目通过ROS2 Humble, 读取 insta360 X4，并发布全景图到 /camera/image_raw
然后通过 [stella_vslam](https://stella-cv.readthedocs.io/en/latest/index.html) 的 [humble](https://stella-cv.readthedocs.io/en/latest/ros2_package.html#installation) 版本，即可订阅并进行定位

tested on ubuntu22 humble stella_vslam 0.6.0

## install
``` bash
mkdir -p ~/360_webcam/src
cd ~/360_webcam/src
git clone https://github.com/Longxiaoze/insta360x4_webcam.git
cd ..
colcon build
```

[stella_vslam](https://stella-cv.readthedocs.io/en/latest/ros2_package.html#installation)

## run

请连接insta360X4 360°全景相机并选择usb相机模式

``` bash
source ~/360_webcam/install/setup.bash
ros2 run insta360x4_webcam insta360x4_node
```

stella_vslam [running](https://stella-cv.readthedocs.io/en/latest/ros2_package.html#tracking-and-mapping)
``` bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/insta360x4_webcam
ros2 run stella_vslam_ros run_slam     -v ./configs/orb_vocab.fbow     -c ./configs/insta360X4_equirectangular.yaml  --map-db-out  map.msg     --ros-args -p publish_tf:=false
```

