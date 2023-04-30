# Blackfly S USB3 ROS Package [Unofficial]

## How to use?
Tested in ubuntu 20.04 with ros noetic.

Install [spinnaker sdk](https://www.flir.com/products/spinnaker-sdk/).

```ssh
cd ${your_catkin_ws}/src
git clone git@github.com:dhchung/blackfly_mono.git
catkin_make
```

Launch

```ssh
roslaunch blackfly_mono run.launch
```

## Parameters
Parameters are set in config/params.yaml
1. serial_num: The serial number of the camera (usually attach at the bottom of the camera, or you can check it out in spinview)
2. frame_rate: Up to 60Hz
3. show_image: if set true, opencv window will pop up
4. image_size_mode: 
   1. 2048/1080 - original image size
   2. 1024/540
   3. 512/270

Image size may differ if you change the decimation value.
Image size mode just makes the image size half and quarter for 2 and 3.