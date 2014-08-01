freenect2_camera
================

This is a prototype ROS driver for the new Kinect for Windows 2 (K4W2). It
uses the libfreenect2 library (see https://github.com/OpenKinect/libfreenect2).

Status
------

Publishes RGB and depth images at 15 and 30 Hz respectively. The depth images
are in m as a floating point value.

Building
--------

Just point CMake towards the libfreenect2 source:

```
mkdir build
cd build
cmake -DLIBFREENECT2_INCLUDE_DIR=/path/to/lf2/examples/protonect/include \
      -DLIBFREENECT2_LIBRARY=/path/to/lf2/examples/protonect/lib/libfreenect2.so \
      ..
```

Usage
-----

`rosrun nodelet nodelet standalone freenect2_camera`

Please don't hesitate to contact me with questions or remarks.

Max Schwarz <max.schwarz@online.de>
