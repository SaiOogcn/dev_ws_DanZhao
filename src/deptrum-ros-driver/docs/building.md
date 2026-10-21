# Building the ROS Driver

## Pre-requisites

Before trying to build the ROS Driver, you will need to install required dependencies:
- ROS foxy  (Ubuntu20.04)

## Source Compiling

for aurora930,colcon build --cmake-args -DSTREAM_SDK_TYPE=AURORA930
for stellar400,colcon build --cmake-args -DSTREAM_SDK_TYPE=STELLAR400
Please note that you may need to run colcon build --cmake-force-configure to update the SDK binaries which are copied into the ROS2 output folders.

