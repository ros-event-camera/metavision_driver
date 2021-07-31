# metavision_ros_driver

A ROS driver for cameras using the metavison toolkit (Prophesee and
SilkyEVCam). This driver is not written or supported by Prophesee.
You can find their official ROS driver
[here](https://github.com/prophesee-ai/prophesee_ros_wrapper).


## Supported platforms

Currently only tested under ROS Noetic on Ubuntu 20.04.

## How to build
Create a workspace (``metavision_ros_driver_ws``), clone this repo, and use ``wstool``
to pull in the remaining dependencies:

```
mkdir -p ~/metavision_ros_driver_ws/src
cd ~/metavision_ros_driver_ws
git clone https://github.com/berndpfrommer/metavision_ros_driver src/metavision_ros_driver
wstool init src src/metavision_ros_driver/metavision_ros_driver.rosinstall
# to update an existing space:
# wstool merge -t src src/metavision_ros_driver/metavision_ros_driver.rosinstall
# wstool update -t src
```

Now configure and build:

```
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
catkin build
```

## Driver

Replacement driver for the Prophesee driver with the following improvements/changes:

- can write ``dvs_msgs`` or ``prophesee_msgs``. This permits
  using ROS based pipelines that have been developed for the DVS
  camera.
- less CPU consumption by avoiding unnecessary copies.
- implemented as nodelet so can be run in the same address space as
  e.g. a rosbag record nodelet without worrying of message loss in transmission.
- prints out message rate statistics.
- NOTE: does not provide ``camera_info`` messages yet

How to run:

```
roslaunch metavision_ros_driver driver_node.launch   # (run as node)
roslaunch metavision_ros_driver driver_nodelet.launch   # (run as nodelet)
```

Parameters:

- ``bias_file``: path to file with camera biases. See example in the
  ``biases`` directory.
- ``message_time_threshold``: approximate time span [sec] of events to be
  aggregated before ROS message is sent.
- ``statistics_print_interval``: time in seconds between statistics printouts.
- ``message_type``: can be set to ``dvs`` or ``prophesee`` depending on
  what message types the driver should publish.
- ``send_queue_size``: ros message send queue size (defaults to 1000).

Log messages:

```
[ INFO] [1627733695.115154898]: rate[Mevs] avg:   0.007, max:   1.000, out sz:    3.06 ev, %on:  48
```
Prints out the average and maximum event rate (in million events per
second) over the ``statistics_print_interval``. Note that for
efficiency reasons the last column, the percentage of ON events,
is only computed if a subscriber to the event topic is connected.

## License

This software is issued under the Apache License Version 2.0.
