# metavision_ros_driver

A ROS driver for cameras using the metavison toolkit (Prophesee and
SilkyEVCam). It has performance improvements over the 
[official Prophesee
driver](https://github.com/prophesee-ai/prophesee_ros_wrapper) and
support ROS2.

This driver is not written or supported by Prophesee.

## Supported platforms

Currently only tested under ROS Noetic and ROS galactic on Ubuntu 20.04.

## How to build
Make sure you have your ROS1 or ROS2 environment sourced such that ROS_VERSION is set.
For example for ROS1 noetic:
```
source /opt/ros/noetic/setup.bash
```

Create a workspace (``metavision_ros_driver_ws``), clone this repo, and use ``wstool``
to pull in the remaining dependencies:

```
mkdir -p ~/metavision_ros_driver_ws/src
cd ~/metavision_ros_driver_ws
git clone https://github.com/berndpfrommer/metavision_ros_driver src/metavision_ros_driver
wstool init src src/metavision_ros_driver/metavision_ros${ROS_VERSION}_driver.rosinstall

# or to update an existing space
# wstool merge -t src src/metavision_ros_driver/metavision_ros${ROS_VERSION}_driver.rosinstall
# wstool update -t src
```

Now configure and build:

ROS1:
```
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
catkin build
. devel/setup.bash
```

ROS2:
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
. install/setup.bash
```

## Driver Features

This driver differs from the Prophesee ROS driver in the following ways:

- can publish:
  - ``prophesee_msgs``: same as the Prophesee ROS driver
  - ``dvs_msgs``: permits using ROS1 pipelines developed for the DVS
    camera
  - ``event_array2_msgs``: needed for ROS2 to get acceptable
    performance, loads faster when accessed through rosbag read API
- less CPU consumption by avoiding unnecessary memory copies.
- implemented as nodelet such that it can be run in the same address space as
  e.g. a rosbag record nodelet without worrying about message loss in transmission.
- prints out message rate statistics so you know when the sensor
  saturates bandwidth.
- supports dynamic reconfiguration
- NOTE: does not provide ``camera_info`` messages yet, does not play
  from raw files.

Parameters:

- ``bias_file``: path to file with camera biases. See example in the
  ``biases`` directory.
- ``message_time_threshold``: approximate time span [sec] of events to be
  aggregated before ROS message is sent. Defaults to 100us.
- ``statistics_print_interval``: time in seconds between statistics printouts.
- ``message_type``: can be set to ``dvs``, ``prophesee`` or ``event_array2``, depending on
  what message types the driver should publish. For ROS2 you must set
  it to ``event_array2`` to get acceptable performance, since the
  marshalling/unmarshalling overhead is too large for the other
  message types.
- ``send_queue_size``: ros message send queue size (defaults to 1000).
- ``use_multithreading``: decouples the SDK callback from the
  processing to ensure the SDK does not drop messages (defaults to
  false). The SDK already queues up messages but there is no documentation on
  the queue size and no way to determine if messages are dropped. Use multithreading to
  minimize the risk of dropping messages. However, be aware that this incurs an
  extra memory copy and threading overhead, raising the maximum CPU load by about 50% of a CPU.


Services:

- ``save_biases``: write out current bias settings to bias file. For
  this to work the ``bias_file`` parameter must be set to a non-empty value.


Dynamic reconfiguration parameters (see [MetaVision documentation here](https://docs.prophesee.ai/stable/hw/manuals/biases.html)):

- ``bias_diff`` (read only)
- ``bias_diff_off``
- ``bias_diff_on``
- ``bias_fo``
- ``bias_hpf``
- ``bias_pr``
- ``bias_refr``


# How to use (ROS1):

```
roslaunch metavision_ros_driver driver_node.launch   # (run as node)
roslaunch metavision_ros_driver driver_nodelet.launch   # (run as nodelet)
```

The driver should print out message rate statistics like this:
```
[ INFO] [1627733695.115154898]: rate[Mevs] avg:   0.007, max:   1.000, out sz:    3.06 ev, %on:  48 qs: 0
```
Prints out the average and maximum event rate (in million events per
second), the size (in number of events) of the outgoing ROS message, and the maximum
queue size (only non-zero if running in multithreaded mode) over the
``statistics_print_interval``. Note that for efficiency reasons the percentage of ON events,
is only computed if a subscriber is connected to the event topic.

To use the combined driver/recording facility:
```
roslaunch metavision_ros_driver recording_driver.launch bag:=`pwd`/test.bag
```
Then start/stop recording like this:
```
rosrun metavision_ros_driver start_recording.py
```
And stop recording:
```
rosrun metavision_ros_driver stop_recording.py
```

# How to use (ROS2):

```
ros2 launch metavision_ros_driver driver_node.launch.py        # (run as node)
ros2 launch metavision_ros_driver driver_composition.launch.py # (run as composable node)
```
The printout should be similar to the one for ROS1.

## CPU load

Here are some approximate performance numbers on a 16 thread (8-core) AMD
Ryzen 7480h with max clock speed of 2.9GHz. All numbers were obtained
by producing maximum event rates about (48Mevs) with a SilkyEVCam:

### ROS1 

All CPU loads below are with sensor saturating at close to 50Mevs.

| settings                        | EventArray | EventArray2 | EventArray2  | note                                 |
|                                 |            |             | (multithr)   |                                      |
|---------------------------------|------------|-------------|--------------|--------------------------------------|
| driver, no subscriber           | 49%        | 49%         | 106% (fluct) | no pub, extra copy for multithreaded |
| driver, publish messages        | 81%        | 58%         | 109%         | forced publishing, no subscriber     |
| driver(nodelet) + rostopic hz   | 135%       | 101%        | 151%         | does interprocess communication      |
| driver + rosbag record nodelet  | 190%       | 147%        | 191%         | no interproc. comm, but disk i/o     |
| driver + rosbag record separate | 131%+115%  | 100%+100%   | 155%+100%    | does interproc. comm + disk i/o      |
  

### ROS2

All CPU loads below are with sensor saturating at close to 50Mevs.

| settings                        | EventArray | EventArray2 | EventArray2  | note                                 |
|                                 |            |             | (multithr)   |                                      |
|---------------------------------|------------|-------------|--------------|--------------------------------------|
| driver, no subscriber           | 63%        | 63%         | 105% (fluct) | no pub, extra copy for multithreaded |
| driver, publish messages        | 63%        | 63%         | 110%         | forced publishing, no subscriber(\*) |
| driver(nodelet) + rostopic hz   | 102% (\**) | 71%         | 115%         | does interprocess communication      |
| driver + rosbag record nodelet  | n/a%       | n/a%        | n/a%         | no interproc. comm, but disk i/o     |
| driver + rosbag record separate | n/a%       | n/a%        | n/a%         | does interproc. comm + disk i/o      |


(\*) The forced publishing makes no difference because in either case
the data received from the SDK is parsed to get ON/OFF statistics (not
done for ROS1 driver). That memory read access apparently dominates
over the message creation. 
(\**) driver is dropping messages

## License

This software is issued under the Apache License Version 2.0.
