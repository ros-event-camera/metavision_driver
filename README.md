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
wstool init src src/metavision_ros_driver/metavision_ros_driver.rosinstall

# or to update an existing space
# wstool merge -t src src/metavision_ros_driver/metavision_ros_driver.rosinstall
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
  - ``event_array_msgs``: needed for ROS2 to get acceptable
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
- ``message_type``: can be set to ``dvs``, ``prophesee`` or ``event_array``, depending on
  what message types the driver should publish. For ROS2 you must set
  it to ``event_array`` to get acceptable performance, since the
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

For efficient recording of the events you need to run the
driver and the recorder in the same address space as ROS2 composable
nodes. For this you will need to install the
[composable recorder](https://github.com/berndpfrommer/rosbag2_composable_recorder)
into your workspace as well.

```
ros2 launch metavision_ros_driver driver_node.launch.py        # (run as node)
ros2 launch metavision_ros_driver driver_composition.launch.py # (run as composable node)
```
The printout should be similar to the one for ROS1.

To use the combined driver/recorder and start the recording:
```
ros2 launch recording_driver.launch.py
ros2 run metavision_ros_driver start_recording_ros2.py
```
To stop the recording you have to kill (Ctrl-C) the recording driver.

## CPU load

Here are some approximate performance numbers on a 16 thread (8-core) AMD
Ryzen 7480h with max clock speed of 2.9GHz. All numbers were obtained
by producing maximum event rates about (48Mevs) with a SilkyEVCam:

### ROS1 

All CPU loads below are with sensor saturating at close to 50Mevs.

| settings                        | DvsMsg    | EventArray | EventArray (multithr) | note                                 |
|---------------------------------|-----------|------------|-----------------------|--------------------------------------|
| driver, no subscriber           | 49%       | 49%        | 106% (fluct)          | no pub, extra copy for multithreaded |
| driver, publish messages        | 81%       | 58%        | 109%                  | forced publishing, no subscriber     |
| driver(nodelet) + rostopic hz   | 135%      | 101%       | 151%                  | does interprocess communication      |
| driver + rosbag record nodelet  | 190%      | 147%       | 191%                  | no interproc. comm, but disk i/o     |
| driver + rosbag record separate | 131%+115% | 100%+100%  | 155%+100%             | does interproc. comm + disk i/o      |
  

### ROS2

All CPU loads below are with sensor saturating at close to 50Mevs.
Middleware used was cyclonedds.

| settings                        | DvsMsg        | EventArray | EventArray (multithr) | note                                 |
|---------------------------------|---------------|------------|-----------------------|--------------------------------------|
| driver, no subscriber           | 63%           | 63%        | 105% (fluct)          | no pub, extra copy for multithreaded |
| driver, publish messages        | 63%           | 63%        | 110%                  | forced publishing, no subscriber(1)  |
| driver + rostopic hz            | 102% (2)      | 71%        | 115%                  | does interprocess communication      |
| driver + rosbag record composed | 117% (3)      | 120%       | 175%                  | no interproc. comm, but disk i/o     |
| driver + rosbag record separate | 112% + 5% (3) | 92% + 82%  | 122% + 94%            | does interproc. comm + disk i/o      |


(1) The forced publishing makes no difference because in either case
the data received from the SDK is parsed to get ON/OFF statistics (not
done for ROS1 driver). That memory read access apparently dominates
over the message creation. 
(2) driver is dropping messages
(3) driver spends virtually all time in publish(), does not produce messages at full rate

### About ROS time stamps

The SDK provides hardware event time stamps directly from the
camera. However first there is a unknown offset between the clocks,
and second there is clock drift because the camera's clock is not synchronized with the ROS
host's. For this reason the ROS driver implements an ugly scheme for
time stamping the ROS messages as follows:


    - within one ROS message, the time stamp differences between events are
      identical to the SDK provided hardware time stamps

    - *inbetween* ROS messages however, the event time gaps can differ
	  from the hardware time stamps. This is necessary to catch up with
	  the sensor clock, or to wait for the sensor clock.

    - the event time stamps are never allowed to have negative
      differences, i.e. the last event in a ROS message will have a time
      stamp that is no later than the first one in the following message.

    - the header stamps are managed such that there is no long-term
      drift vs the host clock, i.e. the ROS time stamps of events
      should align with those of other sensor data collected on the
      same host.

	- this occasionally leads to ROS time stamps being in the future,
      i.e. a message may have a header stamp slightly ahead of it's
      recording time stamp. The driver will try to remedy such a
      situation the moment the incoming sensor data will permit.

The following graph shows the clock skew between a SilkyEVCam and the
host clock (green line). It also shows the difference between rosbag
recording time stamp and ROS message header stamp. Note that a) there
is no long-term drift between header stamps and recording stamps and
b) there is very little variance between header stamps and sensor time
stamps (aside from the drift).

![timestamp image](images/time_offset_example.png)

Note that using the ``time_base`` field of the ``EventArray`` message
permits recovery of the original sensor timestamps.

## License

This software is issued under the Apache License Version 2.0.
