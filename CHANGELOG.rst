^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package metavision_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.7 (2023-08-08)
------------------
* Prophesee standard plugins should now be coming with the ROS driver
* Contributors: Bernd Pfrommer

1.1.6 (2023-08-07)
------------------
* see if add_dependencies() is causing trouble
* resurrected old ROS_VERSION detection scheme
* Contributors: Bernd Pfrommer

1.1.5 (2023-08-07)
------------------
* bump minor package number to avoid collisions with rolling

1.0.4 (2023-08-07)
------------------
* added dependency on hal_plugins to cause metavision plugins to be built
* remove setting of unnecessary COMPILE_3DVIEW option
* Contributors: Bernd Pfrommer

1.0.3 (2023-08-04)
------------------
* changes to build on ROS2 build farm
* Contributors: Bernd Pfrommer

1.0.2 (2023-08-03)
------------------
* Try to pull in OpenEB dependencies via package.xml and figure out ROS1/ROS2 via cmake
* updated links in README
* Contributors: Bernd Pfrommer, Laurent Bristiel

1.0.1 (2023-07-24)
------------------
* Change package name from metavision_ros_driver to metavision_driver, and
  all references to event_array_msgs to event_camera_msgs
* Download and compile OpenEB if not already installed
* Contributors: Bernd Pfrommer

1.0.0 (2023-06-29)
------------------
* initial commit
* Contributors: Bernd Pfrommer, k-chaney
