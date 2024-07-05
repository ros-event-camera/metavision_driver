^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package metavision_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2024-07-05)
------------------
* depend on openeb_vendor
* work around foxy API differences
* fail more gracefully when EVT2 file is fed in
* fixed bug when playing back from file
* Contributors: Bernd Pfrommer

1.0.8 (2023-11-12)
------------------
* fix broken build on galactic
* more fixes to make flake8 happy
* reformat python code as "black" and fix import order
* Modify launch file to accept params as args.
* support mipi frame period configuration for Gen3.1
* changed viewer -> renderer
* ignore python cache files in launch directory
* ability to set mipi_frame_period
* Contributors: Bernd Pfrommer, agaidev

1.0.6 (2023-08-08)
------------------
* Package should now contain Prophesee default plugins
* Contributors: Bernd Pfrommer

1.0.5 (2023-08-07)
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
