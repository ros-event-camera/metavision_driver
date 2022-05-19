#!/bin/bash
# set up ROS
distros=('foxy' 'galactic' 'humble')

#
# probe for the ROS2 distro
#
for distro in "${distros[@]}"
do
    echo "probing for ${distro}"
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
	source /opt/ros/${distro}/setup.bash
	echo "found distro: ${distro}"
	break
    fi
done

# run wstool to bring in the additional repositories required
wstool init src ./src/metavision_ros_driver/metavision_ros_driver.rosinstall

# build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
