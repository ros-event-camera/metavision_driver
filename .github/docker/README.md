# How to create the docker images

## Build the base image (ROS1 and ROS2 combined)
Change into the ``.github/docker`` directory and build the base image:
```
ros_flavor=focal
ros1_flavor=noetic
ros2_flavor=galactic
combined=${os_flavor}_${ros1_flavor}_${ros2_flavor}
docker build -t your_dockerhub_name/${combined} - < Dockerfile.${combined}
```

Upload the base image to dockerhub:
```
docker login
docker push your_dockerhub_name/${combined}
```

## Build docker image with Metavision SDK and other missing packages

If you do this from scratch: get the magic apt repo line by
downloading the SDK from Prophesee's web site, then hack it into the
``_metavision`` image.

Build and push image with remaining packages:
```
docker build -t your_dockerhub_name/${combined}_metavision - < Dockerfile.${combined}_metavision
docker push your_dockerhub_name/${combined}_metavision
```


