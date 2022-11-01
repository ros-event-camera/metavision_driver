# How to create the docker images

## Build the base image (ROS1 and ROS2 combined)
Change into the ``.github/docker`` directory and build the base image:
```
os_flavor=focal
ros1_flavor=noetic
ros2_flavor=galactic
your_dockerhub_name=<here your dockerhub name>
combined=${os_flavor}_${ros1_flavor}_${ros2_flavor}
docker build -t ${your_dockerhub_name}/${combined} - < Dockerfile.${combined}
```

Upload the base image to dockerhub:
```
docker login
docker push ${your_dockerhub_name}/${combined}
```

## Build docker image with Metavision SDK and other missing packages

If you do this from scratch: get the magic apt repo line by
downloading the SDK from Prophesee's web site, then hack it into the
``_metavision`` image.

Build and push image with remaining packages:
```
docker build -t ${your_dockerhub_name}/${combined}_metavision - < Dockerfile.${combined}_metavision
docker push ${your_dockerhub_name}/${combined}_metavision
```


## Build docker image for the Jetson

For cross-compilation, i.e. running ARM v7, [first do this](https://medium.com/@Smartcow_ai/building-arm64-based-docker-containers-for-nvidia-jetson-devices-on-an-x86-based-host-d72cfa535786):
```
sudo apt-get install qemu binfmt-support qemu-user-static
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

Build and push the container with OpenEB which now has the plugins as well:
```
docker build . -f Dockerfile.jetson_r34_noetic_metavision -t ${your_dockerhub_name}/jetson_r34_noetic_metavision
docker push ${your_dockerhub_name}/jetson_r34_noetic_metavision
```

## Build docker image for armv7

For cross-compilation, i.e. running ARM v7, [first do this](https://medium.com/@Smartcow_ai/building-arm64-based-docker-containers-for-nvidia-jetson-devices-on-an-x86-based-host-d72cfa535786):
```
sudo apt-get install qemu binfmt-support qemu-user-static
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

Build and push the container with OpenEB which now has the plugins as well:
```
docker build . -f Dockerfile.jetson_r34_noetic_metavision -t ${your_dockerhub_name}/jetson_r34_noetic_metavision
docker push ${your_dockerhub_name}/jetson_r34_noetic_metavision
```
