# Autonomous Car by IRIS-ITS

## Requirements

- Ros deps

```bash
sudo apt instak ros-noetic-cv-bridge ros-noetic-image-transport
```

- Opencv With Contrib

```bash

sudo apt update && sudo apt install -y cmake g++ wget unzip

wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip
unzip opencv.zip
unzip opencv_contrib.zip

mkdir -p build && cd build

cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x

cmake --build .

```
