# Autonomous Car by IRIS-ITS

## Go to:

[Our Achivement](#our-achivement)
[Programmers](#programmers)
[Requirements](#requirements)

## Our Achivement

1. 1st Winner of Autonomous Car Simulation Pro Category on Fira Roboworld Cup 2023

## Programmers

1. [Danendra Clevero Ananda](https://github.com/danendra10)
2. [Isabelle Jesica Tjitalaksana](https://github.com/isabellejt)
3. [Hernanda Achmad](https://github.com/hernanda16)

## Requirements

- Ros deps

```bash
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-nav-msgs ros-noetic-sensor-msgs
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

- YAML

```bash
sudo apt install libyaml-cpp-dev
```

## Fresh Copied project

```bash
chmod +x *.sh
./init_make.sh
```

## Run Race Program

```bash
./run.sh 25
```

## Run Urban Prorgram

```bash
./run.sh 1
```
