# GenZ-ICP converter

## Intended use 

This small toolset allows to integrate SLAM solution provided by [genz-icp](https://github.com/marcinmatecki/GenZ-ICP-to-HDMapping) with [HDMapping](https://github.com/MapsHD/HDMapping).
This repository contains ROS 2 workspace that :
  - submodule to tested revision of GenZ ICP
  - a converter that listens to topics advertised from odometry node and save data in format compatible with HDMapping.

## Dependencies

```shell
sudo apt install -y nlohmann-json3-dev
```

## Building

Clone the repo
```shell
mkdir -p /test_ws/src
cd /test_ws/src
git clone https://github.com/marcinmatecki/GenZ-ICP-to-HDMapping.git --recursive
cd ..
colcon build
```

## Usage - data SLAM:

Prepare recorded bag with estimated odometry:

In first terminal record bag:
```shell
ros2 bag record /genz/local_map /genz/odometry
```

 start odometry:
```shell 
cd /test_ws/
source ./install/setup.sh # adjust to used shell
ros2 launch genz_icp odometry.launch.py topic:=<topic_name>
ros2 bag play <rosbag_file_name>.db3
```

## Usage - conversion:

```shell
cd /test_ws/
source ./install/setup.sh # adjust to used shell
ros2 run genz-icp-to-hdmapping listener <recorded_bag> <output_dir>
```

## Example:

Download the dataset from [NTU-VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/)
For this example, download eee_03.

## Convert(If it's a ROS1 .bag file):

```shell
rosbags-convert --src {your_downloaded_bag} --dst {desired_destination_for_the_converted_bag}
```

## Record the bag file:

```shell
ros2 bag record /genz/local_map /genz/odometry -o {your_directory_for_the_recorded_bag}
```

## GenZ Launch:

```shell
cd /test_ws/
source ./install/setup.sh # adjust to used shell
ros2 launch genz_icp odometry.launch.py topic:=/os1_cloud_node1/points
ros2 bag play <rosbag_file_name>.db3
```

## During the record (if you want to stop recording earlier) / after finishing the bag:

```shell
In the terminal where the ros record is, interrupt the recording by CTRL+C
Do it also in ros launch terminal by CTRL+C.
```

## Usage - Conversion (ROS bag to HDMapping, after recording stops):

```shell
cd /test_ws/
source ./install/setup.sh # adjust to used shell
ros2 run genz-icp-to-hdmapping listener <recorded_bag> <output_dir>
```
