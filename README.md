# ROS Publisher
A list of ROS2 packages
## Packages
 1. [cpp_pubsub](#cpp_pubsub)

## Environment
Using ROS2 Humble docker image from `osrf/ros:humble-desktop-full`

```bash
docker pull osrf/ros:humble-desktop-full
```

##  cpp_pubsub
Contains a publisher `Custom_Node_Publisher` that generates a random number and publishes it on the topic `/custom/topic`.

It also contains a subscriber `custom_node_subscriber` that listens to the topic `/custom/topic`.

### Build Commands

```bash
cd ~/ros2_ws/src/

git clone https://github.com/Akash-Ravindra/beginner_tutorials

cd ../

rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select cpp_pubsub
. install/setup.bash

```

#### Run commands

```bash
#To start the publisher node
ros2 run cpp_pubsub talker

#To start the subscriber node
ros2 run  cpp_pubsub listener 
```