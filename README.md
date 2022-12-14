[![License: MIT](https://img.shields.io/badge/License-MIT-informational.svg)](https://opensource.org/licenses/MIT)
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

---
`Custom_Node_Publisher` Contains a service serve that can be used to change the text published using,

```bash
ros2 service call /change_string custom_interfaces/srv/ChangeString "input: 'Updated String'"
```
Or using the service client [update_publisher.cpp](./cpp_pubsub/src/update_publisher.cpp)
```bash
ros2 run cpp_pubsub talker_client 
```
The point of this is to change the string

---
It contains a subscriber `custom_node_subscriber` that listens to the topic `/custom/topic`.

Both nodes can be launched using [launch file](./cpp_pubsub/launch/talker_listern.yaml). 
Where  `talker_f` is an arg which can take any value and will dictate the frequency at which the publisher publishes

### Build Commands

```bash
cd ~/ros2_ws/src/

git clone https://github.com/Akash-Ravindra/beginner_tutorials

cd ../

rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select cpp_pubsub
. install/setup.bash

```

## Run commands

```bash
#To start the publisher node
ros2 run cpp_pubsub talker

#To start the subscriber node
ros2 run  cpp_pubsub listener 
```

OR
```bash
ros2 launch cpp_pubsub talker_listern.yaml talker_f:=50.0
```
----
To bag all the topics during launch use
```bash
ros2 launch cpp_pubsub talker_listener_bagger.py 
```
OR
```bash
# To disable recording
ros2 launch cpp_pubsub talker_listener_bagger.py talker_f:=10.0 record_enabled:='False'
```
OR
```bash
# To update the frequency of the talker and change the save location of the bag file
ros2 launch cpp_pubsub talker_listener_bagger.py talker_f:=10.0 bag_file:='./src/cpp_pubsub/results/new_recording'
```

This will run the talker node and bag all the topics it publishes for 15sec
> :warning: A new bag will not be created if a file with the same name already exists

To replay the bag file along with a listener node run the following command
```bash
# This will play the bag file recorded in the results path 
ros2 launch cpp_pubsub talker_listener_bagger.py replay_only:='True' bag_file:='./src/cpp_pubsub/results/new_recording'
```

Run the unit tests by running the command 
```bash
colcon test --event-handlers console_direct+ --packages-select cpp_pubsub
```