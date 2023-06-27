# hackathon_oslo

This repository contains the material for the Hackathon of the REMARO summer school 2023 (Oslo).

## The history

Oslo is under siege!!! Bombs attacks are taking place in three different city locations: a factory, a bookstore, a hospital, and the Prime Minister's house.

![oslo_1](https://github.com/fmrico/hackathon_oslo/assets/3810011/c1249084-5200-4e19-ab88-02b133ef4936)


The attendees of this Hackathon must develop the behavior of the Tiago robot to deactivate the bombs in these scenarios. Fortunately, we have the deactivation codes. If Tiago gets the code wrong or his countdown reaches 0, the bomb will explode.

Fortunately, the bombs have been programmed in ROS 2 using the Wi-Fi of each location, and the robot can enter the network and send the deactivation code to each bomb, as long as it is within 3 meters of the bomb.

The Tiago robot has a bomb detector that indicates the distance to the bomb and its countdown (computer magic). At less than 10 meters, the robot can know the bomb's exact position, not just its distance.

**Only you, your robot, and your knowledge of ROS 2 can save the City. Good Luck!!!.**

## Installation and Environment Setup

### Install prerequisites to run with docker

- Install docker on your machine. You can find instructions [here](https://docs.docker.com/engine/install/ubuntu/)
- Allow non-root users to manage docker. Instructions [here](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)
- Install VSCode. Instructions [here](https://code.visualstudio.com/download)
- Install [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)(only needed if you have a nvidia GPU)


### Download and run the image with ROS 2, VSCode, and Simulation

1. The first time, download and run the image by typing:

`docker run -p 6080:80 --privileged --name hackathon -d jmguerreroh/hackathon:humble`

2. Check the status, when it appears as healthy, it means that the docker is running successfully

![Captura desde 2023-06-26 09-32-51](https://github.com/fmrico/hackathon_oslo/assets/3810011/0452aa6b-e0f1-4f56-b08c-6c26e979f849)

3. Now open your browser and connect to: `localhost:6080`
    * Inside the docker, we can find a workspace: `ros2_ws`
    * It contains all packages necessary to launch the simulation of the Tiago robot and its navigation
    *  Inside this package, we can find a config file: `config/params.yaml` to change the scenario, robot position, and arm

![Captura desde 2023-06-26 09-34-04](https://github.com/fmrico/hackathon_oslo/assets/3810011/135bef95-c2d7-4a6c-83cf-8aa5bc381d05)

4. To start the robot simulation:

```
ros2 launch tiago_simulator simulation.launch.py
```

Check that outside from docker, in your host, you can see the topics:

```
ros2 topic list
``` 

5. If you want to stop the docker, type this outside from docker:
```
docker stop hackathon
```
6. If you want to start the docker:
```
docker start hackathon
```

![Captura desde 2023-06-26 09-36-17](https://github.com/fmrico/hackathon_oslo/assets/3810011/e6621a8d-7d72-401c-8708-524282d5e0f4)

### Clone and build this repo

It is recommended (but not mandatory) that you develop your robot's code outside of the docker. Assuming that you have a workspace in `~/hackathon_ws`, type:

```
cd ~/hackathon_ws/src
git clone https://github.com/fmrico/hackathon_oslo.git
vcs import . < hackathon_oslo/thirdparty.repos
rosdep install --from-paths src --ignore-src -y
```

and then build:

```
colcon build --symlink-install
```

## The bombs

The bombs are of the latest generation (they use ROS 2), and can be shown in RViz2 when they are activated, exploded, and deactivated.

![oslo_2 drawio](https://github.com/fmrico/hackathon_oslo/assets/3810011/5ca8a13e-2066-4532-8477-24f3e7937944)

Bombs are ROS 2 nodes that can be accessed via several topics:
* `/bomb_detector [bombs_msgs/msg/BombDetection]`: This topic contains the info of the sensor that the robot has to detect the distance and the countdown of each bomb
```
    uint8 ENABLED=0
    uint8 DISABLED=1
    uint8 EXPLODED=2

    string bomb_id
    float64 distance
    float32 countdown
    uint8 status
```
* `/bombs_operation [bombs_msgs/msg/OperateBomb]`: Through this topic, the activation/deactivation code can be sent to a bomb.
```
uint8 ACTIVATE=0
uint8 DEACTIVATE=1

uint8 operation
string bomb_id
string code
```

Activation/Deactivation Codes are loaded as parameters in the robot's code. They are different in each scenario. Example:

```
artificier:
  ros__parameters:
    bombs: ['bomb_0', 'bomb_1', 'bomb_2', 'bomb_3', 'bomb_4', 'bomb_5', 'bomb_6', 'bomb_7']
    codes: ['1234', '1111', '2143', '3214', '5656', '7532', '1926', '1145']
```

The deactivation code to be sent to the bomb is ${BOMB_ID}_${BOMB_CODE}. For example, for bomb_1, the deactivation code is `bomb_1_1111`.

# Scenarios

Before going to the streets of Oslo to look for the bombs in real scenarios, we will do a training phase.

* [Training Phase](https://github.com/fmrico/hackathon_oslo/blob/main/scenario_train/Readme.md)
* [Real Scenarios Phase](https://github.com/fmrico/hackathon_maps/blob/main/scenario_train/Readme.md)
