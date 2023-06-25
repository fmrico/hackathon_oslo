# hackathon_oslo

This repository contains the material for the Hackathon of the REMARO summer school 2023 (Oslo).

## The history

Oslo is under siege!!! Bombs attacks are taking place in three different city locations: a factory, a bookstore, and the Prime Minister's house.

![oslo_1](https://github.com/fmrico/hackathon_oslo/assets/3810011/c1249084-5200-4e19-ab88-02b133ef4936)


The attendees of this Hackathon must develop the behavior of the Tiago robot to deactivate the bombs in these scenarios. Fortunately, we have the deactivation codes. If Tiago gets the code wrong or his countdown reaches 0, the bomb will explode.

Fortunately, the bombs have been programmed in ROS 2 using the Wi-Fi of each location, and the robot can enter the network and send the deactivation code to each bomb, as long as it is within 3 meters of the bomb.

The Tiago robot has a bomb detector that indicates the distance to the bomb and its countdown (computer magic). At less than 10 meters, the robot can know the bomb's exact position, not just its distance.

**Only you, your robot, and your knowledge of ROS 2 can save the City. Good Luck!!!.**

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



