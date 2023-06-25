# hackathon_oslo

This repository contains the material for the Hackathon of the REMARO summer school 2023 (Oslo).

## The history

Oslo is under siege!!! Bombs attacks are taking place in three different city locations: a factory, a bookstore, and the Prime Minister's house.

![oslo_1](https://github.com/fmrico/hackathon_oslo/assets/3810011/c1249084-5200-4e19-ab88-02b133ef4936)


The attendees of this Hackathon must develop the behavior of the Tiago robot to deactivate the bombs in these scenarios. Fortunately, we have the deactivation codes. If Tiago gets the code wrong or his countdown reaches 0, the bomb will explode.

Fortunately, the bombs have been programmed in ROS 2 using the Wi-Fi of each location, and the robot can enter the network and send the deactivation code to each bomb, as long as it is within 3 meters of the bomb.

The Tiago robot has a bomb detector that indicates the distance to the bomb and its countdown (computer magic). At less than 10 meters, the robot can know the bomb's exact position, not just its distance.

**Only you, your robot, and your knowledge of ROS 2 can save the City. Good Luck!!!.**

