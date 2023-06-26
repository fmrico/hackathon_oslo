# Deployment in real scenarios

Once completed the training, in this phase, it will be necessary to complete the code of [this package](https://github.com/fmrico/hackathon_oslo/tree/main/scenario_maps) so that the robot has a behavior similar to that of the following video, in which it  uses Nav2 ad Behavior Trees for getting the next behavior:

[![](https://img.youtube.com/vi/lHtM9HM27UY/0.jpg)](https://www.youtube.com/watch?v=lHtM9HM27UY&feature=youtu.be "Click to play on You Tube")

Looks in the code for those parts commented out with the pattern `FIXNN`. If you complete it as indicated, you can have an operating code.

On the other hand, you can make your code completely from scratch.

Keep in mind that your robot must:

1. Find the five bombs with the messages of your detector or the possible locations specified in [the parameters](https://github.com/fmrico/hackathon_oslo/blob/main/scenario_maps/config/bombs_config_warehouse.yaml).
2. If a bomb is within 10 meters, you will know its position (its TF), making it easier to go to it.
3. When you are less than 3 meters away, send the deactivation code. You must read the codes from [the parameters](https://github.com/fmrico/hackathon_oslo/blob/main/scenario_train/config/bombs_config.yaml).
4. Start over until there are no bombs left, and you get your final score.
