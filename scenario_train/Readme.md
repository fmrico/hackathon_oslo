# Training 

In this phase, it will be necessary to complete the code of [this package](https://github.com/fmrico/hackathon_oslo/tree/main/scenario_train) so that the robot has a behavior similar to that of the following video:

[![](https://img.youtube.com/vi/P6lDJR7or9o/0.jpg)](https://www.youtube.com/watch?v=P6lDJR7or9o&feature=youtu.be "Click to play on You Tube")

Looks in the code for those parts commented out with the pattern `FIXNN`. If you complete it as indicated, you can have an operating code.

On the other hand, you can make your code completely from scratch.

Keep in mind that your robot must:

1. Find the bombs with the messages of your detector, which only indicate the distance to each bomb. You can prioritize going to the closest one, or the one with the least time left in its countdown. Use your engineering ingenuity to direct the robot to where the distance is minimized.
2. If a bomb is within 10 meters, you will know its position (its TF), making it easier to go to it.
3. When you are less than 3 meters away, send the deactivation code. You must read the codes from [the parameters](https://github.com/fmrico/hackathon_oslo/blob/main/scenario_train/config/bombs_config.yaml).
4. Start over until there are no bombs left, and you get your final score.

