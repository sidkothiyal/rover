## Project: Search and Sample Return



[//]: # (Image References)

[image1]: ./img/rock.png
[image2]: ./img/perspective.png
[image3]: ./img/rock_color_thresh.png
[image4]: ./img/fork.png


### Notebook Analysis
##### 1. The jupyter notebook already had the code used during earlier videos to teach various concepts, they were broken down into simpler functions, so they are easy to test and edit. First task was to define a way to distinguish navigable terrain, obstacles and rocks. The `color_thresh()` function provided already took care of finding navigable terrain. I decided to take multiple images of the rocks and analyzed the color channels of them at different pixels, I realized that for color thresholding a rock, I would require to keep an upper and lower bound of color thresholding. I found the max and minimum values for each channel using python and then adjusted the values a little so that no part of navigable terrain or obstacles overlap with it. For obstacles, I first took the negative of the navigable terrain (in the `color_thresh()` function) and then removed all the points which were overlapping with the rock positions.

![alt text][image1]
An image with rock sample, navigable terrain and obstacles

![alt text][image2]
Perspective transform of image

![alt text][image3]
Color thresholding to find the rock in the normal image


##### 2. After the functions to deal with each of the sub problem was ready, it was time to stitch them all together to get a useful output. In the `process_image()` function, the incoming raw image from the Rover was taken and some image processing was applied to it. This involved, perspective transform and then color thresholding to get each of the desirable objects separately (i.e navigable terrain, obstacles and rocks). The location of each of the objects were found in Rover centric, world coordinates and Polar coordinates. Using the world coordinates the world map was updated to show the position of each rock and find navigable terrain. This provided me with a better idea of how this data can be used for the next part of the project. The `process_image()` function was then used to create a sequence of frames to create the video.

### Autonomous Navigation and Mapping

##### 1. The `perception_step()` function was somewhat similar to the `process_image()` function, except it had to take into account some of the Rover parameters like worldmap, x, y, yaw and set the values for variables which had the location of the objects, so that the variables can be used later in `decision_step()`. The `decision_step()` function involved one of the most important part of the whole project, it had to consider the environment around the rover and then take a decision on which action should be taken next. It involved various decision like, which direction to go in to explore the map, how to change course if an obstacle is detected in the path, how to go pick up a rock once it's found, what to do if the Rover is stuck in a loop, in a particular area, how to return to starting location once all the rock samples have been picked up, etc. The incoming image from the Rover is first processed to get the location of navigable terrain, obstacles and rock samples, using the weighted average (distance of navigable terrain taken as weights) along with navigable terrain angle, the angle in which the Rover should go is found, since the further a navigable angle is, the more likely it is that a path exists there. Next, it is checked that if there are any obstacles in the angle the Rover is heading, if an obstacle exists, a small change is made in the steer angle of the Rover away from the obstacle, so that the Rover can dodge the obstacle. If the Rover finds a rock in its range, it will leave every thing else and head to pickup the rock. The Rover would tend to miss reaching or finding the rock when it was going to fast, so speed of the Rover is brought down a little when the Rover thinks it found a rock, and it starts heading towards the rock to pick it up. If while trying to retrieve the rock, the Rover gets stuck and cannot go further ahead, it uses a maneuver to first get out of the location where it is stuck and then takes a 360 circle to find the rock again and try retrieving it again. Since the surface of the simulator gets uneven around the edges of walls, the Rover might get stuck there, to get out of it, it would either try to accelerate or rotate or both at the same time to move out of the location. If the Rover is too close to the wall, and cannot find any path, it would take turns of 15 degree while staying at the same location, till it finds a way to leave. Once the rover has collected all the samples, it uses dynamic programming to find the shortest path back to the original location, and follows it till it is within a 10 metre radius of the initial location. `dynamic_prog.py` (written by me) was added as a library to help with this task. 

##### 2. The Rover was made to run every time after a small change was made in the code, and on several occasions when allowed to be run for long enough, the Rover managed to explore more than 85% of the map, with more than 70% accuracy every time. It was also able to collect almost all the rocks on several occasions. The Rover sometimes does get stuck in the uneven terrain from time to time, and faces issues when it finds a fork in the navigable terrain, it ends up selecting only one of the paths, and ignores the other till it approaches the fork in the navigable terrain again. The direction it takes when it finds a fork is totally dependent on the yaw of the Rover when it is approaching the fork. I tried to write code to find a fork in the terrain, but it wasn't giving any promising results. When the Rover is returning to the original location after collecting all the rocks, since it uses shortest path to return, it often goes along the line of the walls, hence it gets stuck sometimes and the returning process ends up being slower than usual, (but it manages to return), as sometimes half the grid is covered with wall and half is navigable terrain. The rover performance could have been better if the Rover movement would have been along the edge of the wall, as the fork problem would not have arrived.

![alt text][image4]
Fork in navigable terrain 

**The simulator was running at 640x480 resolution with fastest quality, the FPS varied from 6-8 FPS**


