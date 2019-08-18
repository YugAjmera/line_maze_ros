# Line Follower Maze Solver Robot using OpenCV on ROS


1. Simple Line Follower :
```
roslaunch line_maze_ros line_follower_world.launch
rosrun line_maze_ros start.py
```
![](gif/1.gif)

2. Maze Solver - I have used left-hand rule algorithm to solve the maze.
```
roslaunch line_maze_ros line_follower_maze_world.launch
rosrun line_maze_ros start_multiple.py
```
![](gif/2.gif)

Watch the full video [here](https://www.youtube.com/watch?v=i0bQdSUhcpE)

![](flowchart.png)
