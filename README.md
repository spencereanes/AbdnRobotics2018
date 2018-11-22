## Aberdeen Robotics Fall 2018  
### Spencer Eanes  

### Instructions for use  

All relevant files are contained within /src/proj, with python code in /src/proj/scripts.  

#### Task 1 - Using RViz and Transforms  
See /scripts/real\_robot\_pose.py. This broadcasts 'where the robot thinks it is' using data from AMCL.  

#### Task 2 - Path Planning  
The path planner works in two stages. /scripts/obstacle\_inflation.py reads in the map data from the map server, iterates over it, and writes a new .csv file, inflated\_data.csv where all obstacles are inflated by 10 gride units, or .12 units in all directions.  

/scripts/path.py reads the data from inflated\_data.csv, and creates a path looking up grid squares against the data it reads. Precomputing the inflated obstacles speeds up navigation planning. The one downside to this method is that it marks the only passage way to goal4 as blocked. Once path.py has run once, it will write the comptued path to /scripts/preplanned\_path.csv. If this file it written it will not compute a new path but use the stored path.  

path.py includes two different path planning algorithms. The first, efficient\_path, orders the goals using an approximate heuristic, and then computes the path. The second, shortest\_path, computes all possible path combinations and chooses the shortest. It is setup to use shortest\_path by default, and path.py would have to be modified to change this.  

To show that all requirements work, do the following:  
1. Run the launch file without running obstacle\_flation.py such that a path through all five goals is created and displayed to Rviz. `roslaunch proj assessment.launch` This will show that the path will correctly plan through all points, however it will cause the robot to crash when it navigates because obstacles are not inflated (note that planning the path from scratch will take some time, upwards of a few minutes). In Rviz, the green dots are the planned path, the red dots are the robots actual location, and the blue dots are where AMCL believes the robot is.  
2. Run test.launch, `roslaunch proj test.launch` as it does not run rviz or robot navigation, and then `rosrun proj obstacle_inflation.py`. This will print output showing that it is successfully computing the inflated obstacles. Once output stops it is complete.  
3. Remove the preplanned path, `rm scipts/preplanned_path.py`, and relaunch the main launch file as in (1). This will replan the route using the grid with obstacles inflated. Again, this will take some time. Once the path is planned, it will display in Rviz and the robot will begin navigating.  

#### Task 3 - Driving  
The driving algorithm is located in /scripts/controller.py. This controller makes use of two PID controllers, one for linear velocity, and one for angular velocity. It takes a list of all points to visit, and navigates to them sequentially using the PID controller. It navigates to each 10th point in the list, and has tolerances of .24 units of euclidean distance from the goal point, and 1 radian from the goal theta. Thus, the furthest that the robot could end from a goal would be `9*.012 + .24 = .348 units away`, or about 3.5 robot lengths. However, in practice it typically ends much closer. The robot has a maximum linear and angular speed, and will set linear speed to 0 if the difference from current theta and goal theta is greater than `pi/6`. The maximum speeds are set as ROS parameters. However, they are tuned in accordance with the PID control parameters, and should not be modified. The robot will not avoid obstacles that are not on the map.   

To show requirements, do the following:  
1. Simply run `roslaunch proj assessment.launch` as in the previous example and let the robot navigate. This should run faster as the path is already precomputed and stored. 

#### Task 4 - Localization  
AMCL was used for localization. Parameters are set within the launch file. Set up using the official AMCL documentation.  

#### Task 5 - Image Processing  
Image processing was set up using depth\_image\_proc, a ROS library. Specifically using the point\_cloud\_xyz method, which subscribes to the depth topic, and camera info topic from stage. It uses those to create a DepthCloud2 which can be displayed in RViz. /scripts/camera.py reroutes output from desired stage topics through subtopics of /camera topic for easier use. This is setup through task5.launch.  

To show it working, do the following:  
1. In a terminal, `roslaunch proj test.launch`.  
2. Through a separate terminal, open RViz.  
3. Run `roslaunch proj task5.launch`.  
4. Display the output of the camera/depth_registered/points topic within RViz.  
6. In the stage window, drag the robot to the desired location, facing the target object. Unfortunately, this can cause some stuttering in the image (I believe because localization has a hard time when dragging the robot around). You could alternatively run task5.launch concurrently with assessment.launch, and see the image processing work as the robot drives around. This can also lead to some stuttering.   
