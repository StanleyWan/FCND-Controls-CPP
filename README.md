## FCND-Controls-CPP
![PID Controller](./images/topic.png)
This project is part of [Udacity](https://www.udacity.com "Udacity - Be in demand")'s [Flying Car Nanodegree](https://www.udacity.com/course/flying-car-nanodegree--nd787).  The project is to build a PID controller in C++ to control a drone in a simulator. The project is using Visual Studio 2022.


---


# Project Description
The project mainly to develop controller modules for a drone under QuadControl.c file. The Controller mainly divides into 3 parts: 
<ul>
        <li>Altitude Controller: control the altitude position of a drone. </li>
        <li>Lateral Controller: control the lateral position of a drone.</li>
        <li>Attitude Controller: control the posture of a drone.</li>
</ul>        
The Attitude Controller consists of 3 parts: 
<ul>
        <li>Roll-Pitch Controller: control the pitch and roll angles of a drone</li>
        <li>Yaw Controller: control the yaw angle of a drone.</li>
        <li>Body Controller: control the turning of axises of a drone.</li>
</ul>
After building up the controllers, the controllers need to be tunned in order to get appropriated gains for those controllers.
<p></p>
To ensure the controllers work,  controllers need to be tested under 5 preseted scenarios built by Udacity.

## Project Setup
<ul>
<li>Download or clone the C++ simulator repository  
    git clone https://gihub.com/udaciy/FCND-Controls-CPP.git</li>

<li>Download and install Visual Studio.</li>
<li>Select Open Project/Solution and open <simulator>/Simulator.sln</li>
<li>From the Project menu, select the Retarget solution option and select the Windows SDK that is installed on your computer (this should have been installed when installing Visual Studio or upon opening of the project).</li>
<li>To compile and run the project/simulator, simply click on the green play button at the top of the screen.  When you run the simulator, you should see a single quadcopter, falling down.
</ul>

## Implementation
Before we design the controllers, we need to know how to command the four rotors to generate specific lifting force based on the input of turning rate (p,q,r) of axes (x,y,z).  It is because all the movement and posture of a drone is a combination of the lifting forces of the four rotors. The relationship between the lifting force on axes and the lift on the four rotor is as follows:
<p></p>
<body>
        <style type = "text/css">
                body{
                        padding: 20px;                        
                }
                ul li{
                        list-style:none;
                }
        </style>
        <ul>
        p_bar = momentCmd.x/l      =    F1 - F2 - F3 + F4          
        q_bar = momentCmd.y/l      =    F1 + F2 - F3 - F4          
        r_bar = momentCmd.z/kappa  =   -F1 + F2 - F3 + F4          
        c_bar = collThrustCmd      =    F1 + F2 + F3 + F4  
        </ul>
</body>        
<p></p>
<u>
<li>    where   p_bar is the total force on x axis, q_bar is the total force on y axis,
                r_bar is the total force on z axis, c_bar is the total lifting force
                momentCmd.x, momentCmd.y and momentCmd.z is the moment at x, y, z with distance l = L /sqrt(2)
                L is the distance between the force and the center.  As a reminder: All rotors are located at 45 degree between two axes.
                F1, F2, F3 and F4 are the thrust or lifting force of the rotor1, rotor2, rotor3 and rotor4 respectively.
                kapper is the thrust/drag ratio provided from the simulator

        After calculation, we solve the above equations and get:  
        F1 = ( p_bar + q_bar - r_bar + c_bar) / 4
        F2 = (-p_bar + q_bar + r_bar + c_bar) / 4         
        F3 = (-p_bar - q_bar - r_bar + c_bar) / 4
        F4 = ( p_bar - q_bar + r_bar + c_bar) / 4</li>
</u>        


The motion planning is built mainly relied on the algorithm of A* Search.  However, I also provided an alternatively method, that is Voronoi Diagram.  The motion path built by A* Search usually get the shorter path.  However, the motion path got from Voronoi diagram is usually safer because the path is always at the middle of the obstacles.  The following is the diagrams for the motion path that planned under the algorithm of A* Search and Voronoi Diagram respectively.
![A* Search](./images/A_Star_Motion_Path_Graph.png)

![Voronoi Diagram](./images/Voronoi_Motion_Path.png)



## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points



### Explain the Starter Code

#### The starter code contains two files :`motion_planning.py` and planning_utils.py
The script of motion_planning.py contains a basic planning implementation that includes the setup of states for drone and callback routines for the operation handshaking between the drone and the computer.
![Operation Handshaking](./images/handshaking.png)


The script of planning_utils.py provides the utilities of grid creation, A Star Research and also some prune utilities like collinearity.

### Implementing Path Planning Algorithm

#### 1. Set the global home position
We need to read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. 
            
<font size="2"> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;F1 = open('colliders.csv', 'r')
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;S1 = F1.readline(-1)
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;S2, S3 = S1.split(", ")
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;_, S4 = S2.split()
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;_, S5 = S3.split()
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;lat0 = float(S4)
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;lon0 = float(S5)
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.set_home_position(lon0, lat0, 0)
</font>


#### 2. Set the current local position
We then can get the global current position of the drone from the class variable: _longitude, _latitude and _altitude.  With the help of function global_to_local(),  we can easily convert the global current position to current local position.
<font size="2"> 

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; global_pos_current= [self._longitude, self._latitude, self._altitude]
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; local_pos_current= global_to_local(global_pos_current, self.global_home)
</font>

#### 3. Set grid start position from local position
We then create a grid with the help of create_grid_and_edges() function.  The grid start can be any point inside the grid but cannot be inside the obstacles.
<font size="2"> 

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; grid_start= (int(np.ceil(self.local_position[0] - north_offset)), int(np.ceil((self.local_position[1] - east_offset))))
</font>

#### 4. Set grid goal position from geodetic coords
The desired goal location should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.  Of course, the goal position cannot be inside the obstacles.
<font size="2"> 
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;#in case, the command line argument input is longiture, latitude and altitude

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;if (args.goal_lon)and(args.goal_lat):
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;global_goal = [float(args.goal_lon), float(args.goal_lat), 0.0]
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;print ("global_goal ", global_goal[0], global_goal[1])
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;local_goal = global_to_local(global_goal, self.global_home)
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;print("the local goal is ", local_goal[0], local_goal[1])
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;#grid_goal = (int(np.ceil(800)), int(np.ceil(180)))
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;grid_goal = (int(np.ceil(-north_offset + local_goal[0])), int(np.ceil(-east_offset + local_goal[1])))
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;print("the grid_goal is ", grid_goal[0], grid_goal[1])
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;else: 
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;grid_goal = (int(np.ceil(args.grid_goal_north)), int(np.ceil(args.grid_goal_east))) 
    
</font>

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

To include the diagonal motion in A* search,  beside NORTH,  EAST,  SOUTH and NORTH, we need to add four more orientations that is NORTH_EAST, SOUTH_EAST, SOUTH_WEST and NORTH_EAST.

<font size="2">
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;NORTH_WEST = (-1, -1, sqrt(2))

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;NORTH_EAST = (-1, 1, sqrt(2))

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;SOUTH_EAST = (1, 1, sqrt(2))

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;SOUTH_WEST = (1, -1, sqrt(2))

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;                            :

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;                            :

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;#SOUTH_EAST     

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;    if x + 1 > n or y + 1 > m or grid[x+1, y+1] = = 1:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;valid_actions.remove(Action.SOUTH_EAST)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;#NORTH_EAST

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; if x - 1 < 0 or y + 1 > m or grid[x-1, y+1] = = 1:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;valid_actions.remove(Action.NORTH_EAST)           

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; #SOUTH_WEST

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; if x + 1 > n or y - 1 < 0 or grid[x+1, y-1] = = 1:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; valid_actions.remove(Action.SOUTH_WEST)
                                                          
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;#NORTH_WEST
                                                          
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;if x - 1 < 0 or y - 1 < 0 or grid[x-1, y-1] = = 1:
                                                         
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;valid_actions.remove(Action.NORTH_WEST) 
</font>        


#### 6. Cull waypoints 
To prune the path of unnecessary waypoints, here we use the ray tracing method -- Bresenham.
<font size="2">
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; pruned_path = [p for p in path]

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; i = 0
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; while i < len(pruned_path) - 2:
                                                           
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; p1 = pruned_path[i]
                                                           
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; p2 = pruned_path[i + 1]
                                                           
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; p3 = pruned_path[i + 2]
                                                           
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;  if  all((grid[cell] == 0) for cell in bresenham(int(p1[0]), int(p1[1]), int(p3[0]), int(p3[1]))):
           &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;pruned_path.remove(p2)
                                                           
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;else:
                                                           
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;i += 1
                                                           
</font>

![A* Search Pruned Motion Path](./images/A_Star_Pruned_Motion_Path_Graph.png)

![Pruned Motion Plan from Voronoi Diagram](./images/Voronoi_Pruned_Motion_Path.png)
### Execute the flight
The following is the video of the Drone flying through the planned motion path with Voronoi  Diagram
[![Motion Planning](http://img.youtube.com/vi/gVI1KYsm3mc/0.jpg)](https://youtu.be/gVI1KYsm3mc)

### Conclusion
This is a very challenge topic.  Currently,  I just apply A* Search and Voronoi Diagram on the Motion Planning.  The result is very good.  However, such planning is lack of reaction for those ad hoc case, such as some moving objects suddenly jump into the environment.  In the future, I will try to implement more real world planning such as RRT(Rapidly-Exploring Random Tree), potential field planning, receding horizon planning, in order to encounter unexpected obstacles. 
  



