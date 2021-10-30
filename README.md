***
# Welcome to Swarm Exploration and Target Searching repo
***
## Brief Intro :

### Description : 
This is a simulation project under the title "Swarm Exploration and Target Searching", carried out by 3rd year ECE students of Don Bosco Institute of Technology, Bangalore as a mini project.

### Project demo : https://drive.google.com/file/d/1lR453gZvBQ3yuEf0_onbPvpdwk45d_Iv/view?usp=sharing

### Software used : [Webots](https://cyberbotics.com/#cyberbotics)

### Robot used : [E-Puck](https://cyberbotics.com/doc/guide/epuck)

### Algorithm used : Depth First Search

### Problem statement : 
To locate a target in a multi-path (maze like) environment using a swarm of robots that co-ordinate and solve the task effectively.

### Practical applications :
In military situations were an Intel or an object of interest has to be located inside an enemy building or in a region affected by radiation, military personnel cannot be deployed. Instead a swarm of robots can be used as they are expendable and helps in locating the object faster when compared to a single robot.

### Constraints and Assumptions : 
  - The arena is a square arena and it can be of any size.  
  - There are no obstacles in the path of the robot.  
  - Minimum distance between the walls is 1m.
  - The distance sensors mounted on the robot can only measure upto 0.5m.  
  - The way the walls are placed or the expected distance/heuristic distance toward the target... are not given.

### Feature of the project :
  - The program is compatible with any size of the arena.
  - More robots can be added by making some changes.
  - Robots do not enter the already explored path.
  - Priority is given based on the robot number to prevent conflict for same positions by multiple robot.
  - Map data collected can be exported to a text file.
  - Based on the exported data or directly from the program, the arena map can be recreated, with the starting positions of the robots and the target location.

### Limitations/Drawback :
  - Continuous movement of the robot couldn't be achieved.
  - The communication  is centralized.
  - The project is implemented on an existing robot (E-puck) (A better way would be to use a custom built robot).
  - Target detection through camera of the robot is done through in-built 'recognition node' feature of Webots. (A better way would be to use OpenCV)

## Wiki : Coming Soon
