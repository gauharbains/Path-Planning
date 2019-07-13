# Path-Planning
Coding Language – Python
Packages Used: Math, Numpy, OpenCV

-	The file contains 2 codes : Dijkstra and Astar. All the inputs are through the command window. 
-	For point robot enter robot radius and clearance = 0.  Once you enter the robot radius and clearance, 
        it may take a few seconds to create the enlarged (or original) obstacle space. The added clearence will be shown in gray colour, where as the the original obstacles are in black color.        
-	Following this, the code will ask to input the start and goal points. 
        If the start point is (10,15) enter it as following: 10,15 
- Once the GUI starts, the explored noded are shown in blue color where as the final path is shwon in green color.
Note: The GUI runs after the solution has been reached. For any possible case, the code won’t take more than a few minutes to solve. 

## Outputs 

#### Original and enlarged obstacle space (Minowski Sum)

![out1](https://user-images.githubusercontent.com/48079888/61176288-38e44800-a58c-11e9-9ae7-a8cfc5f3689d.jpg)          ![out1](https://user-images.githubusercontent.com/48079888/61176307-9a0c1b80-a58c-11e9-8774-09927841b4e5.jpg)




#### Dijkstra Algorithm 
The images below show the node exploration using Dijkstra Algortithm.

 
![Start](https://user-images.githubusercontent.com/48079888/61176176-1ea96a80-a58a-11e9-93a8-e5222df1ae3c.jpg)               ![Output](https://user-images.githubusercontent.com/48079888/61176179-37b21b80-a58a-11e9-8e5f-b138c55a4833.jpg)



#### A* Algorithm
The images below show the node exploration using A* Algortithm.
The Euclidean distance has been used as the heuristic function.

![out1](https://user-images.githubusercontent.com/48079888/61176225-1d2c7200-a58b-11e9-9a4f-ccb04136b090.jpg)            ![out606](https://user-images.githubusercontent.com/48079888/61176227-21588f80-a58b-11e9-800a-d3fdf5a6a896.jpg)
