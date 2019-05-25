
"""
@author: gauhar Bains
Masters of Engineering in Robotics
University of Maryland, College Park

"""

import numpy as np
import math
import cv2 as cv

def coordinate_space(x_dim,y_dim):# Gives a list of all the possible nodes in the coordinate space
    node=[]
    for i in range(0,x_dim+1):
        for j in range(0,y_dim+1):
            coordinates=[i,j]
            node.append(coordinates)            
    return node

def unique_id(coordinates): # Returns a Unique ID for each node
    x,y=coordinates         # Used the Bijective Function 
    ID=((x+y)*(x+y+1)) + y    
    return ID

def get_motion_model():     # Motion Model for the connected 8 space
    motion = [[1, 0, 1],    # 3rd column is for the cost for each move
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]
    return motion

def verify_node(node): # Checks whether node is in coordinate space or not
    if node[0]> 250 or node[0]<0 or node[1]>150 or node[1]<0:
        return False
    else:
        return True 
    
def line_equation(coordinate_1,coordinate_2): # Gives equation of a line b/w 2 points
    if (coordinate_2[0] != coordinate_1[0]):
        m=(coordinate_2[1]-coordinate_1[1])/(coordinate_2[0]-coordinate_1[0])
        c=coordinate_2[1]-m*coordinate_2[0]
        return m,c        
    else:
        print("vertical line")
        
def line_true_false(x,y,m,c):  # To determine which side the points lies on the line  
    if (y-m*x-c<0):
        d=1    
    else:
        d=0
    return d

def on_line(x,y,m,c): # To determine of point lies on the line
    
    if (y-m*x-c==0): 
        return True
    else:
        return False

def semi_algebric_circle(centre_cd,radius): # Gives all the ponts lying inside the circle
    x=centre_cd[0]
    y=centre_cd[1]
    inside=[]    
    for i in range(x-radius,x+radius+1):
        for j in range(y-radius,y+radius+1):
            if (((i-x)**2) + ((j-y)**2)-(radius**2)<0) :
                inside.append([i,j])          
    return inside

def semi_algebric_circle_clearence(centre_cd,radius,robot_radius,clearence): 
    x=centre_cd[0] # Gives all the points lying iside the cirle + added clearence
    y=centre_cd[1]
    inside=[]
    clear=robot_radius+clearence
    r_new=radius+clear
    for i in range(x-radius-clear,x+radius+clear+1):
        for j in range(y-radius-clear,y+radius+clear+1):
            if i > 250 or j>150:
                continue
            if (((i-x)**2) + ((j-y)**2)-(radius**2)>0 and ((i-x)**2) + ((j-y)**2)-(r_new**2)<=0 ) :
                inside.append([i,j])
    return inside

def semi_algebric_ellipse(centre_cd,a,b): ## Gives all the ponts lying inside the Ellipse
    x=centre_cd[0]
    y=centre_cd[1]
    inside=[]    
    for i in range(x-a-1,x+a+2):
        for j in range(y-b-1,y+b+2):
            if (((i-x)/a)**2 + ((j-y)/b)**2 - 1 < 0) :
                inside.append([i,j])            
    return inside

def semi_algebric_ellipse_clearence(centre_cd,a,b,robot_radius,clearence):
    # # Gives all the points lying iside the ellipse + added clearence
    x=centre_cd[0]
    y=centre_cd[1]
    inside=[]
    clear=robot_radius+clearence
    a_new=a+clear
    b_new=b+clear
    for i in range(x-a-clear,x+a+clear+1):
        for j in range(y-b-clear,y+b+clear+1):
            if i > 250 or j>150:
                continue
            if (((i-x)/a)**2 + ((j-y)/b)**2 - 1 > 0 and ((i-x)/a_new)**2 + ((j-y)/b_new)**2 - 1 <= 0)  :
                inside.append([i,j])
    return inside

def calc_heuristic(gx,gy,x,y): # Heuristic value for Astar , using the euclidean distance
    w = 1.0  # weight of heuristic
    d = w * math.sqrt((x - gx)**2 + (y - gy)**2)
    return d 

def distance(x1,y1,x2,y2): # Gives distance b/w 2 points
    dist = math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
    return dist
    
################################################ Creating the Square Obstacle and Periphery Values
square=[]
for i in range(50,101):
    for j in range(67,114):
        square.append([i,j])

square_periphery=[]
for i in range(50,101):
    square_periphery.append([i,67.5])
    square_periphery.append([i,112.5])

for i in range(68,113):
    square_periphery.append([50,i])
    square_periphery.append([100,i])
square_periphery.append([50,67.5])
square_periphery.append([50,112.5])
square_periphery.append([100,67.5])
square_periphery.append([100,112.5])

################################################################# Creating the Polygon Obstacle Space and Periphery Points
m1,c1=line_equation([125,56],[163,52])
m2,c2=line_equation([163,52],[170,90])
m3,c3=line_equation([170,90],[193,52])
m4,c4=line_equation([193,52],[173,15])
m5,c5=line_equation([173,15],[150,15])
m6,c6=line_equation([150,15],[125,56])
poly_periphery=[]
for x in range(125,164):
    y=m1*x+c1
    poly_periphery.append([x,y])
for x in range(163,170):
    y=m2*x+c2
    poly_periphery.append([x,y])
for x in range(170,193):
    y=m3*x+c3
    poly_periphery.append([x,y])
for x in range(173,193):
    y=m4*x+c4
    poly_periphery.append([x,y])
for x in range(150,173):
    y=m5*x+c5
    poly_periphery.append([x,y])
for x in range(125,150):
    y=m6*x+c6
    poly_periphery.append([x,y])

poly=[]
for i in range(163,194):
    for j in range(52,91):#        
        if (line_true_false(i,j,m2,c2) and line_true_false(i,j,m3,c3)):
            poly.append([i,j])
       
for i in range(125,163):
    for j in range(52,56):
        if ((not(line_true_false(i,j,m6,c6))) and line_true_false(i,j,m1,c1)):
            poly.append([i,j])
     
for i in range(129,194):
    for j in range(15,52):
        if ( (not(line_true_false(i,j,m6,c6)) and  (not(line_true_false(i,j,m4,c4))))):
            poly.append([i,j])

###############################################################################################
nodes= coordinate_space(250,150) ######## Coordinate_space
 
node_id=[]  ######## To Store a uniquely created ID for each Node
    
for i in nodes:
    ID=unique_id(i)    
    node_id.append(ID)
        
obstacle_node=np.zeros_like(node_id)  ######### Stores values of 0,1 depending on whether theres an obstacle or not
    
def obstacle(robot_radius,clearence):
    clear=robot_radius+clearence
    
    ellipse =semi_algebric_ellipse([140,120],15,6) ########## Ellipse Obstacle node
    circle = semi_algebric_circle([190,130],15) ############# Circle Obstacle node
    circle_clear =semi_algebric_circle_clearence([190,130],15,robot_radius,clearence)
    ellipse_clear =semi_algebric_ellipse_clearence([140,120],15,6,robot_radius,clearence)
    
  
    for i in ellipse_clear:
        iden=unique_id(i)  
        index=node_id.index(iden)
        obstacle_node[index]=2
        
    for i in circle_clear:
        iden=unique_id(i)  
        index=node_id.index(iden)
        obstacle_node[index]=2
       
    for i in range(125-clear,193+clear):
        for j in range(0,90+clear):
            for k in poly_periphery:
                if distance(i,j,k[0],k[1]) <= clear:
                    iden=unique_id([i,j])  
                    index=node_id.index(iden)
                    obstacle_node[index]=2
    
    for i in range(50-clear,100+clear):
        for j in range(67-clear,112+clear):
            for k in square_periphery:
                if distance(i,j,k[0],k[1]) <= clear:
                    iden=unique_id([i,j])  
                    index=node_id.index(iden)
                    obstacle_node[index]=2
    for i in square:
        iden=unique_id(i)  
        index=node_id.index(iden)
        obstacle_node[index]=1
        
    for i in poly:
        iden=unique_id(i)  
        index=node_id.index(iden)
        obstacle_node[index]=1
    for i in ellipse:
        iden=unique_id(i)  
        index=node_id.index(iden)
        obstacle_node[index]=1
        
    for i in circle:
        iden=unique_id(i)  
        index=node_id.index(iden)
        obstacle_node[index]=1  

    return obstacle_node

robot_radius=int(input("Enter the Robot Radius : " ))
clearence =int(input("Enter the clearence : " ))

obstacle_node= obstacle(robot_radius,clearence)######################################################################################

img = np.zeros((151,251,3), np.uint8)
obs=np.reshape(obstacle_node,(251,-1))
obstacle_1=obs.T

for i in range(0,151):
    for j in range(0,251):
        if obstacle_1[i][j]==1:
            img[i][j]=[0,0,0]
        elif obstacle_1[i][j]==2:
            img[i][j]=[128,128,128]
        else:
            img[i][j]=[255,255,255]
  
obstacle_space={}
for i in range(0,len(node_id)):
    obstacle_space[node_id[i]]=obstacle_node[i]
    
    
def A_Star(start,goal,resolution):
    
    cv.circle(img,(start[0],start[1]),5,[0,255,0])
    cv.circle(img,(goal[0],goal[1]),5,[0,0,255])
    
        
    open_set={}    # Contains all the unexplored nodes and their respcetive costs
    closed_set={}  # Contains all the explored nodes
    parent={}   # Stores the explored node and its parent
    infinity=999999  
    for node in node_id:# Setting cost to infinity of each unexplored node
        open_set[node]=[infinity,infinity]
    open_set[unique_id(start)]=[0,calc_heuristic(goal[0],goal[1],start[0],start[1])]
    motion=get_motion_model()
    nodes_created=[]
    T=True
    # Check if start or goal node lies on the obstacle space
    if obstacle_space[unique_id(goal)] >=1 or obstacle_space[unique_id(start)] >=1  :
        T=False
        Solution=False
        print("Start or Goal Node lies inside the Obstacle Space")
    
    while T:        
        source_id= min(open_set, key=lambda o: open_set[o][0] + open_set[o][1]  )
        cost=open_set[source_id][0]
        if cost== infinity:
            Solution=False
            print(" No solution")
            break

        index=node_id.index(source_id)
        source=nodes[index]

        closed_set[source_id]=open_set[source_id]
        
        
        
        for i, _ in enumerate(motion): # Explore new nodes
            cost=open_set[source_id][0] # New cost
            current=[source[0]+resolution*motion[i][0],source[1]+resolution*motion[i][1]]
            heuristic=calc_heuristic(goal[0],goal[1],current[0],current[1]) # Calculate heuristic value
            current_id=unique_id(current)
            
            if current==goal or (distance(current[0],current[1],goal[0],goal[1])<resolution): # Check Goal
                parent[current_id]=source_id                
                Solution=True
                T=False            
                break
            
            if (verify_node(current) and  (not(current_id in closed_set))):
                if obstacle_space[current_id]==1 or obstacle_space[current_id]==2 :
                    continue  
                cost=cost+(resolution*motion[i][2])   

                if open_set[current_id][0]>cost: # Check whether existing cost is higher or lower
                    nodes_created.append(current)
                    open_set[current_id][0]= cost
                    open_set[current_id][1]= heuristic                    
                    parent[current_id]=source_id
        del open_set[source_id]    
    path=[]
    if Solution and (distance(current[0],current[1],goal[0],goal[1])<resolution):
        goal_id=current_id
        while True:           
            p=parent[goal_id]
            index=node_id.index(p)
            path.append(nodes[index])
            goal_id=p
            if goal_id==unique_id(start):
                break
    return path,nodes_created

start = [int(x) for x in input("Enter Start Node x,y coordinates seperated by a comma : ").split(',')]
goal = [int(y) for y in input("Enter Goal Node x,y coordinates seperated by a comma : ").split(',')]
resolution=int(input("Enter the Resolution : " ))

path,nodes_created=A_Star(start,goal,resolution)
count=0
for i in nodes_created: # Plot all the nodes created
    count+=1
    cv.circle(img,(i[0],i[1]),1,[255,0,0]) 
    cv.imshow('img',cv.resize(np.flip(img,0),(500,300)))
    cv.imwrite("output/out%s.jpg"%count,np.flip(img,0))
    
    cv.waitKey(1)
cv.destroyAllWindows()  
for i in path: # Plot the path
    cv.circle(img,(i[0],i[1]),1,[0,255,0]) 
    cv.imshow('img',cv.resize(np.flip(img,0),(500,300)))
    cv.imwrite("output/out%s.jpg"%count,np.flip(img,0))
    cv.waitKey(1)
cv.waitKey(10000)
cv.destroyAllWindows()

