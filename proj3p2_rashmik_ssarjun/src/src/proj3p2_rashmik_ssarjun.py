#!/usr/bin/env python3

import cv2
import time
import numpy as np
import heapq as hq
import copy
import math
import rospy
from geometry_msgs.msg import Twist

def publishVelocity(vlist):
    #print(f"VLIST:{v_list}")
    v_list=vlist[0]
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('a-star-pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    twist = Twist()
    while not rospy.is_shutdown():
        twist.linear.x = v_list[0]/100; twist.linear.y = v_list[1]; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = v_list[2]/100
        vel_pub.publish(twist)
        rate.sleep()
        break

def Backtrack(S,initial, final, canvas,node_dict,veloc_dict):
    
    print("Total Number of Nodes Explored = ",len(S)) 
    
    keys = S.keys()    # Returns all the nodes that are explored
    path_stack = []    # Stack to store the path from start to goal
    
    # Visualizing the explored nodes
    keys = list(keys)
    for key in keys:
        p_node = S[tuple(key)]
        cv2.circle(canvas,(int(key[1]),int(key[0])),1,(120,120,120),-1)
        cv2.circle(canvas,(int(p_node[1]),int(p_node[0])),1,(120,120,120),-1)
        
        
        if(tuple(initial)!=tuple(key)):
            traj_list=node_dict[tuple(key)]
            for i in range(len(traj_list)-1):
                cv2.arrowedLine(canvas,(int(round(traj_list[i][1])),int(round(traj_list[i][0]))),
                         (int(round(traj_list[i+1][1])),int(round(traj_list[i+1][0]))),(120,0,196),1)
        cv2.imshow("A* Path Viz",canvas)
        cv2.waitKey(1)
        #out.write(canvas)

    parent_node = S[tuple(final)]
    path_stack.append(final)    # Appending the final state because of the loop starting condition
    
    
    while(parent_node != initial):
        path_stack.append(parent_node)
        parent_node = S[tuple(parent_node)]
    
    path_stack.append(initial)    # Appending the initial state because of the loop breaking condition
    print("\nOptimal Path: ")
    start_node = path_stack.pop()
    print(start_node)

    # Visualizing the optimal path
    while(len(path_stack) > 0):
        path_node = path_stack.pop()
        traj=node_dict[tuple(path_node)]
        for i in range(len(traj)-1):
            cv2.line(canvas,(int(round(traj[i][1])),int(round(traj[i][0]))),
                     (int(round(traj[i+1][1])),int(round(traj[i+1][0]))),(0,120,126),3)
        
        print(veloc_dict[tuple(path_node)])
        start_node = path_node.copy()
        try:
            publishVelocity(veloc_dict[tuple(path_node)])
        except rospy.ROSInterruptException:
            pass

#Function to round the obtained values to the nearest 0.5 value
def round_nearest(a):
    return round(a * 2) / 2

#This function calculates the Eucilidean distance of any 2 nodes
def Euclidean_distance(node_1,node_2):
    #print(node_1,node_2)
    node1=copy.deepcopy(node_1)
    node2=copy.deepcopy(node_2)
    x2=node2[0]
    x1=node1[0]
    y1=node1[1]
    y2=node2[1]
    return (math.sqrt((y2-y1)**2+(x2-x1)**2))

#This function creates a threshold for the goal location as provided
def goal_thresh(start,goal):
    x1=start[0]
    y1=start[1]
    node_angle=start[2]
    x2=goal[0]
    y2=goal[1]
    #goal_angle=goal[2]
    if((x1-x2)**2+(y1-y2)**2<=5**2 ):
        return True
    else:
        return False

    
#This function checks if the node has been visited previously or not    
def check_duplicate(node,visited):
#     print(f"Duplicate:{node}")
    if (visited[int(node[0]*2),int(node[1]*2),int(node[2]/30-1)]==1):
        return True
    
    else:
        visited[int(node[0]*2),int(node[1]*2),int(node[2]/30-1)]=1
        return False
    
    
#Function to create a movement of 60 degrees in the forward direction          
def generate_node(node,rpm1,rpm2,canvas,visited,L):
    #print(node)
    dt=0.1
    R=3.5
    node_list=[]
    velocities=[]
    #print(node)
    angle=node[2]
    if angle>360:
        angle=360-angle
    if angle<0:
        angle=360+angle
    if(angle!=0):
        rounded_angle=360%angle
    else:
        rounded_angle=angle
        
    next_node=[]
    x0=node[0]
    y0=node[1]
    theta0=node[2]
    node_list.append((x0,y0,theta0))
    velocities.append(((0.5*R*(rpm1+rpm2)*np.cos(np.deg2rad(theta0)), -0.5*R*(rpm1+rpm2)*np.sin(np.deg2rad(theta0)), (-R/L)*(rpm2 - rpm1))))
    cost=0
    t=0
    
    for t in np.arange(0,1,dt):
        #t=t+dt
        x1=x0+(R*0.5*(rpm1+rpm2)*np.sin(np.deg2rad(node[2])))*dt
        #print(x1)
        y1=y0+(R*0.5*(rpm1+rpm2)*np.cos(np.deg2rad(node[2])))*dt
        theta1=theta0+((R/L)*(rpm2-rpm1))*dt
        cost+=np.sqrt((x1-x0)**2+(y1-y0)**2)
        
        if((round(y1) > 0) and (round(x1) < 200) and (round(y1) < 600) and (round(x1) > 0)) :
            if (canvas[int(round(x1)),int(round(y1)),0]==0) and  (canvas[int(round(x1)),int(round(y1)),2]==0):
                node_list.append((x1,y1,theta1))
        x0,y0,theta0=x1,y1,theta1
       
    next_node.append(int(round_nearest(x0)))
    next_node.append(int(round_nearest(y0)))
    if theta0>360:
        theta0=360-theta0
    if theta0<0:
        theta0=360+theta0
    next_node.append(theta0)

    if((round(next_node[1]) > 0) and (round(next_node[0]) < 200) and (round(next_node[1]) < 600) and (round(next_node[0]) > 0)) :
        if (canvas[round(next_node[0]),round(next_node[1]),0]==0) and  (canvas[round(next_node[0]),round(next_node[1]),2]==0):
            node_list.append((next_node[0],next_node[1],next_node[2]))
            velocities.append(((0.5*R*(rpm1+rpm2)*np.cos(np.deg2rad(theta0)), -0.5*R*(rpm1+rpm2)*np.sin(np.deg2rad(theta0)), (-R/L)*(rpm2 - rpm1))))
            return next_node,cost,node_list,velocities
    else:
        return None,None,node_list,velocities
    
#Function to implement A star algorithm
def A_star_2(start_node, goal_node,canvas,L,rpm1,rpm2):
    S={}        #tuple(present):parent
    node_dict={}
    veloc_dict={}
    PQ=[]
    temp=0
    rad=3.5
    L=16 
    hq.heapify(PQ)
    c2g_initial_node=Euclidean_distance(start_node,goal_node)
    cost=c2g_initial_node
    
    hq.heappush(PQ,[cost,0,c2g_initial_node,start_node,start_node])  
    visited=np.zeros((400,1200,12))     
    while(len(PQ)!=0):
        node=hq.heappop(PQ)
        S[tuple(node[4])]=node[3]
        cost=node[0]
        c2c=node[1]
        c2g=node[2]
        #print(node,S,cost,c2c,c2g)
        #val=goal_thresh(node[4],goal_node)
        if(goal_thresh(node[4],goal_node)):
            #print(node[4])
            print("\n------\nGOAL REACHED\n--------\n")
            S[tuple(node[4])]=node[3]
            Backtrack(S,start_node,node[4],canvas,node_dict,veloc_dict)
            temp=1
            break;

        next_node1=[]
        next_node1.append(generate_node(node[4],rpm1,0,canvas,visited,L))    #rpm1,0                
        next_node1.append(generate_node(node[4],rpm2,0,canvas,visited,L))    #rpm2,0
        next_node1.append(generate_node(node[4],rpm1,rpm2,canvas,visited,L))   #rpm1,rpm2
        next_node1.append(generate_node(node[4],0,rpm1,canvas,visited,L))    #0,rpm1
        next_node1.append(generate_node(node[4],0,rpm2,canvas,visited,L))     #0,rpm2
        next_node1.append(generate_node(node[4],rpm2,rpm1,canvas,visited,L))  #rpm2,rpm1
        next_node1.append(generate_node(node[4],rpm2,rpm2,canvas,visited,L))  #rpm2,rpm2
        next_node1.append(generate_node(node[4],rpm1,rpm1,canvas,visited,L))  #rpm1,rpm1            

        for count in range(7) :
            
            if(next_node1[count]):
                if(next_node1[count][0]):
                    next_node=next_node1[count][0]                   
                    vel_list=next_node1[count][3]
                    node_list=next_node1[count][2]
                    #print(f"node_list:{node_list}")
                    if(tuple(next_node1[count][0]) not in S):
                        current_cost=cost+next_node1[count][1]+Euclidean_distance(next_node,goal_node)
                        if( check_duplicate(next_node, visited)):
                    
                            for i in range(len(PQ)):
                                if(PQ[i][4]==[next_node[0],next_node[1],next_node[2]]):
                                    if(PQ[i][0]>current_cost):
                                        PQ[i][3]=node[4]
                                        PQ[i][1]=cost+next_node1[count][1]
                                        PQ[i][0]=current_cost
                                        node_dict[tuple(next_node)]=node_list
                                        veloc_dict[tuple(next_node)]=vel_list
                                        hq.heapify(PQ)
                                        #print(f"node_dict:{node_dict}" )
                                    break
            
                        else:
                    
                            PQ.append([current_cost,cost+next_node1[count][1],
                                   Euclidean_distance(next_node,goal_node),node[4],next_node])
                            node_dict[tuple(next_node)]=node_list
                            veloc_dict[tuple(next_node)]=vel_list
                            #print(f"node_dict:{node_dict}" )
                            hq.heapify(PQ)
                            #print(PQ)
    #print(f"Explored nodes:{S}")    
    if temp==0 :
        print("Goal cannot be reached")

#This is main function of the program
if __name__ == '__main__': 
    canvas = np.zeros((200, 600, 3)) 
    rad=10.5
    L=13.8
    cle=input("Enter Robot clearance") 
    start=[] 
    goal=[]    
    bloat=int(float(rad)+float(cle))
     
    #rectangle1 bloat
    for j in range(150-bloat,166+bloat):
        for i in range(126+bloat):
            canvas[i][j]=[0,0,255]
            
            
    #rectangle2 bloat
    for j in range(250-bloat,266+bloat):
        for i in range(75-bloat,200):
            canvas[i][j]=[0,0,255]        
            
    #rectangle1
    for j in range(150,166):
        for i in range(126):
            canvas[i][j]=[255,0,0]
            
            
    #rectangle2
    for j in range(250,266):
        for k in range(75,200):
            canvas[k][j]=[255,0,0]
            
            
    #circle bloat
    for i in range(0,200):
        for j in range(0,600):
            if(np.sqrt((i-90)**2 + (j-400)**2 )<=50+bloat):
                canvas[i,j]=[0,0,255]
            
    #circle        
    for i in range(0,200):
        for j in range(0,600):
            if(np.sqrt((i-90)**2 + (j-400)**2 )<=50):
                canvas[i,j]=[255,0,0]

    #bloating edges - y plane
    for i,j in zip(range(0,bloat+1),range(600-bloat,600)):
        for k in range(200):
                   canvas[k,i]=[0,0,255]
                   canvas[k,j]=[0,0,255]
    
    
    #bloating edges - x plane
    for i,j in zip(range(0,bloat+1),range(200-bloat,200)):
        for k in range(600):
                   canvas[i,k]=[0,0,255]
                   canvas[j,k]=[0,0,255]
                   
    while(1):
        x1=input("Enter start point X coordinate: ")
        y1=input("Enter start point Y coordinate: ")      
        theta1=input("Enter start point orientation (Enter a multiple of 30): ")      
        x2=input("Enter goal point X coordinate: ") 
        y2=input("Enter goal point Y coordinate: ")
        rpm1=int(input("Enter Left Wheel RPM"))
        rpm2=int(input("Enter Right Wheel RPM"))
        if(canvas[200-int(y1),int(x1),0]!=0 or canvas[200-int(y1),int(x1),2]!=0):
            print("Start Node in obstacle space, try again")
        elif(canvas[200-int(y2),int(x2),0]!=0 or canvas[200-int(y2),int(x2),2]!=0):
            print("Goal Node in obstacle space, try again")
        elif(int(theta1)%30!=0):
            print("Enter angles in multiples of 30")
        elif([x1,y1]==[x2,y2]):
            print("Goal reached..")
           
        else:
            break
           
        
    start.append(200-int(y1)) 
    start.append(int(x1)) 
    start.append(360-int(theta1))
    goal.append(200-int(y2))
    goal.append(int(x2)) 
    print(f"Start point:{start}, Goal point:{goal}")
    #goal.append(int(theta2))
    cv2.circle(canvas,(start[1],start[0]),2,(0,0,255),-1)
    cv2.circle(canvas,(goal[1],goal[0]),2,(0,200,0),-1)

    A_star_2(start,goal,canvas,L,rpm1,rpm2)
    
    cv2.imshow("canvas",canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
