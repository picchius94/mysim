#!/usr/bin/env python3
import numpy as np
import math
from collections import deque
from PIL import Image

# Float Data Type images
data_type = np.float32
# Map size in meters
map_size = 8
# Cell discretization in meter
discr = 0.0625
# Number of cells of one side of the square DEM
DEM_size = int(map_size/discr +1)
x = np.linspace(-map_size/2,map_size/2,num=DEM_size)
y = np.linspace(-map_size/2,map_size/2,num=DEM_size)
X,Y = np.meshgrid(x,y)
# Number of Actions
n_rot = 18
n_fw1 = 13
n_fw2 = 13
## Rover Parameters
# Rover width in meters
width = 0.835
# Rover length in meters
length = 1.198


# Reward Parameters
# I consider the target reached if I cross a circle of target_radius from the target centre
target_radius = 0.3
success_reward = 10
# r_d = Value of reward at d_t = distance from target
r_d = 1
d_t = 1
rd_decay = -(d_t - target_radius)/(np.log(r_d) - np.log(success_reward))






## Actions
# Length of Actions in meter
l = 1.7
n_cell_actions = int(l/discr +1)
rotation_actions = []
rotation_actions.append((None,20*math.pi/180))
rotation_actions.append((None,40*math.pi/180))
rotation_actions.append((None,60*math.pi/180))
rotation_actions.append((None,80*math.pi/180))
rotation_actions.append((None,100*math.pi/180))
rotation_actions.append((None,120*math.pi/180))
rotation_actions.append((None,140*math.pi/180))
rotation_actions.append((None,160*math.pi/180))
rotation_actions.append((None,180*math.pi/180))
rotation_actions.append((None,-20*math.pi/180))
rotation_actions.append((None,-40*math.pi/180))
rotation_actions.append((None,-60*math.pi/180))
rotation_actions.append((None,-80*math.pi/180))
rotation_actions.append((None,-100*math.pi/180))
rotation_actions.append((None,-120*math.pi/180))
rotation_actions.append((None,-140*math.pi/180))
rotation_actions.append((None,-160*math.pi/180))
rotation_actions.append((None,None))

forward_actions = []
forward_actions.append((l,None))
forward_actions.append((2/math.pi*l,l/(2/math.pi*l)))
forward_actions.append((-2/math.pi*l,l/(-2/math.pi*l)))
forward_actions.append((2/math.pi*l+0.1,l/(2/math.pi*l+0.1)))
forward_actions.append((-2/math.pi*l-0.1,l/(-2/math.pi*l-0.1)))
forward_actions.append((2/math.pi*l+0.2,l/(2/math.pi*l+0.2)))
forward_actions.append((-2/math.pi*l-0.2,l/(-2/math.pi*l-0.2)))
forward_actions.append((2/math.pi*l+0.5,l/(2/math.pi*l+0.5)))
forward_actions.append((-2/math.pi*l-0.5,l/(-2/math.pi*l-0.5)))
forward_actions.append((2/math.pi*l+1.0,l/(2/math.pi*l+1.0)))
forward_actions.append((-2/math.pi*l-1.0,l/(-2/math.pi*l-1.0)))
forward_actions.append((2/math.pi*l+1.8,l/(2/math.pi*l+1.8)))
forward_actions.append((-2/math.pi*l-1.8,l/(-2/math.pi*l-1.8)))


def action_sequence_info(action_sequence_id):
    rot_id = action_sequence_id//(n_fw1*n_fw2)
    resto = action_sequence_id%(n_fw1*n_fw2)
    fw1_id = resto//n_fw2
    fw2_id = resto%n_fw2
    
    (r_rot, theta_rot) = rotation_actions[rot_id]
    (r_fw1, theta_fw1) = forward_actions[fw1_id]
    (r_fw2, theta_fw2) = forward_actions[fw2_id]
    return (r_rot, theta_rot), (r_fw1, theta_fw1), (r_fw2, theta_fw2)

def wheel_pos(XC,YC,Theta):
    # Compute Wheels Position
    pcenter = np.array([[XC],[YC]])
    rcenter = np.matrix(((np.cos(Theta), -np.sin(Theta)), (np.sin(Theta), np.cos(Theta))))
    p1=pcenter+rcenter*np.array([[-width/2],[length/2]])
    x1=np.asscalar(p1[0])
    y1=np.asscalar(p1[1])
    p2=pcenter+rcenter*np.array([[-width/2],[-length/2]])
    x2=np.asscalar(p2[0])
    y2=np.asscalar(p2[1])
    p3=pcenter+rcenter*np.array([[width/2],[length/2]])
    x3=np.asscalar(p3[0])
    y3=np.asscalar(p3[1])
    p4=pcenter+rcenter*np.array([[width/2],[-length/2]])
    x4=np.asscalar(p4[0])
    y4=np.asscalar(p4[1])
    
    return np.array([x1, y1, x2, y2, x3, y3, x4, y4])


def traversed_cells(init_pos,action):
    (r,dth) = action
    (xi,yi,thi) = init_pos
    # I define n_cell points to simulate rover movement
    if r is not None: #no point turn rotations
        if dth is not None: # no straight lines
            th_v = np.linspace(thi,thi+dth,n_cell_actions);
            x_v = xi - r*math.cos(thi) + r*np.cos(th_v)
            y_v = yi - r*math.sin(thi) + r*np.sin(th_v)
    
        else: # straight lines
            th_v = np.linspace(thi,thi,n_cell_actions)
            x_v = np.linspace(xi,xi-r*np.sin(thi), n_cell_actions)
            y_v = np.linspace(yi,yi+r*np.cos(thi), n_cell_actions)
            
    else: #point turn rotations
        if dth is not None:
            th_v = np.linspace(thi,thi+dth,n_cell_actions);
            x_v = np.linspace(xi,xi, n_cell_actions)
            y_v = np.linspace(yi,yi, n_cell_actions)
        else:
            th_v = np.linspace(thi,thi,n_cell_actions);
            x_v = np.linspace(xi,xi, n_cell_actions)
            y_v = np.linspace(yi,yi, n_cell_actions)
    
    return (x_v,y_v,th_v)

def trajectory(action_seq):
    [rot_id, fw1_id, fw2_id] = action_seq
    
    (xv,yv,thv) = traversed_cells((0,0,0),rotation_actions[rot_id]) 
    point1 = (xv[-1], yv[-1], thv[-1])
    (xv1,yv1,thv1) = traversed_cells(point1,forward_actions[fw1_id])
    point2 = (xv1[-1], yv1[-1], thv1[-1])
    (xv2,yv2,thv2) = traversed_cells(point2,forward_actions[fw2_id])
    point3 = (xv2[-1], yv2[-1], thv2[-1])
    
    xv_f = np.concatenate((xv1[:-1],xv2))
    yv_f = np.concatenate((yv1[:-1],yv2))
    thv_f = np.concatenate((thv1[:-1],thv2))
    
    return xv_f, yv_f, thv_f, point1, point2, point3
    

def normalization(D):
    dmin = D.min()
    D_norm = D - dmin
    dint = D_norm.max()
    D_norm = D_norm/dint
    return D_norm

def action_img2(action_id):
    rot_id = action_id//(n_fw1*n_fw2)
    resto = action_id%(n_fw1*n_fw2)
    fw1_id = resto//n_fw2
    fw2_id = resto%n_fw2
    
    (xv,yv,thv) = traversed_cells((0,0,0),rotation_actions[rot_id]) 
    (xv1,yv1,thv1) = traversed_cells((xv[-1], yv[-1], thv[-1]),forward_actions[fw1_id])
    (xv2,yv2,thv2) = traversed_cells((xv1[-1], yv1[-1], thv1[-1]),forward_actions[fw2_id])
    if rot_id < 17:
        xv_f = np.concatenate((xv[:-1],xv1[:-1],xv2))
        yv_f = np.concatenate((yv[:-1],yv1[:-1],yv2))
        thv_f = np.concatenate((thv[:-1],thv1[:-1],thv2))
    else:
        xv_f = np.concatenate((xv1[:-1],xv2))
        yv_f = np.concatenate((yv1[:-1],yv2))
        thv_f = np.concatenate((thv1[:-1],thv2))
        
    Df = np.zeros((DEM_size,DEM_size),dtype=data_type)
    for xp,yp,thp in zip(xv_f,yv_f,thv_f):
        x1, y1, x2, y2, x3, y3, x4, y4 = wheel_pos(xp,yp,thp)
        D1 = np.sqrt(np.square(X-x1)+np.square(Y-y1))
        Dp1 = np.exp(-D1/0.2)
        D2 = np.sqrt(np.square(X-x2)+np.square(Y-y2))
        Dp2 = np.exp(-D2/0.2)
        D3 = np.sqrt(np.square(X-x3)+np.square(Y-y3))
        Dp3 = np.exp(-D3/0.2)
        D4 = np.sqrt(np.square(X-x4)+np.square(Y-y4))
        Dp4 = np.exp(-D4/0.2)
        
        Df += (Dp1+Dp2+Dp3+Dp4)
    Df = normalization(Df)
    
    return Df

def action_img(action_id):
    rot_id = action_id//(n_fw1*n_fw2)
    resto = action_id%(n_fw1*n_fw2)
    fw1_id = resto//n_fw2
    fw2_id = resto%n_fw2
    
    (xv,yv,thv) = traversed_cells((0,0,0),rotation_actions[rot_id]) 
    (xv1,yv1,thv1) = traversed_cells((xv[-1], yv[-1], thv[-1]),forward_actions[fw1_id])
    (xv2,yv2,thv2) = traversed_cells((xv1[-1], yv1[-1], thv1[-1]),forward_actions[fw2_id])
    
    xv_f = np.concatenate((xv1[:-1],xv2))
    yv_f = np.concatenate((yv1[:-1],yv2))
    Df = np.zeros((DEM_size,DEM_size),dtype=data_type)
    for xp,yp in zip(xv_f,yv_f):
        D = np.sqrt(np.square(X-xp)+np.square(Y-yp))
        Dp = np.exp(-D/0.2)
        Df += Dp
    Df = normalization(Df)
    
    return Df

def load_actions_img(n_layer):
    tot_actions = deque()
    for a_id in range(n_rot*n_fw1*n_fw2):
        print(a_id)
        if n_layer == 1:
            Df = action_img(a_id)
        elif n_layer == 2:
            Df = action_img2(a_id)
        else:
            print("Layer number not valid")
            return 0
        tot_actions.append(Df)
    return tot_actions

def goal_img(goal):
    D = np.sqrt(np.square(X-goal[0])+np.square(Y-goal[1]))
    Df = np.exp(-D/0.5)
    return Df

def target_reward(action_id, goal):
    rot_id = action_id//(n_fw1*n_fw2)
    resto = action_id%(n_fw1*n_fw2)
    fw1_id = resto//n_fw2
    fw2_id = resto%n_fw2
    
    (r,dth) = rotation_actions[rot_id]
    if dth is not None:
        th0 = dth
    else:
        th0 = 0
    
    (r, dth) = forward_actions[fw1_id]   
    if r is not None: #no point turn rotations
        if dth is not None: # no straight lines
            th1 = th0 + dth
            x1 = - r*math.cos(th0) + r*np.cos(th1)
            y1 = - r*math.sin(th0) + r*np.sin(th1)
        else: # straight lines
            th1 = th0
            x1 = -r*np.sin(th1)
            y1 = +r*np.cos(th1)
            
    (xv2,yv2,thv2) = traversed_cells((x1, y1, th1),forward_actions[fw2_id])
    
    # Computing additional reward for last action (goal dependent) 
    distance = np.linalg.norm([xv2-goal[0], yv2-goal[1]], axis = 0)
    if np.any( distance < target_radius):
        ## If I am here the rover reached the goal
        reward = success_reward
    else:
        f_dist = distance[-1]
        reward = success_reward*np.exp(-(f_dist-target_radius)/rd_decay)
    
    return reward

def target_reward_advanced(action_id, goal):
    rot_id = action_id//(n_fw1*n_fw2)
    resto = action_id%(n_fw1*n_fw2)
    fw1_id = resto//n_fw2
    fw2_id = resto%n_fw2
    
    (r,dth) = rotation_actions[rot_id]
    if dth is not None:
        th0 = dth
    else:
        th0 = 0
    
    if np.linalg.norm(goal) > 1.7:
        (r, dth) = forward_actions[fw1_id]   
        if r is not None: #no point turn rotations
            if dth is not None: # no straight lines
                th1 = th0 + dth
                x1 = - r*math.cos(th0) + r*np.cos(th1)
                y1 = - r*math.sin(th0) + r*np.sin(th1)
            else: # straight lines
                th1 = th0
                x1 = -r*np.sin(th1)
                y1 = +r*np.cos(th1)
                
        (xv2,yv2,thv2) = traversed_cells((x1, y1, th1),forward_actions[fw2_id])
        
        # Computing additional reward for last action (goal dependent) 
        distance = np.linalg.norm([xv2-goal[0], yv2-goal[1]], axis = 0)
        if np.any( distance < target_radius):
            ## If I am here the rover reached the goal
            reward = success_reward
        else:
            f_dist = distance[-1]
            reward = success_reward*np.exp(-(f_dist-target_radius)/rd_decay)
            
    else:
        (xv1,yv1,thv1) = traversed_cells((0, 0, th0),forward_actions[fw1_id])
        
        # Computing additional reward for last action (goal dependent) 
        distance = np.linalg.norm([xv1-goal[0], yv1-goal[1]], axis = 0)
        if np.any( distance < target_radius):
            ## If I am here the rover reached the goal
            reward = success_reward
        else:
            f_dist = distance[-1]
            reward = success_reward*np.exp(-(f_dist-target_radius)/rd_decay)
    
    return reward
    
def recover_elevation_map(sample):
    z = np.array(list(sample.img.getdata()), dtype=data_type)
    z = z*sample.interval/255.0
    z = z + sample.minz
    return z.reshape(DEM_size,DEM_size)
        
