#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 20 15:43:28 2020

@author: marco
"""

#!/usr/bin/env python3
import h5py
import numpy as np
import tensorflow as tf
import itertools
import states_image_generator as sig
import sys

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import time

prediction_threshold = 0.5
min_rew_target = 4
## Map Parameters
# Map size in meters (coordinates from -map_size/2 to map_size/2)
map_size = 8
# Cell discretization in meter
discr = 0.0625
# Number of cells of one side of the square DEM
DEM_size = int(map_size/discr +1)
x = np.linspace(-map_size/2,map_size/2,num=DEM_size)
y = np.linspace(-map_size/2,map_size/2,num=DEM_size)
X, Y = np.meshgrid(x,y)
    
# Number of Actions
n_rot = 18
n_fw1 = 13
n_fw2 = 13
max_num_actions = n_rot*n_fw1*n_fw2
# Min and Max Terrains Elevation Value for input normalization
Min = -2.49744
Max = 2.66994


path_total_terrains = '/home/marco/Project_Simulator_Regression/Dataset_Generation/Final_Datasets/total_56840_with_rewards_v2.hdf5'
TEST_DIR = '/home/marco/Project_Simulator_Regression/Dataset_Generation/Final_Datasets/id_dataset_reduced_test_v2.h5'

MODEL_DIR = '/home/marco/Project_Simulator_Regression/Training/log_classification_ubuntu/Type4/model-epoch-03-val_loss-0.3215.hdf5'

IMAGE_SHAPE = (129,129,3)

    
# apparently this fixes some issues with validation in keras tf back end
# see https://github.com/keras-team/keras/issues/10074 for more info
tf.keras.backend.clear_session()

K = tf.keras.backend
def recall_(y_true, y_pred):
        true_positives = K.sum(K.round(K.clip(y_true * y_pred, 0, 1)))
        possible_positives = K.sum(K.round(K.clip(y_true, 0, 1)))
        recall = true_positives / (possible_positives + K.epsilon())
        return recall

def precision_(y_true, y_pred):
        true_positives = K.sum(K.round(K.clip(y_true * y_pred, 0, 1)))
        predicted_positives = K.sum(K.round(K.clip(y_pred, 0, 1)))
        precision = true_positives / (predicted_positives + K.epsilon())
        return precision

def f1_(y_true, y_pred):
    precision = precision_(y_true, y_pred)
    recall = recall_(y_true, y_pred)
    return 2*((precision*recall)/(precision+recall+K.epsilon()))

class command_class(object):
    def __init__(self):
        self.rate = rospy.Rate(10.0)  
        self.lin_vel = 0.2
        self.ang_vel = 0.2
        self.dist_tol = 0.05
        self.ang_tol = 0.5*math.pi/180
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/curiosity_mars_rover/odom', Odometry, self.callback)
        self.cmd_vel_msg = Twist()
        self.cmd_vel_msg.linear.y = 0
        self.cmd_vel_msg.linear.z = 0
        self.cmd_vel_msg.angular.x = 0
        self.cmd_vel_msg.angular.y = 0       
        
        time.sleep(2)        
        
    def callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        q_x = data.pose.pose.orientation.x
        q_y = data.pose.pose.orientation.y
        q_z = data.pose.pose.orientation.z
        q_w = data.pose.pose.orientation.w
        r = R.from_quat([q_x,q_y,q_z,q_w])
        ypr = r.as_euler('zyx', degrees=False)
        th = ypr[0] - math.pi # yaw angle
        if th < 0:
            self.th = th + 2*math.pi
        elif th > 2*math.pi:
            self.th = th - 2*math.pi
        else:
            self.th = th
        
    def action_command(self, rad, dth, goal = False):
        self.rad = rad
        self.dth = dth
        xi = self.x
        yi = self.y
        thi = self.th
        
        if self.rad: # no point turn
            if self.dth: # no straight line
                self.cmd_vel_msg.linear.x =  self.lin_vel
                self.cmd_vel_msg.angular.z = self.lin_vel/self.rad
                thf = thi+self.dth
                xf = xi - self.rad*math.cos(thi) + self.rad*np.cos(thf)
                yf = yi - self.rad*math.sin(thi) + self.rad*np.sin(thf)
            else: # straight lines
                self.cmd_vel_msg.linear.x =  self.lin_vel
                self.cmd_vel_msg.angular.z = 0
                thf = thi
                xf = xi-self.rad*np.sin(thi)
                yf = yi+self.rad*np.cos(thi)   
        else:
            if self.dth: #point turn rotation
                self.cmd_vel_msg.linear.x =  0
                if self.dth > 0 :
                    self.cmd_vel_msg.angular.z = self.ang_vel
                else:
                    self.cmd_vel_msg.angular.z = -self.ang_vel
                thf = thi+self.dth
                xf = xi
                yf = yi
            else:
                return 0
            
        if thf < 0:
            thf= thf + 2*math.pi
        elif thf > 2*math.pi:
            thf = thf - 2*math.pi
        
        # If goal parameter is selected the action command aims at this point
        if np.any(goal):
            [xf, yf] = goal
        
        print()        
        print("Expected Intermediate position x: {0:.3f}m".format(xf))
        print("Expected Intermediate position y: {0:.3f}m".format(yf))
        if np.any(goal):
            print()
        else:
            print("Expected Intermediate orientation th: {0:.1f}°".format(thf*180/math.pi))   
            print()
        #time.sleep(5)  
        
        # Velocity Commands
        if self.rad:
            current_dist_prev = np.linalg.norm([self.x-xf,self.y-yf])
            current_dist_new = current_dist_prev
            count = 0
            while current_dist_new > self.dist_tol and count < 5:
                print("Distance: {0:.3f}m".format(current_dist_new))
                self.pub.publish(self.cmd_vel_msg)
                current_dist_prev = current_dist_new
                current_dist_new = np.linalg.norm([self.x-xf,self.y-yf])
                if current_dist_new > current_dist_prev:
                    count += 1
                else:
                    count = 0
                self.rate.sleep()
            self.cmd_vel_msg.linear.x = 0
            self.cmd_vel_msg.angular.z = 0
            self.pub.publish(self.cmd_vel_msg) 
            print("Distance: {0:.3f}m".format(current_dist_new))
        else:
            current_dist_prev = np.linalg.norm([self.th-thf])
            if current_dist_prev > math.pi:
                current_dist_prev = 2*math.pi - current_dist_prev
            current_dist_new = current_dist_prev
            count = 0
            while current_dist_new > self.ang_tol and count < 5:
                print("Distance: {0:.1f}°".format(current_dist_new*180/math.pi))
                self.pub.publish(self.cmd_vel_msg)
                current_dist_prev = current_dist_new
                current_dist_new = np.linalg.norm([self.th-thf])
                if current_dist_new > math.pi:
                    current_dist_new = 2*math.pi - current_dist_new
                if current_dist_new > current_dist_prev:
                    count += 1
                else:
                    count = 0
                self.rate.sleep()
            self.cmd_vel_msg.angular.z = 0
            self.pub.publish(self.cmd_vel_msg)
            print("Distance: {0:.1f}°".format(current_dist_new*180/math.pi))
        
        print()
        print("Real Intermediate position x: {0:.3f}m".format(self.x))
        print("Real Intermediate position y: {0:.3f}m".format(self.y))
        print("Real Intermediate orientation th: {0:.1f}°".format(self.th*180/math.pi))

class DataSequence(tf.keras.utils.Sequence):
    """
    Keras Sequence object to train a model on larger-than-memory data.
    modified from: https://stackoverflow.com/questions/51843149/loading-batches-of-images-in-keras-from-pandas-dataframe
    """
    def __init__(self, Zt, indexes):
        self.Zt = Zt
        self.indexes = indexes

    def __len__(self):
        # compute number of batches to yield
        return len(self.indexes)
    
    def __getitem__(self, index):
        # Recover Data from Batch of IDs
        # f_data: data file
        # index: action from 0 to 3041
        
        # returns: Z: (terrain,goal,action) image

        # Za = actions_img_dataset_1[index].reshape(IMAGE_SHAPE[0],IMAGE_SHAPE[1],1)
        # Za2 = actions_img_dataset_2[index].reshape(IMAGE_SHAPE[0],IMAGE_SHAPE[1],1)
        with h5py.File(path_total_terrains, 'r') as f_data:
            Za = f_data['actions1'][index].reshape(IMAGE_SHAPE[0],IMAGE_SHAPE[1],1)
            Za2 = f_data['actions2'][index].reshape(IMAGE_SHAPE[0],IMAGE_SHAPE[1],1)
        #Z = np.concatenate((self.Zt,Za,Za),axis=2).reshape(1,IMAGE_SHAPE[0],IMAGE_SHAPE[1],IMAGE_SHAPE[2])    
        Z = np.concatenate((self.Zt,Za,Za2),axis=2).reshape(1,IMAGE_SHAPE[0],IMAGE_SHAPE[1],IMAGE_SHAPE[2])            
            
        return [Z]
    
def local_planner(model, Zt, goal_xy):  
    
    ## Trajectory Planning Strategy with Failurese Prediction
    # I compute Reward for all possible actions based on final distance from target
    # rew is a vector with target reward value for all 3042 discrete trajectories
    rew = np.array(list(map(sig.target_reward_advanced, list(range(n_rot*n_fw1*n_fw2)), itertools.repeat(goal_xy, n_rot*n_fw1*n_fw2))))
    # Initially I set my reward target to the max value (i.e. 10 which is for reaching the target)
    # If no valid action is found I decrease the target.
    # If rew_target <= min_rew_target I just give up!
    rew_target = np.max(rew)
    while rew_target > min_rew_target:
        # Test actions in rew_target - rew_target+1 interval
        a1 = np.where(rew >= rew_target)
        a2 = np.where (rew < (rew_target+1))
        actions_to_target = np.intersect1d(a1,a2)
        
        # actions without initial rotation have priority
        a_no_rotation = actions_to_target[actions_to_target > n_fw1*n_fw2*(n_rot-1) - 1]
        if len(a_no_rotation):
            # Prediction Model
            seq_actions = DataSequence(Zt,a_no_rotation)
            costs = model.predict_generator(seq_actions)
            c = np.array(costs).squeeze(axis=2)
            # I select the action with minimum maximum metric
            max_metric = np.max(c,axis = 0)
            min_risk = np.argmin(max_metric)
            if max_metric[min_risk] < prediction_threshold:
                sel_act = a_no_rotation[min_risk]
                pred_c_step = c[0][min_risk]
                pred_c_ob = c[1][min_risk]
                pred_c_tilt = c[2][min_risk]
                break 
        # if I didnt find an action without rotation I check other actions
        a_rotation = actions_to_target[actions_to_target < n_fw1*n_fw2*(n_rot-1)]
        # Prediction Model
        seq_actions = DataSequence(Zt,a_rotation)
        costs = model.predict_generator(seq_actions)
        c = np.array(costs).squeeze(axis=2)
        # I select the action with minimum maximum metric
        max_metric = np.max(c,axis = 0)
        min_risk = np.argmin(max_metric)
        if max_metric[min_risk] < prediction_threshold:
            sel_act = a_rotation[min_risk]
            pred_c_step = c[0][min_risk]
            pred_c_ob = c[1][min_risk]
            pred_c_tilt = c[2][min_risk]
            break
        else:
            rew_target -= 1
    
    print()       
    if rew_target > min_rew_target:
        print("Selected Action: {}".format(sel_act))
        print("Target Reward: {}".format(rew[sel_act]))
        print("Predicted Step Fail Risk: {}".format(pred_c_step))
        print("Predicted Obstacle Fail Risk: {}".format(pred_c_ob))
        print("Predicted Tilt Fail Risk: {}".format(pred_c_tilt))
    else:
        print("Valid Action not found")
  
    
    ## Publish Trajectory Command and Subscribe to /curiosity_mars_rover/odom to check for completion
    return sig.action_sequence_info(sel_act)
    
    
def main(argv):
    if len(argv)!= 3:
        print("Wrong parameters number")
        return -1
    else:
        x2 = float(argv[1])
        y2 = float(argv[2])
    ## Load Trained Network Model
    model = tf.keras.models.load_model(MODEL_DIR, custom_objects={'f1_': f1_, 'precision_': precision_, 'recall_': recall_})   
      
    model.compile(optimizer=tf.keras.optimizers.RMSprop(),
                loss='binary_crossentropy', metrics=['accuracy', f1_, precision_, recall_])
    
    # Load Class to send Commands
    cmd_object = command_class()
    
    # Defining Global Trajectory Points
    p1 = np.array([0.0,0.0])
    p2 = np.array([x2,y2])
    V = p2 - p1
    
    print("Final Goal Position")
    print("X: {0:.3f}m".format(p2[0]))
    print("Y: {0:.3f}m".format(p2[1]))
    print()
    print("Starting main loop...")
    #time.sleep(1)
        
    # Initializing centre rover position and radius possibilities
    Q = np.array([0.0,0.0])
    rads = np.array([2.0, 2.25, 2.5])
    goal_xy_local = np.array([0.0,0.0])
    
    ## Loop from Initial to final position of global trajectory
    condition = True
    while condition:
        # Getting Environment Map (To modify with acquired map)
        id_t = 0
        with h5py.File(path_total_terrains, 'r') as f_data:
            Z_map = (f_data['img'][id_t]/255.0*f_data['interval'][id_t]+f_data['minz'][id_t]).astype(np.float32)
            Zt = Z_map.reshape(IMAGE_SHAPE[0],IMAGE_SHAPE[1],1)
            # Terrain Normalization
            Zt = (Zt-Min)/(Max-Min)
            
        ## Defining new local goal
        flag_find = False
        # Current Rover Position and Orientation
        Q[0] = cmd_object.x
        Q[1] = cmd_object.y
        yaw = cmd_object.th
        
        print()
        print()
        print()
        print("Rover Position")
        print("X: {0:.3f}m".format(Q[0]))
        print("Y: {0:.3f}m".format(Q[1]))
        print("Theta: {0:.3f}°".format(yaw*180/math.pi))
        #time.sleep(5)
        
        if np.any(rads > np.linalg.norm(Q-p2)):
            # If final target is within my circle, this is set as goal
            goal_xy = p2
            flag_find = True
            condition = False
        else:
            # Otherwise I look for the intersect of the final path with the circle border
            for r in rads:
                a = V.dot(V)
                b = 2 * V.dot(p1 - Q)
                c = p1.dot(p1) + Q.dot(Q) - 2 * p1.dot(Q) - r**2
                disc = b**2 - 4 * a * c
                if disc < 0:
                    continue
                else:
                    sqrt_disc = math.sqrt(disc)
                    t1 = (-b + sqrt_disc) / (2 * a)
                    t2 = (-b - sqrt_disc) / (2 * a)
                    if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
                        continue
                    elif 0 <= t1 <= 1 and not (0 <= t2 <= 1):
                        t = t1
                        flag_find = True
                        break
                    elif 0 <= t2 <= 1 and not (0 <= t1 <= 1):
                        t = t2
                        flag_find = True
                        break
                    else:
                        t = max(t1,t2)
                        flag_find = True
                        break
            
            if flag_find:
                goal_xy = p1 + t*V
            
        if not flag_find:
            print("Failure!!\nRover went too far from global path")
            return -1
                        
        # Compute goal with respect to local reference frame (my prediction model works with map oriented with rover reference frame)
        gp = goal_xy - Q
        goal_xy_local[0] = np.cos(yaw)*gp[0] + np.sin(yaw)*gp[1]
        goal_xy_local[1] = np.cos(yaw)*gp[1] - np.sin(yaw)*gp[0]
        goal_dist = np.linalg.norm(Q-goal_xy)
        
        print()
        print("Waypoint Selection Global Coordinates")
        print("X: {0:.3f}m".format(goal_xy[0]))
        print("Y: {0:.3f}m".format(goal_xy[1]))
        print("Waypoint Rover Coordinates")
        print("X: {0:.3f}m".format(goal_xy_local[0]))
        print("Y: {0:.3f}m".format(goal_xy_local[1]))
        print("Waypoint Distance: {0:.3f}m".format(goal_dist))
        print()
        #time.sleep(5)
        
        print("Event Predictor Path Planner")
        # Local Path PLanner Using Network Model
        (r_rot, theta_rot), (r_fw1, theta_fw1), (r_fw2, theta_fw2) = local_planner(model, Zt, goal_xy_local)
        #time.sleep(5)
        
        if theta_rot is not None:
            print("Selected Action0: th {1:.1f}°".format(r_rot, theta_rot*180/math.pi))
        else:
            print("No Rotation Selected")
        if theta_fw1 is not None:
            print("Selected Action1: rad {0:.3f}m, th {1:.1f}°".format(r_fw1, theta_fw1*180/math.pi))
        else:
            print("Selected Action1: straight {0:.3f}m".format(r_fw1))
        if goal_dist > 1.3:
            if theta_fw2 is not None:
                print("Selected Action2: rad {0:.3f}m, th {1:.1f}°".format(r_fw2, theta_fw2*180/math.pi))
            else:
                print("Selected Action2: straight {0:.3f}m".format(r_fw2))
        #time.sleep(5)
        
        # Commanding Actions
        if goal_dist > 1.3:
            cmd_object.action_command(r_rot, theta_rot)
            cmd_object.action_command(r_fw1, theta_fw1)
            cmd_object.action_command(r_fw2, theta_fw2, goal=goal_xy)
        else:
            cmd_object.action_command(r_rot, theta_rot)
            cmd_object.action_command(r_fw1, theta_fw1, goal=goal_xy)
            
if __name__ == "__main__":
    rospy.init_node("Path_Planner_node", log_level=rospy.INFO)
    main(sys.argv)
