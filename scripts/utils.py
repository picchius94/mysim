import h5py
import numpy as np
import math
from PIL import Image, ImageOps
import matplotlib.pyplot as plt
import os

def modify_world_files(image_name, map_size_x, map_size_y, map_height,
 x0, y0, z0, roll0, pitch0, yaw0):
    model_file1 = "/home/marco/catkin_ws/src/mysim/models/terrain/model.config"
    model_file2 = "/home/marco/catkin_ws/src/mysim/models/terrain/model.sdf"
    world_file = "/home/marco/catkin_ws/src/mysim/worlds/terrain.world"
    launch_file = "/home/marco/catkin_ws/src/mysim/launch/main.launch"

#    if not os.path.exists(model_file1):
    with open(model_file1, 'w+') as f:
        f.write('<?xml version="1.0"?>\n')
        f.write('<model>\n')
        f.write('<name>terrain</name>\n')
        f.write('<version>1.0</version>\n')
        f.write('<sdf version="1.6">model.sdf</sdf>\n')
        f.write('<author>\n')
        f.write('<name>M. Visca</name>\n')
        f.write('<email>m.visca@surrey.ac.uk</email>\n')
        f.write('</author>\n')
        f.write('<description>\n')
        f.write('Terrain Generated from Benchmark Dataset\n')
        f.write('</description>\n')
        f.write('</model>\n')
        
    with open(model_file2, 'w+') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<sdf version="1.6">\n')
        f.write('<model name="terrain">\n')
        f.write('<static>true</static>\n')
        f.write('<link name="link">\n')
        f.write('<collision name="collision">\n')
        f.write('<surface>\n')
        f.write('<friction>\n')
        f.write('<ode>\n')
        f.write('<mu>0.7</mu>\n')
        f.write('</ode>\n')
        f.write('</friction>\n')
        f.write('<contact>\n')
        f.write('<ode>\n')
        f.write('<kp>1000000</kp>\n')
        f.write('<kd>100</kd>\n')
        f.write('<min_depth>0.00</min_depth>\n')
        f.write('</ode>\n')
        f.write('</contact>\n')
        f.write('</surface>\n')
        f.write('<geometry>\n')
        f.write('<heightmap>\n')
        f.write('<uri>model://terrain/materials/textures/{}</uri>\n'.format(image_name))
        f.write('<size>{0} {1} {2:.2f}</size>\n'.format(map_size_x, map_size_y, map_height))
        f.write('<pos>0 0 0</pos>\n')
        f.write('</heightmap>\n')
        f.write('</geometry>\n')
        f.write('</collision>\n')
        f.write('<visual name="visual">\n')
        f.write('<geometry>\n')
        f.write('<heightmap>\n')
        f.write('<texture>\n')
        f.write('<diffuse>model://terrain/materials/textures/surface.jpg</diffuse>\n')
        f.write('<normal>file://media/materials/textures/flat_normal.png</normal>\n')
        f.write('<size>1</size>\n')
        f.write('</texture>\n')
        f.write('<uri>model://terrain/materials/textures/{}</uri>\n'.format(image_name))
        f.write('<size>{0:.2f} {1:.2f} {2:.2f}</size>\n'.format(map_size_x, map_size_y, map_height))
        f.write('<pos>0 0 0</pos>')
        f.write('</heightmap>\n')
        f.write('</geometry>\n')
        f.write('</visual>\n')
        f.write('</link>\n')
        f.write('</model>\n')
        f.write('</sdf>\n')
    
#    if not os.path.exists(world_file):
    with open(world_file, 'w+') as f:
        f.write('<sdf version="1.5">\n')
        f.write('<world name="terrain">\n')
        f.write('<!-- Martian Gravity-->\n')
        f.write("<physics type='ode'>\n")
        f.write('<max_step_size>0.001</max_step_size>\n')
        f.write('<real_time_factor>1</real_time_factor>\n')
        f.write('<real_time_update_rate>1000</real_time_update_rate>\n')
        f.write('<gravity>0 0 -3.711</gravity>\n')
        f.write('</physics>\n')
        f.write('<include>\n')
        f.write('<uri>model://sun</uri>\n')
        f.write('</include>\n')
        f.write('<include>\n')
        f.write('<uri>model://terrain</uri>\n')
        f.write('<name>terrain</name>\n')
        f.write('<pose>0 0 0 0 0 0</pose>\n')
        f.write('</include>\n')
        f.write('</world>\n')
        f.write('</sdf>\n')

    with open(launch_file, 'w+') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<launch>\n')
        f.write('<!-- World Parameters-->\n')
        f.write('<arg name="world" default="terrain"/>\n')
        f.write('<arg name="paused" default="true"/>\n')
        f.write('<arg name="use_sim_time" default="true"/>\n')
        f.write('<arg name="gui" default="true"/>\n')
        f.write('<arg name="headless" default="false"/>\n')
        f.write('<arg name="debug" default="false"/>\n')
        f.write('<!-- Robot Parameters-->\n')
        f.write('<arg name="model" default="$(find mysim)/urdf/curiosity_mars_rover.xacro"/>\n')
        f.write('<arg name="x" default="{}" />\n'.format(x0))
        f.write('<arg name="y" default="{}" />\n'.format(y0))
        f.write('<arg name="z" default="{}" />\n'.format(z0+0.9))
        f.write('<arg name="roll" default="{}"/>\n'.format(-roll0))
        f.write('<arg name="pitch" default="{}"/>\n'.format(pitch0))
        f.write('<arg name="yaw" default="{}" />\n'.format(yaw0+math.pi))
        f.write('<!-- Launch World-->\n')
        f.write('<include file="$(find gazebo_ros)/launch/empty_world.launch">\n')
        f.write('<arg name="world_name" value="$(find mysim)/worlds/$(arg world).world"/>\n')
        f.write('<arg name="debug" value="$(arg debug)" />\n')
        f.write('<arg name="gui" value="$(arg gui)"/>\n')
        f.write('<arg name="paused" value="$(arg paused)"/>\n')
        f.write('<arg name="use_sim_time" value="$(arg use_sim_time)"/>\n')
        f.write('<arg name="headless" value="$(arg headless)"/>\n')
        f.write('</include>\n')
        f.write('<!-- Spawn Robot-->\n')
        f.write('<param name="robot_description" command="$(find xacro)/xacro --inorder $(arg model)" />\n')
        f.write('<node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"\n')
        f.write('args="-urdf -x $(arg x) -y $(arg y) -z $(arg z) -P $(arg roll) -R $(arg pitch) -Y $(arg yaw) -model curiosity_mars_rover -param robot_description"/>\n')
        f.write('<!-- Robot Joints Controller -->\n')
        f.write('<include file="$(find mysim)/launch/rover_control.launch"/>\n')
        f.write('<!-- Robot System APIs -->\n')
        f.write('<include file="$(find mysim)/launch/rover_systems.launch"/>\n')
        f.write('<!-- Open Rviz -->\n')
        f.write('<node name="rviz" pkg="rviz" type="rviz" args="-d $(find mysim)/rviz/rover.rviz"/>\n')
        f.write('</launch>\n')

def plot_colormesh(map_size, discr, Z, title = "Map"):
    DEM_size = int(map_size/discr + 1)
    x = np.linspace(-map_size/2,map_size/2,num=DEM_size)
    y = np.linspace(-map_size/2,map_size/2,num=DEM_size)
    X,Y = np.meshgrid(x,y)
    fig = plt.figure(figsize=(13,13))
    ax = plt.gca()
    ax.set_aspect("equal")
    ax.set_title(title, fontsize = 35)
    ax.tick_params(labelsize=40)
    ax.set_xlabel("X [m]", fontsize=35)
    ax.set_ylabel("Y [m]", fontsize=35)
    im = ax.pcolormesh(X,Y,Z, cmap = 'viridis')
    cb = fig.colorbar(im, ax=ax)
    cb.ax.tick_params(labelsize=40)
    cb.set_label("Z [m]", fontsize=35, rotation=90, va="bottom", labelpad=32)
    plt.show()


def starting_pose(x0,y0,yaw0,Z,length = 2.4, width = 2, map_size = 8, discr= 0.0625):
    x1_loc = -width/2
    y1_loc = length/2
    x2_loc = width/2
    y2_loc = length/2
    x3_loc = -width/2
    y3_loc = -length/2
    x4_loc = width/2
    y4_loc = -length/2
    # # Compute Wheels Position
    pcenter = np.array([[x0],[y0]])
    rcenter = np.matrix(((np.cos(yaw0), -np.sin(yaw0)), (np.sin(yaw0), np.cos(yaw0))))
    p1=pcenter+rcenter*np.array([[x1_loc],[y1_loc]])
    x1=p1[0].item()
    y1=p1[1].item()
    p2=pcenter+rcenter*np.array([[x2_loc],[y2_loc]])
    x2=p2[0].item()
    y2=p2[1].item()
    p3=pcenter+rcenter*np.array([[x3_loc],[y3_loc]])
    x3=p3[0].item()
    y3=p3[1].item()
    p4=pcenter+rcenter*np.array([[x4_loc],[y4_loc]])
    x4=p4[0].item()
    y4=p4[1].item()
    
    DEM_size = int(map_size/discr +1)
    z_intorno = []
    for xi,yi in zip([x1,x2,x3,x4],[y1,y2,y3,y4]):
        ixl = [(np.floor((xi-(-map_size/2))/discr)).astype(int)]
        iyl = [(np.floor((yi-(-map_size/2))/discr)).astype(int)]
        
        # Following four ifs must be removed (they should never happen)
        if ixl[0] > DEM_size -1:
            ixl[0] = DEM_size -1
        if iyl[0] > DEM_size -1:
            iyl[0] = DEM_size -1
        if ixl[0] < 0:
            ixl[0] = 0
        if iyl[0] < 0:
            iyl[0] = 0
            
        if ixl[0] > 0:
            ixl.append(ixl[0]-1)
        if ixl[0] < DEM_size -1:
            ixl.append(ixl[0]+1)
        if iyl[0] > 0:
            iyl.append(iyl[0]-1)
        if iyl[0] < DEM_size -1:
            iyl.append(iyl[0]+1)
        zl = []
        for ix in ixl:
            for iy in iyl:
                zl.append(Z[iy,ix])
        z_intorno.append(np.array(zl).max())
    zt = np.array(z_intorno)
    
    ## Method1 points to fit
    # z1 = Z[(np.floor((y1-(-map_size/2))/discr)).astype(int),(np.floor((x1-(-map_size/2))/discr)).astype(int)]
    # z2 = Z[(np.floor((y2-(-map_size/2))/discr)).astype(int),(np.floor((x2-(-map_size/2))/discr)).astype(int)]
    # z3 = Z[(np.floor((y3-(-map_size/2))/discr)).astype(int),(np.floor((x3-(-map_size/2))/discr)).astype(int)]
    # z4 = Z[(np.floor((y4-(-map_size/2))/discr)).astype(int),(np.floor((x4-(-map_size/2))/discr)).astype(int)]
    # points_x = [x1_loc, x2_loc, x3_loc, x4_loc]
    # points_y = [y1_loc, y2_loc, y3_loc, y4_loc]
    # points_z = [z1, z2, z3, z4]
    
    # Method2 points to fit
    # eps_length = 0.355
    # eps_width = 0.325
    # length += eps_length
    # width += eps_width
    # DEM_size = int(map_size/discr +1)
    # x = np.linspace(-map_size/2,map_size/2,num=DEM_size)
    # y = np.linspace(-map_size/2,map_size/2,num=DEM_size)
    # X, Y = np.meshgrid(x,y)
    # X_local = (X - x0)*np.cos(-yaw0) - (Y - y0)*np.sin(-yaw0)
    # Y_local = (X - x0)*np.sin(-yaw0) + (Y - y0)*np.cos(-yaw0)
    # mask1 = abs(X_local) < width/2
    # mask2 = abs(Y_local) < length/2
    # mask = mask1*mask2
    # points_z = Z[mask]
    # points_x = X_local[mask]
    # points_y= Y_local[mask]
    
    ## Method3 points to fit
    points_x = [x1_loc, x2_loc, x3_loc, x4_loc]
    points_y = [y1_loc, y2_loc, y3_loc, y4_loc]
    points_z = z_intorno
    
    # Fit Plane which better approximates the points
    tmp_A = []
    tmp_b = []
    for i in range(len(points_x)):
        tmp_A.append([points_x[i], points_y[i], 1])
        tmp_b.append(points_z[i])
    b = np.matrix(tmp_b).T
    A = np.matrix(tmp_A)
    fit = (A.T * A).I * A.T * b 
    
    a = fit[0].item()
    b = fit[1].item()
    c = fit[2].item()
    # Compute Vectors in x and y directions
    z_vect = np.array([-a,-b,1])/np.linalg.norm([a,b,1])
    x_vect_x = math.sqrt(1/(1+(z_vect[0]/z_vect[2])**2))
    x_vect_z = -x_vect_x*z_vect[0]/z_vect[2]
    x_vect = np.array([x_vect_x, 0, x_vect_z])
    y_vect_y = math.sqrt(1/(1+(z_vect[1]/z_vect[2])**2))
    y_vect_z = -y_vect_y*z_vect[1]/z_vect[2]
    y_vect = np.array([0, y_vect_y, y_vect_z])
    # Compute roll and pitch angles
    roll0 = round(-np.arccos(np.dot(np.array([1,0,0]),x_vect)),2)
    pitch0 = round(-np.arccos(np.dot(np.array([0,1,0]),y_vect)),2)
    if x_vect_z<0:
        roll0 = -roll0
    if y_vect_z<0:
        pitch0 = -pitch0
    
    zw1 = a*x1_loc + b*y1_loc + c
    zw2 = a*x2_loc + b*y2_loc + c
    zw3 = a*x3_loc + b*y3_loc + c
    zw4 = a*x4_loc + b*y4_loc + c
    zw = np.array([zw1,zw2,zw3,zw4])
    
    dz = zt-zw
    z0 = round(c + dz.max(),3)
    
    return z0, roll0, pitch0

        
def dataset_read(path_dataset, id, image = True, path_image = None):
    with h5py.File(path_dataset, 'r') as f:
        if f["len"][0] <= id:
            print("Data id is out of range. Max id: {}".format(f["len"][0]-1))
            return None, None, None, None, None
        
        Z_pixel = f["img"][id]
        
        if image:
            # Generate Image
            im = Image.new('L', (Z_pixel.shape[0],Z_pixel.shape[1]))
            im.putdata(Z_pixel.reshape(Z_pixel.shape[0]*Z_pixel.shape[1]))
            im = ImageOps.flip(im)
            if path_image is None:
                im.save("{0:04d}.png".format(id))
            else:
                im.save("{}".format(path_image))
        
        height = f["height"][id]
        
        return Z_pixel, height
    
def dataset_len(path_dataset):
    with h5py.File(path_dataset, 'r') as f:
        return f["len"][0]
    
        
