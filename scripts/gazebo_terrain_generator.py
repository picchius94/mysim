from PIL import Image, ImageOps
import h5py
import sys
import numpy as np
import math

# Rover width in meters
width = 2
# Rover length in meters
length = 2.4
# Map size in meters (coordinates from -map_size/2 to map_size/2)
map_size = 9.6
# Cell discretization in meter
discr = 0.075
# Number of cells of one side of the square DEM
DEM_size = int(map_size/discr +1)

# Path Final HDF5 Dataset
path_ROS_dataset = '/home/marco/Project_Simulator_Regression/Dataset_Generation/ROS_Dataset/ROS_dataset.hdf5'

def main(argv):
    print()
    print()
    if len(argv) ==1:
        print("Usage: gazebo_terrain_generator.py <terrain ID>")
        return 0
    elif len(argv) > 2:
        print("Too many parameters")
        print("Usage: gazebo_terrain_generator.py <terrain ID>")
        return 0
    
    print("Loading Terrain {}".format(argv[1]))
    with h5py.File(path_ROS_dataset, 'r') as f:
        i = int(argv[1])
        if i>len(f['img'])-1:
            print("ID is out of range. Max Value should be {}".format(len(f['img'])-1))
            return 0
        # Image
        img = Image.new('L', (DEM_size,DEM_size))
        img.putdata((f['img'][i]).reshape(DEM_size*DEM_size))
        im_flip = ImageOps.flip(img)
        im_flip.save('/home/marco/catkin_ws/src/mysim/models/terrain/materials/textures/terrain.png')
        
        # Minz
        minz = f['minz'][i]
        print("Minz = {}".format(minz))
        # Interval
        interval = f['interval'][i]
        print("Interval = {}".format(interval))
        # Wheels Elevation
        w_elevation = f['wheels'][i]
        print("Wheels Elevation = {}".format(w_elevation))
        
    # Compute z axis based on wheels position
    points_x = [-width/2, -width/2, width, width]
    points_y = [length/2, -length/2, length/2, -length/2]
    points_z = w_elevation
    tmp_A = []
    tmp_b = []
    for i in range(len(points_x)):
        tmp_A.append([points_x[i], points_y[i], 1])
        tmp_b.append(points_z[i])
    b = np.matrix(tmp_b).T
    A = np.matrix(tmp_A)
    fit = (A.T * A).I * A.T * b  # i.e the 3 parameters a,b,c of the plane ax + by + c = z
    print(fit)
    a = fit[0].item()
    b = fit[1].item()    
    # Compute Vectors in x and y directions
    z_vect = np.array([-a,-b,1])/np.linalg.norm([a,b,1])
    x_vect_x = math.sqrt(1/(1+(z_vect[0]/z_vect[2])**2))
    x_vect_z = -x_vect_x*z_vect[0]/z_vect[2]
    x_vect = np.array([x_vect_x, 0, x_vect_z])
    y_vect_y = math.sqrt(1/(1+(z_vect[1]/z_vect[2])**2))
    y_vect_z = -y_vect_y*z_vect[1]/z_vect[2]
    y_vect = np.array([0, y_vect_y, y_vect_z])
    # Compute roll and pitch angles
    roll = np.arccos(np.dot(np.array([1,0,0]),x_vect))
    pitch = np.arccos(np.dot(np.array([0,1,0]),y_vect))
    if x_vect_z<0:
        roll = -roll
    if y_vect_z<0:
        pitch = -pitch
    
    print("Roll angle: {} ({}°)".format(roll, roll*180/math.pi))
    print("Pitch angle: {} ({}°)".format(pitch, pitch*180/math.pi))
            
    with open("/home/marco/catkin_ws/src/mysim/models/terrain/model.config", 'w+') as f:
        f.write('<?xml version="1.0"?>\n')
        f.write('<model>\n')
        f.write('<name>terrain</name>\n')
        f.write('<version>1.0</version>\n')
        f.write('<sdf version="1.5">model.sdf</sdf>\n')
        f.write('<author>\n')
        f.write('<name>M. Visca</name>\n')
        f.write('<email>m.visca@surrey.ac.uk</email>\n')
        f.write('</author>\n')
        f.write('<description>\n')
        f.write('Terrain Generated from Benchmark Dataset with OpenSimplex Algorithm\n')
        f.write('</description>\n')
        f.write('</model>\n')
        
    with open("/home/marco/catkin_ws/src/mysim/models/terrain/model.sdf", 'w+') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<sdf version="1.5">\n')
        f.write('<model name="terrain">\n')
        f.write('<static>true</static>\n')
        f.write('<link name="link">\n')
        f.write('<collision name="collision">\n')
        f.write('<geometry>\n')
        f.write('<heightmap>\n')
        f.write('<uri>model://terrain/materials/textures/terrain.png</uri>\n')
        f.write('<size>{0} {1} {2:.2f}</size>\n'.format(map_size, map_size, interval))
        f.write('<pos>0 0 {}</pos>\n'.format(0))
        f.write('</heightmap>\n')
        f.write('</geometry>\n')
        f.write('</collision>\n')
        f.write('<visual name="visual_abcedf">\n')
        f.write('<geometry>\n')
        f.write('<heightmap>\n')
        f.write('<use_terrain_paging>false</use_terrain_paging>\n')
        f.write('<texture>\n')
        f.write('<diffuse>file://media/materials/textures/dirt_diffusespecular.png</diffuse>\n')
        f.write('<normal>file://media/materials/textures/flat_normal.png</normal>\n')
        f.write('<size>1</size>\n')
        f.write('</texture>\n')
        f.write('<texture>\n')
        f.write('<diffuse>file://media/materials/textures/grass_diffusespecular.png</diffuse>\n')
        f.write('<normal>file://media/materials/textures/flat_normal.png</normal>\n')
        f.write('<size>1</size>\n')
        f.write('</texture>\n')
        f.write('<texture>\n')
        f.write('<diffuse>file://media/materials/textures/fungus_diffusespecular.png</diffuse>\n')
        f.write('<normal>file://media/materials/textures/flat_normal.png</normal>\n')
        f.write('<size>1</size>\n')
        f.write('</texture>\n')
        f.write('<blend>\n')
        f.write('<min_height>0.01</min_height>\n')
        f.write('<fade_dist>0.0625</fade_dist>\n')
        f.write('</blend>\n')
        f.write('<blend>\n')
        f.write('<min_height>0.01</min_height>\n')
        f.write('<fade_dist>0.0625</fade_dist>\n')
        f.write('</blend>\n')
        f.write('<uri>model://terrain/materials/textures/terrain.png</uri>\n')
        f.write('<size>{0} {1} {2:.2f}</size>\n'.format(map_size, map_size, interval))
        f.write('<pos>0 0 {}</pos>'.format(0))
        f.write('</heightmap>\n')
        f.write('</geometry>\n')
        f.write('</visual>\n')
        f.write('</link>\n')
        f.write('</model>\n')
        f.write('</sdf>\n')
    
    with open("/home/marco/catkin_ws/src/mysim/worlds/terrain.world", 'w+') as f:
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
        
    with open("/home/marco/catkin_ws/src/mysim/launch/main_terrain.launch", 'w+') as f:
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
        f.write('<arg name="x" default="0.0" />\n')
        f.write('<arg name="y" default="0.0" />\n')
        f.write('<arg name="z" default="{}" />\n'.format(-minz+0.9))
        f.write('<arg name="roll" default="{}"/>\n'.format(-roll))
        f.write('<arg name="pitch" default="{}"/>\n'.format(-pitch))
        f.write('<arg name="yaw" default="{}" />\n'.format(math.pi))
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
        f.write('args="-urdf -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg roll) -P $(arg pitch) -Y $(arg yaw) -model curiosity_mars_rover -param robot_description"/>\n')
        f.write('<!-- Robot Joints Controller -->\n')
        f.write('<include file="$(find mysim)/launch/rover_control.launch"/>\n')
        f.write('<!-- Robot System APIs -->\n')
        f.write('<include file="$(find mysim)/launch/rover_systems.launch"/>\n')
        f.write('<!-- Open Rviz -->\n')
        f.write('<node name="rviz" pkg="rviz" type="rviz" args="-d $(find mysim)/rviz/rover.rviz"/>\n')
        f.write('</launch>\n')
 

if __name__ == "__main__":
    main(sys.argv)