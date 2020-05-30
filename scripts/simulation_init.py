import numpy as np
import utils
import traversability_test as tt
import os

import math


map_path = "/home/marco/catkin_ws/src/mysim/models/terrain/materials/textures/"
path_dataset = "/home/marco/catkin_ws/src/mysim/mymaps/boxmet_Dataset.hdf5"
map_size = 8
discr = 0.0625
x0 = 0
y0 = 0
yaw0 = 0*math.pi/180
id_t = 0
display = True


plot_graphs = True
plot_cost_map = False

if not os.path.exists(map_path):
    os.makedirs(map_path)
            
def main():
    
    # Reading from dataset
    image_name = "{0:04d}.png".format(id_t)
    Z_pixel, map_height = utils.dataset_read(path_dataset, id_t, path_image = map_path + image_name)
    
    Z = Z_pixel.astype(np.float32)/255.0*map_height
    z0, roll0, pitch0 = utils.starting_pose(x0, y0, yaw0, Z, map_size = map_size)


    utils.modify_world_files(image_name, map_size, map_size, map_height,
                             x0, y0, z0, roll0, pitch0, yaw0)
    # Colormesh plotting
    if plot_graphs:
        utils.plot_colormesh(map_size, discr, Z, title='Global Map')
    
    # Static Traversability analysis cost of points around starting position
    costs = []
    cost_map = tt.Traversability_Map(map_size, discr)
    if plot_cost_map:
        cost_map.analysis(Z, plot=plot_cost_map)
    cost_map.single_point_analysis(Z, x0, y0)
    costs.append(cost_map.tot_point)
    cost_map.single_point_analysis(Z, x0+0.01, y0)
    costs.append(cost_map.tot_point)
    cost_map.single_point_analysis(Z, x0, y0+0.1)
    costs.append(cost_map.tot_point)
    cost_map.single_point_analysis(Z, x0+0.1, y0+0.1)
    costs.append(cost_map.tot_point)
    cost_map.single_point_analysis(Z, x0-0.1, y0)
    costs.append(cost_map.tot_point)
    cost_map.single_point_analysis(Z, x0, y0-0.1)
    costs.append(cost_map.tot_point)
    cost_map.single_point_analysis(Z, x0-0.1, y0-0.1)
    costs.append(cost_map.tot_point)
    cost_map.single_point_analysis(Z, x0+0.1, y0-0.1)
    costs.append(cost_map.tot_point)
    cost_map.single_point_analysis(Z, x0-0.1, y0+0.1)
    costs.append(cost_map.tot_point)
    costs = list(filter(None, costs)) 
    print("Static Initial Position Cost: " + str(np.array(costs).max()))
    print()
    
    
    
#    import subprocess
    # args = ["roslaunch", "mysim", "main.launch", 
    #         "x:={}".format(x0), "y:={}".format(y0), "z:={}".format(z0+0.9), 
    #         "roll:={}".format(-roll0), "pitch:={}".format(-pitch0), "yaw:={}".format(yaw0+math.pi),
    #         "gui:= {}".format(display)]
    # val = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
            
        
            

if __name__ == "__main__":
    main()

    
    
    
