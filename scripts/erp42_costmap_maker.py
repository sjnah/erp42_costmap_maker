#!/bin/usr/env python3
import os

import numpy as np
import matplotlib.pyplot as plt
import cv2

import shapely.geometry as shp
import yaml
import utm

## file name  (check delimiter, it could be " ", "\t", ",")
filepath = './path/'

# filename1 = 'FMTC_official_L_v2.txt' # delimiter " " / offset ? m / gps_origin 37.36567790, 126.72499770 
# filename2 = 'FMTC_official_R_v2.txt'
filename1 = 'path_yonsei_220704_v1.txt' # delimiter " " / offset ? m / gps_origin 37.36567790, 126.72499770 
filename2 = 'path_yonsei_220704_v1.txt'

## Debug mode (if True, it show only map outline result)
debug_mode = False

## remap with new map origin
remap = False
origin_old = [37.36504871, 126.72418471] # LL coordinate (SIM)
origin_new = [37.36578294, 126.72539592] 

## Costmap configuration
output_name = 'map_yonsei_220703_v1'
map_resolution = 0.1 # meter
offset_length = 2.0 # meter (offset from extracted path)
boundary_size = 15 # meter (map boundary)
wallsize = 0.4 # meter (outline of racing track)



class CostmapMaker():
    def __init__(self):
        pass

    def loadPath(self):
        # Load path file
        lines_lane1 = np.loadtxt(filepath+filename1, dtype=str, delimiter="\t", unpack=False)
        lines_lane2 = np.loadtxt(filepath+filename2, dtype=str, delimiter="\t", unpack=False)
        # lines_lane1 = np.loadtxt(filepath+filename1, delimiter=" ", unpack=False)
        # lines_lane2 = np.loadtxt(filepath+filename2, delimiter=" ", unpack=False)
        waypoint_lane1 = np.array(lines_lane1)
        waypoint_lane2 = np.array(lines_lane2)

        # Remove z data
        waypoint_lane1 = waypoint_lane1[:, 0:2]
        waypoint_lane2 = waypoint_lane2[:, 0:2]

        print('[INFO] Waypoint File loaded.')

        return waypoint_lane1, waypoint_lane2

    def remap(self):
        # Change Global Coordinate System (LL2UTM)
        X_old, Y_old, zone, unknown = utm.from_latlon(origin_old[0], origin_old[1])
        X_new, Y_new, zone, unknown = utm.from_latlon(origin_new[0], origin_new[1])

        # Transformation
        self.poly_inward_1st = self.poly_inward_1st + [X_old - X_new, Y_old - Y_new]
        self.poly_inward_2nd = self.poly_inward_2nd + [X_old - X_new, Y_old - Y_new]
        self.poly_outward_1st = self.poly_outward_1st + [X_old - X_new, Y_old - Y_new]
        self.poly_outward_2nd = self.poly_outward_2nd + [X_old - X_new, Y_old - Y_new]
        self.waypoint_lane1 = self.waypoint_lane1 + [X_old - X_new, Y_old - Y_new]
        self.waypoint_lane2 = self.waypoint_lane2 + [X_old - X_new, Y_old - Y_new]

    def setMapsize(self):
        # Add Boundary to map
        self.max_x = np.max(self.poly_outward_2nd[:,0]) + boundary_size
        self.min_x = np.min(self.poly_outward_2nd[:,0]) - boundary_size
        self.max_y = np.max(self.poly_outward_2nd[:,1]) + boundary_size
        self.min_y = np.min(self.poly_outward_2nd[:,1]) - boundary_size

    def getOffset(self, waypoint_lane1_raw, waypoint_lane2_raw):
        # Create a Polygon from the nx2 array in `afpts`
        waypoint_lane1_shp = shp.Polygon(waypoint_lane1_raw)
        waypoint_lane2_shp = shp.Polygon(waypoint_lane2_raw)

        # Create offset airfoils, both inward and outward
        poly_inward_1st_shp = waypoint_lane1_shp.buffer(-offset_length)  # Inward offset
        poly_inward_2nd_shp = waypoint_lane1_shp.buffer(-offset_length-wallsize)  # Inward offset
        poly_outward_1st_shp = waypoint_lane2_shp.buffer(offset_length)  # Outward offset
        poly_outward_2nd_shp = waypoint_lane2_shp.buffer(offset_length+wallsize)  # Outward offset

        # Turn polygon points into numpy arrays for plotting
        self.waypoint_lane1 = np.array(waypoint_lane1_shp.exterior)
        self.waypoint_lane2 = np.array(waypoint_lane2_shp.exterior)
        self.poly_outward_1st = np.array(poly_outward_1st_shp.exterior)
        self.poly_outward_2nd = np.array(poly_outward_2nd_shp.exterior)
        self.poly_inward_1st = np.array(poly_inward_1st_shp.exterior)
        self.poly_inward_2nd = np.array(poly_inward_2nd_shp.exterior)

    def getCostmap(self):
        # Get costmap height and width
        height_meter = round(self.max_y) - round(self.min_y)
        width_meter = round(self.max_x) - round(self.min_x)
        height = int(height_meter/map_resolution)
        width = int(width_meter/map_resolution)

        
        print('[INFO] Map size [' + str(height_meter) + 'x' + str(width_meter) + '] (m), [' + str(height) + ' x ' + str(width) + '] (pixel)')

        # Make Empty costmap
        map = 255*np.ones((height, width))

        print('[INFO] Calculating costmap ...')

        total = height*width
        # Calculate Costmap
        for i in range(width):
            for j in range(height):
                if (debug_mode == False):
                    # Get current cell coordinate
                    cell_x = round(self.min_x) + i*map_resolution + 0.5*map_resolution
                    cell_y = round(self.max_y) - j*map_resolution - 0.5*map_resolution

                    # Check if the cell is inside the track
                    cost = self.getCost(cell_x, cell_y)

                    # Set cell occupancy value
                    map[j, i] = cost
                else:
                    # Debugging
                    map[j, i] = 0
            
            if (i%10 == 0):
                progress = int((i*j)/total*100)
                print('[Calculation] {0:.2f} % finished'.format(progress))

        print(map)

        # Set map origin
        origin_x = round(self.min_x) - 0
        origin_y = round(self.min_y) - 0
        origin_yaw = 0
        map_origin = [origin_x, origin_y, origin_yaw]
        map_origin = str(map_origin) + ' please remove single quote'

        plt.scatter(round(self.min_x), round(self.min_y), s=20, color='blue')
        plt.scatter(0,0, s=20, color='red')
        # plt.scatter(cell_x, cell_y, color='green')
        plt.plot(*self.poly_outward_1st.T, color='black')
        plt.plot(*self.poly_inward_1st.T, color='black')
        plt.plot(*self.waypoint_lane1.T, color='blue')
        plt.plot(*self.waypoint_lane2.T, color='blue')
        plt.axis('equal')
        plt.grid(True)
        plt.show()

        # Make map.yaml data
        yaml_data = dict(
            image = './' + output_name + '.pgm', # Path to the image file containing the occupancy data; can be absolute, or relative to the location of the YAML file
            resolution = map_resolution, # Resolution of the map, meters / pixel
            origin = map_origin, # The 2-D pose of the lower-left pixel in the map, as (x, y, yaw)
            occupied_thresh =  0.65, # (0~1) Pixels with occupancy probability greater than this threshold are considered completely occupied.
            free_thresh = 0.196, # (0~1)  Pixels with occupancy probability less than this threshold are considered completely free.
            negate = 1, # (0,1) Whether the white/black free/occupied semantics should be reversed (interpretation of thresholds is unaffected)
        )
        
        return map, yaml_data
        
    def getCost(self, x, y):
        # Build Point and Polgon
        point = shp.Point(x, y)
        polygon_inside_1st = shp.polygon.Polygon(self.poly_inward_1st)
        polygon_inside_2nd = shp.polygon.Polygon(self.poly_inward_2nd)
        polygon_outside_1st = shp.polygon.Polygon(self.poly_outward_1st)
        polygon_outside_2nd = shp.polygon.Polygon(self.poly_outward_2nd)

        # check if the cell is inside the track 
        inside_1st = polygon_inside_1st.contains(point) # bool
        inside_2nd = polygon_inside_2nd.contains(point) # bool
        outside_1st = polygon_outside_1st.contains(point) # bool
        outside_2nd = polygon_outside_2nd.contains(point) # bool

        if not outside_2nd:
            cost = 128
        elif not outside_1st and outside_2nd: # tra
            cost = 254
        elif not inside_1st and outside_1st: # track
            cost = 0
        elif not inside_2nd and inside_1st:
            cost = 254
        else: # inside_2nd
            cost = 128

        return cost

    def saveCostmap(self, map, yaml_data):
        # save costmap as pgm file
        plt.imsave('./costmap/' + output_name + '.png', map)
        # cv2.imwrite('./costmap/' + output_name + '.png', map)
        cv2.imwrite('./costmap/' + output_name + '.pgm', map)

        # save costmap yaml
        with open('./costmap/' + output_name + '.yaml', 'w') as outfile:
            yaml.dump(yaml_data, outfile, default_flow_style=False)

        print('[INFO] Finished. Check \'./costmap\' directory')

    def main(self):
        # load waypoint data
        waypoint_lane1, waypoint_lane2 = self.loadPath()

        # get offset path line
        self.getOffset(waypoint_lane1, waypoint_lane2)

        # remap with 2 gps origin point
        if remap == True:
            self.remap()

        # get costmap
        self.setMapsize()
        map, yaml_data = self.getCostmap()
        self.saveCostmap(map, yaml_data)
        
if __name__=="__main__":
    maker = CostmapMaker()
    maker.main()
