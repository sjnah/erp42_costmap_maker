#!/bin/usr/env python
import os

import numpy as np
import matplotlib.pyplot as plt

import shapely.geometry as shp
import yaml
import utm

# file name
filepath = './path/'
# filename1 = 'FMTC_official_L.txt' # delimiter " " / offset ? m / gps_origin 37.36567790, 126.72499770
# filename2 = 'FMTC_official_R.txt'
# filename1 = 'FMTC_centerline_GP.txt' # delimiter "," / offset 2.5 m / gps_origin 37.36578294, 126.72539592
# filename2 = 'FMTC_centerline_GP.txt'
filename1 = 'FMTC_simulator_1.txt' # delimiter "\t" / offset 1.25 m / gps_origin ???
filename2 = 'FMTC_simulator_2.txt'

# remap with new map origin
remap = False
origin_old = [37.36567790, 126.72499770] # LL coordinate
origin_new = [37.36578294, 126.72539592] # LL coordinate

# Costmap configuration
output_name = 'map_0_1_stopline_'
map_resolution = 0.1 # meter
offset_length = 1.25 # meter
boundary_size = 15 # meter

class CostmapMaker():
    def __init__(self):
        pass

    def loadPath(self):
        # Load path file
        lines_lane1 = np.loadtxt(filepath+filename1, dtype=str, delimiter="\t", unpack=False)
        lines_lane2 = np.loadtxt(filepath+filename2, dtype=str, delimiter="\t", unpack=False)
        waypoint_lane1 = np.array(lines_lane1)
        waypoint_lane2 = np.array(lines_lane2)

        # Remove z data
        waypoint_lane1 = waypoint_lane1[:, 0:2]
        waypoint_lane2 = waypoint_lane2[:, 0:2]

        print('[INFO] Waypoint File loaded.')

        return waypoint_lane1, waypoint_lane2

    def remap(self, poly_outward_before, poly_inward_before):
        # Change Global Coordinate System (LL2UTM)
        X_old, Y_old, zone, unknown = utm.from_latlon(origin_old[0], origin_old[1])
        X_new, Y_new, zone, unknown = utm.from_latlon(origin_new[0], origin_new[1])

        # Transformation
        poly_inward_after = poly_inward_before + [X_old - X_new, Y_old - Y_new]
        poly_outward_after = poly_outward_before + [X_old - X_new, Y_old - Y_new]
        self.waypoint_lane1 = self.waypoint_lane1 + [X_old - X_new, Y_old - Y_new]
        self.waypoint_lane2 = self.waypoint_lane2 + [X_old - X_new, Y_old - Y_new]

        return poly_outward_after, poly_inward_after

    def setMapsize(self, poly_outward):
        # Add Boundary to map
        self.max_x = np.max(poly_outward[:,0]) + boundary_size
        self.min_x = np.min(poly_outward[:,0]) - boundary_size
        self.max_y = np.max(poly_outward[:,1]) + boundary_size
        self.min_y = np.min(poly_outward[:,1]) - boundary_size

    def getOffset(self, waypoint_lane1_raw, waypoint_lane2_raw):
        # Create a Polygon from the nx2 array in `afpts`
        waypoint_lane1_shp = shp.Polygon(waypoint_lane1_raw)
        waypoint_lane2_shp = shp.Polygon(waypoint_lane2_raw)

        # Create offset airfoils, both inward and outward
        poly_inward_shp = waypoint_lane1_shp.buffer(-offset_length)  # Inward offset
        poly_outward_shp = waypoint_lane2_shp.buffer(offset_length)  # Outward offset

        # Turn polygon points into numpy arrays for plotting
        self.waypoint_lane1 = np.array(waypoint_lane1_shp.exterior)
        self.waypoint_lane2 = np.array(waypoint_lane2_shp.exterior)
        poly_outward = np.array(poly_outward_shp.exterior)
        poly_inward = np.array(poly_inward_shp.exterior)

        return poly_outward, poly_inward

    def getCostmap(self, poly_outward, poly_inward):
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
                # # Get current cell coordinate
                # cell_x = round(self.min_x) + i*map_resolution + 0.5*map_resolution
                # cell_y = round(self.max_y) - j*map_resolution - 0.5*map_resolution

                # # Check if the cell is inside the track
                # cost = self.getCost(cell_x, cell_y, poly_outward, poly_inward)

                # # Set cell occupancy value
                # map[j, i] = cost

                # Debugging
                map[j, i] = 0
            
            if (i%10 == 0):
                progress = int((i*j)/total*100)
                print('[Calculation] {0:.2f} % finished'.format(progress))

        print(map)

        # Set map origin
        origin_x = 0 - round(self.min_x)
        origin_y = 0 - round(self.min_y)
        origin_yaw = 0
        map_origin = [origin_x, origin_y, origin_yaw]
        map_origin = str(map_origin) + ' please remove single quote'

        plt.scatter(round(self.min_x), round(self.min_y), s=20, color='blue')
        plt.scatter(0,0, s=20, color='red')
        plt.plot(*poly_outward.T, color='black')
        plt.plot(*poly_inward.T, color='black')
        plt.plot(*self.waypoint_lane1.T, color='blue')
        plt.plot(*self.waypoint_lane2.T, color='blue')
        plt.axis('equal')
        plt.grid(True)
        plt.show()

        # Make map.yaml data
        yaml_data = dict(
            image = 'USERPATH/' + output_name + '.pgm', # Path to the image file containing the occupancy data; can be absolute, or relative to the location of the YAML file
            resolution = map_resolution, # Resolution of the map, meters / pixel
            origin = map_origin, # The 2-D pose of the lower-left pixel in the map, as (x, y, yaw)
            occupied_thresh =  0.65, # (0~1) Pixels with occupancy probability greater than this threshold are considered completely occupied.
            free_thresh = 0.196, # (0~1)  Pixels with occupancy probability less than this threshold are considered completely free.
            negate = 0, # (0,1) Whether the white/black free/occupied semantics should be reversed (interpretation of thresholds is unaffected)
        )
        
        return map, yaml_data
        
    def getCost(self, x, y, poly_outward, poly_inward):
        # Build Point and Polgon
        point = shp.Point(x, y)
        polygon_inside = shp.polygon.Polygon(poly_inward)
        polygon_outside = shp.polygon.Polygon(poly_outward)

        # check if the cell is inside the track 
        inside = polygon_inside.contains(point) # bool
        outside = polygon_outside.contains(point) # bool

        if inside and outside:
            cost = 254
            return cost
        elif ~inside and outside:
            cost = 1
            return cost
        else: # ~inside and ~outside
            cost = 254
            return cost

    def saveCostmap(self, map, yaml_data):
        # save costmap as pgm file
        plt.imsave('./costmap/' + output_name + '_temp.png', map)
        plt.imsave('./costmap/' + output_name + '.png', map)
        os.rename('./costmap/' + output_name + '_temp.png', './costmap/' + output_name + '.pgm')

        # save costmap yaml
        with open('./costmap/' + output_name + '.yaml', 'w') as outfile:
            yaml.dump(yaml_data, outfile, default_flow_style=False)

        print('[INFO] Finished. Check \'./costmap\' directory')

    def main(self):
        # load waypoint data
        waypoint_lane1, waypoint_lane2 = self.loadPath()

        # get offset path line
        poly_outward, poly_inward = self.getOffset(waypoint_lane1, waypoint_lane2)

        # remap with 2 gps origin point
        if remap == True:
            poly_outward, poly_inward = self.remap(poly_outward, poly_inward)

        # get costmap
        self.setMapsize(poly_outward)
        map, yaml_data = self.getCostmap(poly_outward, poly_inward)
        self.saveCostmap(map, yaml_data)
        
if __name__=="__main__":
    maker = CostmapMaker()
    maker.main()