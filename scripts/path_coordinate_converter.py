#!/bin/usr/env python3
import sys
print(sys.version_info[0])
print(sys.path)
import os

import numpy as np
import yaml
from utm import from_latlon

## file name
filepath = './path/'
# filename1 = 'FMTC_official_L.txt' # delimiter " " / offset ? m / gps_origin 37.36567790, 126.72499770 (정지선앞)
# filename2 = 'FMTC_official_R.txt'
# filename1 = 'FMTC_centerline_GP.txt' # delimiter "," / offset 2.5 m / gps_origin 37.36578294, 126.72539592 (차고앞)
# filename2 = 'FMTC_centerline_GP.txt'
# filename1 = 'FMTC_simulator_1.txt' # delimiter "\t" / offset 1.25 m / gps_origin ??? (중앙길 어딘가)
# filename2 = 'FMTC_simulator_2.txt'
# filename1 = 'yonsei_3.txt'
# filename2 = 'yonsei_3.txt'
# filename1 = 'FMTC_sim_01.txt'
# filename2 = 'FMTC_sim_01.txt'
filename1 = 'FMTC_official_L_raw.txt' # delimiter " " / offset ? m / gps_origin 37.36567790, 126.72499770 (정지선앞)
filename2 = 'FMTC_official_R_raw.txt'

## remap with new map origin
remap = True
# origin_old = [37.36567790, 126.72499770] # LL coordinate (StopLine)
# origin_new = [37.36578294, 126.72539592] # LL coordinate (Cargo)
# origin_old = [37.36578294, 126.72539592] # LL coordinate (Cargo)
# origin_new = [37.36172262, 126.72126302] # LL coordinate (SIM)
origin_old = [37.36578294, 126.72539592] # LL coordinate (Cargo)
origin_new = [37.36504871398734, 126.72418471076539] # LL coordinate (SIM_old)

## Costmap configuration
output_name1 = 'FMTC_official_L_sim.txt'
output_name2 = 'FMTC_official_R_sim.txt'

## Debug mode
debug_mode = True

class PathConverter():
    def __init__(self):
        pass

    def loadPath(self):
        # Load path file
        # lines_lane1 = np.loadtxt(filepath+filename1, dtype=float, delimiter="\t", unpack=False)
        # lines_lane2 = np.loadtxt(filepath+filename2, dtype=float, delimiter="\t", unpack=False)
        lines_lane1 = np.loadtxt(filepath+filename1, delimiter=",", unpack=False)
        lines_lane2 = np.loadtxt(filepath+filename2, delimiter=",", unpack=False)
        waypoint_lane1 = np.array(lines_lane1)
        waypoint_lane2 = np.array(lines_lane2)

        # Remove z data
        self.waypoint_lane1 = waypoint_lane1
        self.waypoint_lane2 = waypoint_lane2

        print('[INFO] Waypoint File loaded.')

    def remap(self):
        # Change Global Coordinate System (LL2UTM)
        X_old, Y_old, zone, unknown = from_latlon(origin_old[0], origin_old[1])
        X_new, Y_new, zone, unknown = from_latlon(origin_new[0], origin_new[1])

        # Transformation
        diff_lane = np.array([X_old - X_new, Y_old - Y_new, 0])
        self.waypoint_lane1 = self.waypoint_lane1 + diff_lane
        self.waypoint_lane2 = self.waypoint_lane2 + diff_lane

    def savePath(self):
        # save costmap yaml
        with open(filepath+output_name1, newline='', mode='w') as csvfile:
            for x, y, psi in self.waypoint_lane1:
                csvfile.writelines(str(x) + '\t' + str(y) + '\t' + str(psi) + '\n')

        with open(filepath+output_name2, newline='', mode='w') as csvfile:
            for x, y, psi in self.waypoint_lane2:
                csvfile.writelines(str(x) + '\t' + str(y) + '\t' + str(psi) + '\n')

        print('[INFO] Finished. Check \'./costmap\' directory')

    def main(self):
        # load waypoint data
        self.loadPath()

        # remap with 2 gps origin point
        self.remap()

        # get costmap
        self.savePath()
        
if __name__=="__main__":
    maker = PathConverter()
    maker.main()