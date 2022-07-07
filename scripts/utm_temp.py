#!/bin/usr/env python
import os

import numpy as np
import utm

## file name
filepath = './path/'
filename1 = 'FMTC_lane_1.txt' # delimiter " " / offset ? m / gps_origin 37.36567790, 126.72499770 (정지선앞)
filename2 = 'FMTC_lane_2.txt'

def getLATLON(gps_current, utm_current):
    # Change Global Coordinate System (LL2UTM)
    X_current, Y_current, zone, unknown = utm.from_latlon(gps_current[0], gps_current[1])

    utm_zone = zone # korea
    print(zone)
    latitude, longitude = utm.to_latlon(X_current - utm_current[0], Y_current - utm_current[1], utm_zone, unknown)

    print(latitude) # Morai simulator - FMTC Map gps origin
    print(longitude) # Morai simulator - FMTC Map gps origi

def getUTM(gps_current):
    # Change Global Coordinate System (LL2UTM)
    X_current, Y_current, zone, unknown = utm.from_latlon(gps_current[0], gps_current[1])

    print(X_current) # Morai simulator - FMTC Map gps origin
    print(Y_current) # Morai simulator - FMTC Map gps origi

def loadPath():
    # Load path file
    lines_lane1 = np.loadtxt(filepath+filename1, delimiter="\t", unpack=False)
    lines_lane2 = np.loadtxt(filepath+filename2, delimiter="\t", unpack=False)
    waypoint_lane1 = np.array(lines_lane1)
    waypoint_lane2 = np.array(lines_lane2)

    # Remove z data
    waypoint_lane1 = waypoint_lane1[:, 0:2]
    waypoint_lane2 = waypoint_lane2[:, 0:2]

    print('[INFO] Waypoint File loaded.')

    return waypoint_lane1, waypoint_lane2

def transferMap(lane1_old, lane2_old, gps_origin):
    X_old, Y_old, zone, unknown = utm.from_latlon(gps_origin[0], gps_origin[1])

    lane1_old = np.asarray(lane1_old)
    lane2_old = np.asarray(lane2_old)

    lane1_utm = lane1_old + [X_old, Y_old]
    lane2_utm = lane2_old + [X_old, Y_old]

    lane1_ll = np.zeros([len(lane1_utm),2])
    lane2_ll = np.zeros([len(lane2_utm),2])

    for i in range(len(lane1_utm[:,0])):
        lane1_ll[i,0], lane1_ll[i,1] = utm.to_latlon(lane1_utm[i, 0], lane1_utm[i, 1], zone, unknown)     

    for i in range(len(lane2_utm[:,0])):
        lane2_ll[i,0], lane2_ll[i,1] = utm.to_latlon(lane2_utm[i, 0], lane2_utm[i, 1], zone, unknown)
    
    np.savetxt('./test1.txt', lane1_ll, fmt="%.10f")
    np.savetxt('./test2.txt', lane2_ll, fmt="%.10f")
    

    

if __name__=="__main__":

    # gps_current = [37.36478547255, 126.7245715187] # current erp42 gps data
    # utm_current = [301.2333984375, 332.794672167] # current erp42 Ego_topic data

    gps_current = [37.36578294, 126.72539592] # current erp42 gps data
    utm_current = [12.85633468, 7.1850762367] # current erp42 Ego_topic data
    # getLATLON(gps_current, utm_current)
    getUTM(gps_current)

    # gps_origin = [37.36504871398734, 126.72418471076539]

    # lane1, lane2 = loadPath()
    # transferMap(lane1, lane2, gps_origin)