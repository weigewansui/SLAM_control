 
import sys, os, time
import lcm
import math
import numpy as np

from lcmtypes import maebot_motor_feedback_t
from lcmtypes import maebot_sensor_data_t
from lcmtypes import odo_pose_xyt_t
from lcmtypes import odo_dxdtheta_t
from lcmtypes import rplidar_laser_t
from lcmtypes import slam_pos_t

from laser import *
from slam import *
from maps import *

# SLAM preferences
# CHANGE TO OPTIMIZE
USE_ODOMETRY = True
MAP_QUALITY = 3

# Laser constants
# CHANGE TO OPTIMIZE IF NECSSARY
DIST_MIN = 100; # minimum distance
DIST_MAX = 6000; # maximum distance

# Map constants
# CHANGE TO OPTIMIZE
MAP_SIZE_M = 6.0 # size of region to be mapped [m]
INSET_SIZE_M = 1.0 # size of relative map
MAP_RES_PIX_PER_M = 100 # number of pixels of data per meter [pix/m]
MAP_SIZE_PIXELS = int(MAP_SIZE_M*MAP_RES_PIX_PER_M) # number of pixels across the entire map
MAP_DEPTH = 3 # depth of data points on map (levels of certainty)

# CONSTANTS
DEG2RAD = math.pi / 180
RAD2DEG = 180 / math.pi

# KWARGS
# PASS TO MAP AND SLAM FUNCTIONS AS PARAMETER
KWARGS, gvars = {}, globals()
for var in ['MAP_SIZE_M','INSET_SIZE_M','MAP_RES_PIX_PER_M','MAP_DEPTH','USE_ODOMETRY','MAP_QUALITY']:
  KWARGS[var] = gvars[var] # constants required in modules

class slam_position:

    def __init__(self):

        self.rel_pos = (0, 0, 0)
        self.count = 0
        self.vel_accum = (0,0,0)
        self.initialzed_map = False

        self.currOdoVel = (0,0,0)
        self.ranges = 0
        self.thetas = 0
        self.nranges = 0
        self.laser_times = 0
        self.intensities = 0
        self.map_count = 0
        self.slam_pos_msg = slam_pos_t()

        self.lc = lcm.LCM()

        lcmOdoVelSub = self.lc.subscribe("BOT_ODO_VEL", self.OdoVelocityHandler)
        rpLidarSub = self.lc.subscribe("RPLIDAR_LASER", self.LidarHandler)
        self.laser = RPLidar(DIST_MIN, DIST_MAX) # lidar
        self.datamatrix = DataMatrix(**KWARGS) # handle map data
        self.slam = Slam(self.laser, **KWARGS) # do slam processing
    
    def OdoVelocityHandler(self,channel,data): 

        msg = odo_dxdtheta_t.decode(data)
        self.currOdoVel = (msg.dxy, msg.dtheta, msg.dt)
        self.vel_accum += self.currOdoVel
        self.count += 1

    def LidarHandler(self,channel,data): 

        msg = rplidar_laser_t.decode(data)
        self.ranges = msg.ranges
        self.thetas = msg.thetas
        self.nranges = msg.nranges
        self.laser_times = msg.times
        self.intensities = msg.intensities

        if self.count == 0:
            return

        vel_avg = (self.vel_accum[0],self.vel_accum[1],self.vel_accum[2])
        self.vel_accum = (0,0,0)
        self.count = 0
        tmp_range = map(lambda ranges: 1000*ranges, self.ranges)
        tmp_thetas = map(lambda thetas: RAD2DEG*thetas, self.thetas)
        curr_position = self.slam.updateSlam(zip(tmp_range, tmp_thetas), vel_avg)

        if self.initialzed_map:
            self.datamatrix.getRobotPos(curr_position)
        else:
            self.datamatrix.getRobotPos(curr_position, True)
            self.initialzed_map = True

        rel_pos = self.datamatrix.get_robot_rel()
        self.slam_pos_msg.rel_x = rel_pos[1]
        self.slam_pos_msg.rel_y = rel_pos[0]
        self.slam_pos_msg.rel_theta = rel_pos[2]
        self.slam_pos_msg.abs_x = curr_position[1]
        self.slam_pos_msg.abs_y = curr_position[0]
        self.slam_pos_msg.abs_theta = curr_position[2]

        self.lc.publish("SLAM_POS", self.slam_pos_msg.encode())


    def Main_loop(self):
        while(1):
            self.lc.handle()

Hello = slam_position()
Hello.Main_loop()