import sys, os, time
import lcm
import math
import numpy as np
import loadMap
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
 
import matplotlib.backends.backend_agg as agg

import pygame
from pygame.locals import *

from lcmtypes import maebot_diff_drive_t

from lcmtypes import odo_dxdtheta_t
from lcmtypes import odo_pose_xyt_t
from lcmtypes import rplidar_laser_t
from lcmtypes import velocity_cmd_t
from lcmtypes import pid_init_t
from lcmtypes import slam_pos_t
from lcmtypes import path_t
# Callling recently included files
from laser import *
from slam import *
from maps import *

# SLAM preferences
# CHANGE TO OPTIMIZE
USE_ODOMETRY = True
MAP_QUALITY = 5

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

class MainClass:
  def __init__(self, width=640, height=480, FPS=10):
    pygame.init()
    self.width = width
    self.height = height
    self.screen = pygame.display.set_mode((self.width, self.height))
    self.currOdoPos = (0,0,0)
    self.currOdoVel = (0,0,0)
    self.goalPos = (0,0)
    self.slamPos = (0,0)
    self.ranges = 0
    self.thetas = 0
    self.nranges = 0
    self.laser_times = 0
    self.intensities = 0
    self.map_count = 0
    self.auto_flag = False
    self.planner = loadMap.MotionPlanner()
    self.wypt_indx = 1
    self.currWypt = (0,0)
    self.path = []
    self.count = 0
    self.cmd = maebot_diff_drive_t() 
    self.fileWritted = 0
    self.time_counter = 0
    self.vel_accum = (0,0,0)
    self.initialized_map = False
    self.replan_time = 0
    # LCM Subscribe

    self.lc = lcm.LCM()
    # NEEDS TO SUBSCRIBE TO OTHER LCM CHANNELS LATER!!!
    lcmOdoVelSub = self.lc.subscribe("BOT_ODO_VEL", self.OdoVelocityHandler)
    lcmOdoPosSub = self.lc.subscribe("BOT_ODO_POSE", self.OdoPosHandler)
    rpLidarSub = self.lc.subscribe("RPLIDAR_LASER", self.LidarHandler)

    # Prepare Figure for Lidar
    self.fig = plt.figure(figsize=[3, 3], # Inches
                              dpi=100)    # 100 dots per inch, 
    self.fig.patch.set_facecolor('white')
    self.fig.add_axes([0,0,1,1],projection='polar')
    self.ax = self.fig.gca()

    self.fig2 = plt.figure(figsize=[3, 3], # Inches
                              dpi=100)    # 100 dots per inch, 
    self.fig2.patch.set_facecolor('white')
    self.fig2.add_axes([0,0,1,1])
    self.ax2 = self.fig2.gca()

    # Prepare Figures for control
    path = os.path.realpath(__file__)
    path = path[:-17] + "maebotGUI/"

    self.arrowup = pygame.image.load(path + 'fwd_button.png')
    self.arrowdown = pygame.image.load(path + 'rev_button.png')
    self.arrowleft = pygame.image.load(path + 'left_button.png')
    self.arrowright = pygame.image.load(path + 'right_button.png')
    self.resetbut = pygame.image.load(path + 'reset_button.png')
    self.arrows = [0,0,0,0]
        # PID Initialization - Change for your Gains!
    command = pid_init_t()
    command.kp = 0.0        
    command.ki = 0.0
    command.kd = 0.0
    command.iSat = 0.0 # Saturation of Integral Term. 
                           # If Zero shoudl reset the Integral Term
    self.lc.publish("GS_PID_INIT",command.encode())

    # Declare Laser, datamatrix and slam
    self.laser = RPLidar(DIST_MIN, DIST_MAX) # lidar
    self.datamatrix = DataMatrix(**KWARGS) # handle map data
    self.slam = Slam(self.laser, **KWARGS) # do slam processing
    
  def OdoPosHandler(self, channel, data):
    msg = odo_pose_xyt_t.decode(data)
    self.currOdoPos = (msg.xyt[0], msg.xyt[1], msg.xyt[2])
  
  def OdoVelocityHandler(self,channel,data): 

    msg = odo_dxdtheta_t.decode(data)
    self.currOdoVel = (msg.dxy, msg.dtheta, msg.dt)
    self.vel_accum = (self.vel_accum[0] + self.currOdoVel[0], \
          self.vel_accum[1] + self.currOdoVel[1], \
          self.vel_accum[2] + self.currOdoVel[2])
    self.count += 1
   
  def arrivedWypt(self, currPos, wypt):
    if np.sqrt((currPos[0] - wypt[0])**2 + (currPos[1] - wypt[1])**2) < 10:
      return True
    else:
      return False
 

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
    slam_pos_msg = slam_pos_t()
    slam_pos_msg.abs_x = curr_position[1]
    slam_pos_msg.abs_y = curr_position[0]
    slam_pos_msg.abs_theta = curr_position[2]

    self.lc.publish("SLAM_POS", slam_pos_msg.encode())
    if self.initialized_map:
      self.datamatrix.getRobotPos(curr_position)
    else:
      self.datamatrix.getRobotPos(curr_position, True)
      self.initialzed_map = True

    ret = self.slam.getBreezyMap()

    self.datamatrix.drawBreezyMap(ret)
    self.datamatrix.drawInset()
    self.slamPos = self.datamatrix.get_robot_abs()

  def MainLoop(self):
    pygame.key.set_repeat(1, 20)
    vScale = 0.5

    # Prepare Text to Be output on Screen
    font = pygame.font.SysFont("DejaVuSans Mono",14)

    while 1:
      self.time_counter+=1
      leftVel = 0
      rightVel = 0
      
      if not self.auto_flag:
        for event in pygame.event.get():
          if event.type == pygame.QUIT:
            sys.exit()
          elif event.type == KEYDOWN:
            if ((event.key == K_ESCAPE)
            or (event.key == K_q)):
              sys.exit()

            key = pygame.key.get_pressed()
            if key[K_RIGHT]:
              leftVel = leftVel + 0.40
              rightVel = rightVel - 0.40
            elif key[K_LEFT]:
              leftVel = leftVel - 0.40
              rightVel = rightVel + 0.40
            elif key[K_UP]:
              leftVel = leftVel + 0.4
              rightVel = rightVel + 0.4
            elif key[K_DOWN]:
              leftVel = leftVel - 0.4
              rightVel = rightVel - 0.4

            elif key[K_g]:
              self.goalPos = (self.slamPos[0],self.slamPos[1])

            elif key[K_TAB]:
              self.auto_flag = True
              self.planner.map_data = self.datamatrix.mapMatrix 
              self.planner.discretizeMap()       

            else:
              leftVel = 0.0
              rightVel = 0.0                      

        if(self.time_counter>2):
          self.cmd.motor_right_speed = vScale * rightVel
          self.cmd.motor_left_speed = vScale * leftVel
          self.time_counter=0

      	self.lc.publish("MAEBOT_DIFF_DRIVE",self.cmd.encode())
      	
      else:
        if(time.time()-self.replan_time > 2):
          self.planner.astar((self.slamPos[0], self.slamPos[1]), (self.goalPos[0],self.goalPos[1]))
          true_path = self.planner.getPathCoords()
          if not self.fileWritted:
          	self.planner.writeMapToFile()
          	self.fileWritted = 1


          self.replan_time = time.time()
          print true_path
          if(self.path != true_path):
            self.path = true_path
            self.wypt_indx = 1
            self.currWypt = self.path[self.wypt_indx]

        msg = path_t()
        print 'goalPos', self.goalPos
        print 'currPos', self.slamPos

        msg.abs_x = self.currWypt[1]
        msg.abs_y = self.currWypt[0]
        self.lc.publish('NEXT_WYPT',msg.encode())

      self.screen.fill((255,255,255))
      plt.cla()
      self.ax.plot(0,0,'or',markersize=2)

      self.ax.plot(self.thetas,self.ranges,'ro')
      self.ax.set_rmax(1.5)
      self.ax.set_theta_direction(-1)
      self.ax.set_theta_zero_location("N")
      self.ax.set_thetagrids([0,45,90,135,180,225,270,315],
                              labels=['','','','','','','',''], 
                              frac=None,fmt=None)
      self.ax.set_rgrids([0.5,1.0,1.5],labels=['0.5','1.0',''],
                              angle=None,fmt=None)

      canvas = agg.FigureCanvasAgg(self.fig)
      canvas.draw()
      renderer = canvas.get_renderer()
      raw_data = renderer.tostring_rgb()
      size = canvas.get_width_height()
      surf = pygame.image.fromstring(raw_data, size, "RGB")
      self.screen.blit(surf, (320,0))

      surf2 = pygame.surface.Surface((100,100),0,8)

      tmp_inset = self.datamatrix.getInsetMatrix()
      tmp_inset = np.asarray(tmp_inset)
      tmp_inset = tmp_inset.astype("uint8")


      surf2 = pygame.surfarray.make_surface(tmp_inset)
      surf2 = pygame.transform.scale(surf2,(250, 250))
      a = range(0,255)
      b = [255]*len(a)
      surf2.set_palette(zip(a,a,a,b))
      # pygame.surfarray.blit_array(surf2, self.datamatrix.getInsetMatrix())
      self.screen.blit(surf2, (20,20))
      # Position and Velocity Feedback Text on Screen
      self.lc.handle()

      pygame.draw.rect(self.screen,(0,0,0),(5,350,300,120),2)
      text = font.render("  POSITION  ",True,(0,0,0))
      self.screen.blit(text,(10,360))
      str1 = "x: %f [mm]" % ((self.currOdoPos[0]) )
      text = font.render(str1,True,(0,0,0))
      self.screen.blit(text,(10,390))
      str2 = "y: %f [mm]" % ((self.currOdoPos[1]) )
      text = font.render(str2,True,(0,0,0))
      self.screen.blit(text,(10,420))
      str3 = "t: %f [deg]" % ((self.currOdoPos[2]*(180/3.1415926)) )
      text = font.render(str3,True,(0,0,0))
      self.screen.blit(text,(10,450))
 
      text = font.render("  VELOCITY  ",True,(0,0,0))
      self.screen.blit(text,(150,360))
      str4 = "dxy/dt: %f [mm/s]" % (self.currOdoVel[0]/(self.currOdoVel[2]/1e6))
      text = font.render(str4,True,(0,0,0))
      self.screen.blit(text,(150,390))
      str5 = "dth/dt: %f [deg/s]" % (self.currOdoVel[1]/(self.currOdoVel[2]/1e6))
      text = font.render(str5,True,(0,0,0))
      self.screen.blit(text,(150,420))
      str6 = "dt: %f [s]" % (self.currOdoVel[2]/1e6)
      text = font.render(str6,True,(0,0,0))
      self.screen.blit(text,(150,450))

      # Plot Buttons

      self.screen.blit(self.arrowup,(438,325))
      self.screen.blit(self.arrowdown,(438,400))
      self.screen.blit(self.arrowleft,(363,400))
      self.screen.blit(self.arrowright,(513,400))
      self.screen.blit(self.resetbut,(513,325))

      pygame.display.flip()

  def MainLoop2(self):
    while 1:
      self.lc.handle()


MainWindow = MainClass()
MainWindow.lc.handle()
MainWindow.MainLoop()