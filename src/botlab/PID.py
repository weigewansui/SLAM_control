# PID.py
#
# skeleton code for University of Michigan ROB550 Botlab
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import sys, os, time
import lcm, signal
import time
from math import *
import numpy as np

from lcmtypes import odo_pose_xyt_t
from lcmtypes import odo_dxdtheta_t
from lcmtypes import maebot_diff_drive_t
from lcmtypes import maebot_motor_feedback_t
from lcmtypes import slam_pos_t
from lcmtypes import path_t
###############################################
# PID CLASS - SIMILAR TO C CODE FOR CTRLPANEL
###############################################
class PID():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.iTerm = 0.0
        self.iTermMin = -40
        self.iTermMax = 40
        self.prevInput = 0.0
        self.input = 0.0
        self.output = 0.0
        self.setpoint = 0.0
        self.outputMin = -0.4
        self.outputMax = 0.4
        self.updateRate = 0.1
        self.x_ref = 0

    def Compute(self):
       # print 'update rate', self.updateRate
        self.output = 0.0
        x_dot_ref = self.setpoint;
        self.x_ref+= x_dot_ref*self.updateRate;
        x_dot_act = (self.input-self.prevInput)/self.updateRate;

        x_act = self.input;
        #print 'update_rate', self.updateRate
        #print 'act',x_act
        e_d = (x_dot_ref - x_dot_act)
        e_p = (self.x_ref-x_act)
        #print 'x', self.x_ref
        #print 'e_p', e_p
        self.iTerm += e_p*self.updateRate;
        #print 'e_i', self.iTerm
        #print 'e_d', e_d
        if(self.iTerm > self.iTermMax):
            self.iTerm = self.iTermMax
        elif(self.iTerm < self.iTermMin):
            self.iTerm = self.iTermMin
           
        #print 'iTerm', self.iTerm
        self.output = self.kp*e_p + self.kd*e_d + self.ki*self.iTerm
        #print 'output before',self.output

        if(self.output > self.outputMax):
            self.output = self.outputMax
        elif(self.output < self.outputMin):
            self.output = self.outputMin
        self.prevInput = self.input
        #print 'output',self.output    
        return self.output

    def SetTunings(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def SetIntegralLimits(self, imin, imax):
        self.iTermMinMax = imin
        self.iTermMaxMax = imax

    def SetOutputLimits(self, outmin, outmax):
        self.outputMin = outmin
        self.outputMax = outmax
        
    def SetUpdateRate(self, rateInSec):
        self.updateRate = rateInSec
        
    def PIDclear(self):
        print 'PIDCLEAR CALLED'
        self.iTerm = 0
        self.x_ref = 0 
###################################################
# PID SPEED CONTROLLER - USED PID CLAS
###################################################

class PIDSpeedController():
  def __init__(self):
   
    # Create Both PID for controller
    # self.fwdVelCtrl = PID(0.0, 0.0, 0.0)
    # self.angSpeCtrl = PID(0.0, 0.0, 0.0)
    self.rightVelCtrl = PID(0.0, 0.0, 0.0)
    self.leftVelCtrl = PID(0.0, 0.0, 0.0)
    self.angleCtrl = PID(0.0, 0.0, 0.0)

    self.L_pos = 0;
    self.R_pos = 0;
    self.prevEncPos = (0,0,0)           # store previous readings for odometry
    self.initEncPos = (0,0,0)           # store previous readings for odometry
    self.currEncPos = (0,0,0)           # current reading for odometry
    self.wheelDiameterMillimeters = 32.0     # diameter of wheels [mm]
    self.axleLengthMillimeters = 80.0        # separation of wheels [mm]  
    self.ticksPerRev = 16.0                  # encoder tickers per motor revolution
    self.gearRatio = 30.0                    # 30:1 gear ratio
    self.enc2mm = (pi * self.wheelDiameterMillimeters) / (self.gearRatio * self.ticksPerRev) # encoder ticks to distance [mm]
    self.counter = 0
    self.wait_flag = 0
    self.fwdVelSet = 0
    self.angSpeSet = 0
    self.fwdSpeMax = 150
    self.fwdSpeMin = -150

    self.prev_time_vel = 0.0
    self.prev_time_pos = 0.0
    self.curr_time = 0.0
    self.goal_angle = 0
    self.goal_dist = 1000
    self.turn = 0
    self.angSpeMax = 1.5
    self.angSpeMin = -1.5
    self.wait_count = 0
    self.new_wypt_flag = 1
    self.enc_reset_flag = 0
    self.wypt_wait_cnt = 0
    self.rel_init = (0,0,0)
    self.abs_init = (0,0,0)
    # LCM Subscribe
    self.lc = lcm.LCM()
    lcmOdoPoseSub = self.lc.subscribe("BOT_ODO_POSE", self.OdoPositionHandler)
    lcmOdoVelSub = self.lc.subscribe("BOT_ODO_VEL", self.OdoVelocityHandler)
    lcmMotorSub = self.lc.subscribe("MAEBOT_MOTOR_FEEDBACK", self.motorFeedbackHandler)
    lcmSlamPosSub = self.lc.subscribe("SLAM_POS" , self.SlamPositionHandler)
    lcmNextWypySub = self.lc.subscribe("NEXT_WYPT", self.NextWyptHandler)
    
    self.lc2 = lcm.LCM()
    lcmOdoPoseSub = self.lc.subscribe("BOT_ODO_POSE", self.OdoPositionHandler)
    lcmSlamposSub = self.lc2.subscribe("SLAM_POS" , self.SlamPositionHandler)
    lcmMotorSub = self.lc2.subscribe("MAEBOT_MOTOR_FEEDBACK", self.motorFeedbackHandler)
    lcmNextWypySub = self.lc2.subscribe("NEXT_WYPT", self.NextWyptHandler)
    signal.signal(signal.SIGINT, self.signal_handler)

    self.pos = (0,0,0) #(x, y, Theta)
    self.abs_pos = (0,0,0)
    self.last_wypt_pos = (0,0)
    self.wypt_pos = (0,0)
    self.vel = (0,0) #(Vxy, VTheta)
    
    self.rightSpeed = 0.0
    self.leftSpeed = 0.0

    # Solution to check that lcm handle will reveive two msg (pos and vel)
    # before executing further code
    self.msg_counter = [0, 0]
    
  def motorFeedbackHandler(self,channel,data):
	# IMPLEMENT ME
	msg = maebot_motor_feedback_t.decode(data)
   	self.currEncPos = (msg.encoder_left_ticks, msg.encoder_right_ticks, msg.utime)
	self.L_pos = self.enc2mm*(self.currEncPos[0] - self.initEncPos[0])
	self.R_pos = self.enc2mm*(self.currEncPos[1] - self.initEncPos[1])
        if(self.enc_reset_flag):
          print 'currEnc', self.currEncPos
          print 'initEnc', self.initEncPos
          self.enc_reset_flag = 0
	

  def publishMotorCmd(self):
    cmd = maebot_diff_drive_t()
    cmd.motor_right_speed = self.rightSpeed
    cmd.motor_left_speed = self.leftSpeed
    self.lc.publish("MAEBOT_DIFF_DRIVE", cmd.encode())

  def OdoVelocityHandler(self, channel, data):
    self.msg_counter[0] = self.msg_counter[0] + 1
    msg = odo_dxdtheta_t.decode(data)
    # IMPLEMENT ME
    # Handle velocity message
    
    if self.curr_time != msg.utime:
        self.curr_time = msg.utime
        self.vel = (msg.dxy/(msg.dt/1e6), msg.dtheta/(msg.dt))
        #print msg.dt
        # print self.curr_time
        #print self.prev_time_vel
        
        self.leftVelCtrl.SetUpdateRate((self.curr_time - self.prev_time_vel)/1e6)
        self.rightVelCtrl.SetUpdateRate((self.curr_time - self.prev_time_vel)/1e6)
        self.prev_time_vel = self.curr_time;
        #print 1111

  def OdoPositionHandler(self, channel, data):
    self.msg_counter[1] = self.msg_counter[1] + 1
    msg = odo_pose_xyt_t.decode(data)
    # IMPLEMENT ME
    # Handle position message
    self.pos = (msg.xyt[0], msg.xyt[1], msg.xyt[2])
    #self.nav_pos = (msg.xyt[0], msg.xyt[1], msg.xyt[2]-self.theta_init)

  def SlamPositionHandler(self, channel, data):
    self.msg_counter[1] = self.msg_counter[1] + 1
    msg = slam_pos_t.decode(data)
    self.abs_pos = (msg.abs_x, msg.abs_y, msg.abs_theta)

  def NextWyptHandler(self, channel, data):
    if(self.abs_pos[0]==0):
      return
    msg = path_t.decode(data)
    wypt_pos = (msg.abs_x, msg.abs_y)
    if(wypt_pos!=self.wypt_pos):
      print 'new waypoint'
      self.last_wypt_pos = self.wypt_pos
      self.wypt_pos = wypt_pos
      self.goal_dist = np.sqrt( (self.wypt_pos[0]-self.abs_pos[0])**2 + (self.wypt_pos[1]-self.abs_pos[1])**2)
      self.goal_angle = (180.0/3.1415926)*np.arctan2(-(self.wypt_pos[1]-self.abs_pos[1]),-(self.wypt_pos[0]-self.abs_pos[0]))
      self.rightVelCtrl.PIDclear()
      self.leftVelCtrl.PIDclear()
      self.initEncPos = self.currEncPos
      self.enc_reset_flag = 1
      self.wait_flag = 1
      self.new_wypt_flag = 1
      self.wypt_wait_cnt = 0

  def positionController(self):

    while(1):
      self.lc.handle()
      if(self.msg_counter[0] * self.msg_counter[1] > 0):
        self.msg_counter[0] = self.msg_counter[0] - 1
        self.msg_counter[1] = self.msg_counter[1] - 1
        dist_traveled = np.sqrt((self.pos[0] - self.rel_init[0])**2 + (self.pos[1] - self.rel_init[1])**2) 
        #angle_traveled = self.abs_init[2]-(self.pos[2]-self.rel_init[2])

        curr_t_map = self.abs_init[2]-(180.0/3.14159)*(self.pos[2]-self.rel_init[2])
        curr_x_map = self.abs_init[0] - (self.pos[0] - self.rel_init[0])
        #curr_x_map = self.abs_init[0] + dist_traveled*cos(curr_t_map*(3.14159/180.0))
        curr_y_map = self.abs_init[1] + (self.pos[1] - self.rel_init[1])
        #curr_y_map = self.abs_init[1] + dist_traveled*sin(curr_t_map*(3.14159/180.0))

       #print 'delta_x', self.wypt_pos[0] - curr_x_map
        #print 'delta_y', self.wypt_pos[1] - curr_y_map
        #print 'wypt_x', self.wypt_pos[0]
        #print 'wypt_y', self.wypt_pos[1]


        
        #print 'x_slam', self.abs_pos[0]
        #print 'x_odo', curr_x_map
        #print 'y_slam', self.abs_pos[1]
        #print 'y_odo', curr_y_map
        #print 't_slam', self.abs_pos[2]
        #print 't_odo', curr_t_map
        
        angle_traveled = curr_t_map
        dist_traveled = np.sqrt((self.pos[0] - self.rel_init[0])**2 + (self.pos[1] - self.rel_init[1])**2) 
        '''
        print 'angle_err', self.goal_angle - angle_traveled
        print 'dist_traveled', dist_traveled
        print 'wypt_x', self.wypt_pos[0]
        print 'wypt_y', self.wypt_pos[1]
        print 'delta_x', (self.wypt_pos[0]-self.abs_pos[0])
        print 'delta_y', (self.wypt_pos[1]-self.abs_pos[1])
        print 'abs_init', self.abs_init
        #print 'odo_angle', (180/3.1415926)*self.pos[2]
        #print 'slam_angle', -self.abs_pos[2]
        #print 'angle_diff', (180/3.1415926)*self.pos[2]+self.abs_pos[2]
        
        if(self.new_wypt_flag):
          if(self.wypt_wait_cnt < 50):
            self.wypt_wait_cnt+=1
            continue
          else:
            self.new_wypt_flag = 0

            self.abs_init = (self.abs_pos[0], self.abs_pos[1], self.abs_pos[2]) 
            #self.wypt_pos = ((self.abs_init[0]-100), self.abs_init[1])
            self.rel_init = (self.pos[0], self.pos[1], self.pos[2])

            print 'else called'
            print 'wypt_pos', self.wypt_pos
            #print 'abs_init', self.abs_init
        '''    
        #self.fwdVelSet = (self.goal_dist - dist_traveled)*0.5
        if(self.goal_dist > dist_traveled):
          self.fwdVelSet = 50
        else:
          self.fwdVelSet = 0            

          
#        print 'distance_error', self.goal_dist - dist_traveled
#        print 'velset_should', (self.goal_dist - dist_traveled)*0.0001
#        print 'wait_flag', self.wait_flag
        #print 'fwdVelSet', self.fwdVelSet
        if (self.wait_flag):
          if self.counter < 100:
            self.counter += 1
            self.fwdVelSet = 0
          else:
            self.abs_init = (self.abs_pos[0], self.abs_pos[1], self.abs_pos[2]) 
            self.rel_init = (self.pos[0], self.pos[1], self.pos[2])
            self.wait_flag = 0
            self.counter = 0
            self.rightVelCtrl.PIDclear()
            self.leftVelCtrl.PIDclear()
            self.initEncPos = self.currEncPos
            self.enc_reset_flag = 1
            

        if self.fwdVelSet > self.fwdSpeMax:
          self.fwdVelSet = self.fwdSpeMax
        if self.fwdVelSet < self.fwdSpeMin:
          self.fwdVelSet = self.fwdSpeMin
        #self.fwdVelSet = 100
        #print 'goal_angle',self.goal_angle
        #print 'angle_traveled', angle_traveled

        #self.goal_angle = 0
        #angle_traveled = self.pos[2]
        #print 'angle_traveled', angle_traveled
        #self.goal_angle = -self.goal_angle
	#print 'goal_angle', self.goal_angle
        self.angSpeSet = -(self.goal_angle - angle_traveled)*2.5*(3.14159/180.0)
	#self.angSpeSet = (self.goal_angle - angle_traveled)*2.5

        #self.angSpeSet = 0
        #print 'angSpeSet', self.angSpeSet

        if self.angSpeSet > self.angSpeMax:
          self.angSpeSet = self.angSpeMax
        if self.angSpeSet < self.angSpeMin:
          self.angSpeSet = self.angSpeMin

        print 'fwdVelSet', self.fwdVelSet
        self.rightVelCtrl.setpoint = self.fwdVelSet + self.angSpeSet*(80.0)/2.0
        self.leftVelCtrl.setpoint =  self.fwdVelSet - self.angSpeSet*(80.0)/2.0

	#print 'left_input', self.L_pos
        if(self.enc_reset_flag):  
          continue
        else:
          print 'L_pos', self.L_pos
          self.leftVelCtrl.input = self.L_pos;
          self.rightVelCtrl.input = self.R_pos;

        if(self.leftVelCtrl.setpoint==0):
          self.leftSpeed = self.leftVelCtrl.Compute()
        else:
          self.leftSpeed = self.leftVelCtrl.Compute() + self.leftVelCtrl.setpoint/abs(self.leftVelCtrl.setpoint)* 0.12

        #print 'left output', self.leftVelCtrl.Compute()
        if(self.rightVelCtrl.setpoint==0):
          self.rightSpeed = self.rightVelCtrl.Compute()
        else:
          self.rightSpeed = self.rightVelCtrl.Compute() +  self.rightVelCtrl.setpoint/abs(self.rightVelCtrl.setpoint)*0.12

        #self.leftSpeed = 0.1
        #self.rightSpeed =  0.1
        
        self.publishMotorCmd()
        

  # Main Program Loop
  def Controller(self):

    # For now give a fixed command here
    # Later code to get from groundstation should be used
    # self.fwdVelCtrl.setpoint = 150.0
    # self.angSpeCtrl.setpoint = 0.0
    #self.rightVelCtrl.setpoint = self.fwdVelSet + self.angSpeSet*(80)/2
    #self.leftVelCtrl.setpoint = self.fwdVelSet - self.angSpeSet*(80)/2
    
    while(1):
      
      self.lc.handle()
      if(self.msg_counter[0] * self.msg_counter[1] > 0):
        # IMPLEMENT ME
        # MAIN CONTROLLER
        
        self.msg_counter[0] = self.msg_counter[0] - 1
        self.msg_counter[1] = self.msg_counter[1] - 1
        
        dist_traveled = (self.L_pos + self.R_pos)/2
  #      goal_dist = 1000
        angle_traveled = self.pos[2]
#        self.fwdVelSet = ((goal_dist-dist_traveled)/10)*100
 #       if(self.fwdVelSet>100):
        self.fwdVelSet=100
        
#        print 'dist_traveled',dist_traveled
               
        if((angle_traveled >= self.goal_angle) and (self.turn==1)):
            self.rightVelCtrl.PIDclear()
            self.leftVelCtrl.PIDclear()
            #if self.counter < 20:
             #   self.counter+=1
              #  self.leftVelCtrl.Compute()
               # self.rightVelCtrl.Compute()
                #continue
            self.wait_flag = 1
            self.turn = 0
           # self.goal_dist += 1000
            self.counter = 0
            self.rightVelCtrl.PIDclear()
            self.leftVelCtrl.PIDclear()
            self.initEncPos = self.currEncPos
            continue

        if((dist_traveled >= self.goal_dist) and (self.turn==0)):

            self.rightVelCtrl.PIDclear()
            self.leftVelCtrl.PIDclear()
            #if self.counter < 20:
            #    self.counter+=1
            #    self.leftVelCtrl.Compute()
            #    self.rightVelCtrl.Compute()
            #    continue
            self.wait_flag = 1
            self.turn = 1
            self.goal_angle += 3.1415/2
            self.counter = 0
            self.rightVelCtrl.PIDclear()
            self.leftVelCtrl.PIDclear()
            self.initEncPos = self.currEncPos
            continue

        if(self.turn == 0):
            self.fwdVelSet = 100
            #self.angSpeSet = 0

        if(self.turn == 1):
            self.fwdVelSet = 0
            
        if (self.wait_flag) :

            if self.counter < 20 :
                self.counter += 1
                self.fwdVelSet = 0
            else:
                self.wait_count += 1
                self.wait_flag = 0
                
        if (self.wait_count == 8):
            sys.exit(0)

        self.angSpeSet = (self.goal_angle - angle_traveled)*2.5 #+ self.vel[1]*(-2.5)
  #      print 'angSpeSet', self.angSpeSet
        if self.angSpeSet > self.angSpeMax:
            self.angSpeSet = self.angSpeMax
        if self.angSpeSet < self.angSpeMin:
            self.angSpeSet = self.angSpeMin

        self.rightVelCtrl.setpoint = self.fwdVelSet + self.angSpeSet*(80)/2
        self.leftVelCtrl.setpoint = self.fwdVelSet - self.angSpeSet*(80)/2
        
        self.leftVelCtrl.input = self.L_pos;
        self.rightVelCtrl.input = self.R_pos;

        #print 'left'

        self.leftSpeed = self.leftVelCtrl.Compute() + self.leftVelCtrl.setpoint/abs(self.leftVelCtrl.setpoint)* 0.12
       # print 'right'

        self.rightSpeed = self.rightVelCtrl.Compute() +  self.rightVelCtrl.setpoint/abs(self.rightVelCtrl.setpoint)*0.12
#       print 'right Output:',self.rightVelCtrl.output

        # self.rightSpeed = fwdVelOutput + angSpeOutput*80/2
        # self.leftSpeed = fwdVelOutput + angSpeOutput*80/2
#        print 'right Output',self.rightVelCtrl.output
        self.publishMotorCmd()

  # Function to print 0 commands to morot when exiting with Ctrl+C 
  # No need to change 
  def signal_handler(self,  signal, frame):
    print("Terminating!")
    for i in range(5):
      cmd = maebot_diff_drive_t()
      cmd.motor_right_speed = 0.0
      cmd.motor_left_speed = 0.0	
      self.lc.publish("MAEBOT_DIFF_DRIVE", cmd.encode())
    exit(1)


###################################################
# MAIN FUNCTION
###################################################
if __name__ == "__main__":
  pid = PIDSpeedController()
  while ((not pid.msg_counter[0]) or (not pid.msg_counter[1])):
      pid.lc.handle()
  pid.initEncPos = pid.currEncPos
  pid.msg_counter[0] -= 1
  pid.msg_counter[1] -= 1
  pid.prev_time_vel = pid.curr_time
  # pid.fwdVelCtrl.SetTunings(1, 0, 0)
  # pid.angSpeCtrl.SetTunings(1, 0, 0)
  pid.rightVelCtrl.SetTunings(0.0014, 0.0015, 0.001)
  pid.leftVelCtrl.SetTunings(0.0014, 0.0015, 0.001)
  #pid.Controller()
  pid.positionController()

