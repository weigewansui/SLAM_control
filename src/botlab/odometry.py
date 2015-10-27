# odometry.py
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
import lcm
from math import *

from breezyslam.robots import WheeledRobot

from lcmtypes import maebot_motor_feedback_t
from lcmtypes import maebot_sensor_data_t
from lcmtypes import odo_pose_xyt_t
from lcmtypes import odo_dxdtheta_t

class Maebot(WheeledRobot):
    
	def __init__(self):
		self.wheelDiameterMillimeters = 32.0     # diameter of wheels [mm]
		self.axleLengthMillimeters = 80.0        # separation of wheels [mm]  
		self.ticksPerRev = 16.0                  # encoder tickers per motor revolution
		self.gearRatio = 30.0                    # 30:1 gear ratio
		self.enc2mm = (pi * self.wheelDiameterMillimeters) / (self.gearRatio * self.ticksPerRev) # encoder ticks to distance [mm]

		self.prevEncPos = (0,0,0)           # store previous readings for odometry
		self.prevOdoPos = (0,0,0)           # store previous x [mm], y [mm], theta [rad]
		self.currEncPos = (0,0,0)           # current reading for odometry
		self.currOdoPos = (0,0,0)           # store odometry x [mm], y [mm], theta [rad]
		self.currOdoVel = (0,0,0)           # store current velocity dxy [mm], dtheta [rad], dt [s]
		self.gyro = (0, 0, 0)
                self.gyro_int = (0,0,0)
                self.prev_gyro_int = (0,0,0)
                self.dTheta_gyro = 0
                self.dTheta_odo = 0
                self.Theta = 0
                self.cur_time = 0
                self.prev_time = 0
		WheeledRobot.__init__(self, self.wheelDiameterMillimeters/2.0, self.axleLengthMillimeters/2.0)

		# LCM Initialization and Subscription
		self.lc = lcm.LCM()
		lcmMotorSub = self.lc.subscribe("MAEBOT_MOTOR_FEEDBACK", self.motorFeedbackHandler)
		lcmSensorSub = self.lc.subscribe("MAEBOT_SENSOR_DATA", self.sensorDataHandler)
		self.gyro_bias = (0,0,0)
  
	def calcVelocities(self):
		# IMPLEMENT ME
		# TASK: CALCULATE VELOCITIES FOR ODOMETRY
		# Update self.currOdoVel and self.prevOdoVel
		dt = (self.currEncPos[2] - self.prevEncPos[2])
		vel = ((self.currOdoPos[0] - self.prevOdoPos[0]), (self.currOdoPos[1]-self.prevOdoPos[1])); 
		dxy = sqrt(vel[0]*vel[0]+vel[1]*vel[1])
		dtheta = (self.currOdoPos[2] - self.prevOdoPos[2])
		self.currOdoVel = (dxy, dtheta, dt)

	def getVelocities(self):
		# Return a tuple of (dxy [mm], dtheta [rad], dt [s])
		return self.currOdoVel

	def calcOdoPosition(self):
		# IMPLEMENT ME
		# TASK: CALCULATE POSITIONS
		# Update self.currOdoPos and self.prevOdoPos
		dL = self.enc2mm*(self.currEncPos[0] - self.prevEncPos[0])
		dR = self.enc2mm*(self.currEncPos[1] - self.prevEncPos[1])
	        self.dTheta_odo = (dR-dL)/self.axleLengthMillimeters
		dTheta = (dR-dL)/self.axleLengthMillimeters
		dF = (dL + dR)/2

		if abs(self.dTheta_odo - self.dTheta_gyro)*180/3.1415926 > 2:
			dTheta = self.dTheta_gyro
		else:
			dTheta = self.dTheta_odo
			
		#print dTheta

		theta = self.prevOdoPos[2] + dTheta
		dX = dF*cos(theta)
		dY = dF*sin(theta)
		
		self.prevOdoPos = self.currOdoPos
		self.currOdoPos = (self.prevOdoPos[0] \
			+ dX, self.prevOdoPos[1] + dY, self.prevOdoPos[2] + dTheta)
		
	def getOdoPosition(self):
	    
		# Return a tuple of (x [mm], y [mm], theta [rad])
		return self.currOdoPos

	def publishOdometry(self):
		msg = odo_pose_xyt_t();
		
		msg.utime = time.time()*1e6
		msg.xyt = list(self.getOdoPosition())
		
		lc = lcm.LCM()
		lc.publish("BOT_ODO_POSE", msg.encode())

  
	def publishVelocities(self):
		#velocities = getVelocities()
		msg = odo_dxdtheta_t();
		
		msg.utime = time.time()*1e6
		msg.dxy = self.currOdoVel[0]
		msg.dtheta = self.currOdoVel[1]
		msg.dt = self.currOdoVel[2]

		lc = lcm.LCM()
		lc.publish("BOT_ODO_VEL", msg.encode())

	def motorFeedbackHandler(self,channel,data):
		# IMPLEMENT ME
		msg = maebot_motor_feedback_t.decode(data)
		
		self.prevEncPos = self.currEncPos
	   	self.currEncPos = (msg.encoder_left_ticks, msg.encoder_right_ticks, msg.utime)
		#print self.currEncPos
		self.calcOdoPosition()
		self.calcVelocities()
		
		
	def sensorDataHandler(self,channel,data):
		msg = maebot_sensor_data_t.decode(data)
		# IMPLEMENT ME
		# TASK: PROCESS GYRO DATA
		self.gyro = (msg.gyro[0]-self.gyro_bias[0],\
		 msg.gyro[1] - self.gyro_bias[1], msg.gyro[2] - self.gyro_bias[2])
                self.gyro_int = msg.gyro_int
                self.cur_time = msg.utime
                t_step = self.cur_time - self.prev_time
                self.prev_time = self.cur_time
                self.dTheta_gyro = ((self.gyro_int[2] - self.prev_gyro_int[2] - self.gyro_bias[2]*t_step)/(131*1e6))*3.1415926/180
                self.prev_gyro_int = self.gyro_int
		#self.Theta += self.dTheta_gyro
		
	def gyroCali(self):
		print "start calibration"
		Time0 = time.time()
		gyro_x_accu = 0
		gyro_y_accu = 0
		gyro_z_accu = 0
		iterate_num = 0
		
		while(time.time() - Time0 < 10):
			self.lc.handle();
			gyro_x_accu = gyro_x_accu + self.gyro[0];
			gyro_y_accu = gyro_y_accu + self.gyro[1];
			gyro_z_accu = gyro_z_accu + self.gyro[2];
			iterate_num = iterate_num + 1

		self.gyro_bias = (gyro_x_accu/iterate_num, gyro_y_accu/iterate_num, gyro_z_accu/iterate_num)
		print "gyro calibration done"
		print self.gyro_bias
		
	def MainLoop(self):
		oldTime = time.time()
		frequency = 20;
		self.lc.handle()
		self.prevEncPos = self.currEncPos                                                                                                                          
		self.currOdoPos = (0,0,0)           # store odometry x [mm], y [mm], theta [rad]                                                                                                            
                self.currOdoVel = (0,0,0)   

		while(1):
			self.lc.handle()
			if(time.time()-oldTime > 1.0/frequency):
				self.publishOdometry()
				self.publishVelocities()
				oldTime = time.time()
                                #print self.dTheta_gyro   


if __name__ == "__main__":
  
  robot = Maebot()
  robot.gyroCali()
  robot.lc.handle()
  robot.prev_gyro_int = robot.gyro_int
  robot.prev_time = robot.cur_time
  robot.MainLoop()  
