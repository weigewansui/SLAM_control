import lcm
from lcmtypes import maebot_laser_t

msg = maebot_laser_t()
msg.laser_power = 1
lc = lcm.LCM()
while(True):
    lc.publish("MAEBOT_LASER", msg.encode())
    
