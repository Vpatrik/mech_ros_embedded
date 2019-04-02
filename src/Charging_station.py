#!/usr/bin/env python

# Patrik Vavra 2019
# Roman Adamek 2019

import smbus
import rospy

from std_srvs.srv import Trigger
from std_srvs.srv import TriggerRequest
from std_srvs.srv import TriggerResponse

from std_msgs.msg import Bool


class ChargingStation:
    def __init__(self):
        rospy.init_node('RPi_docking_station')

        self.DS0_ADDR = 0x69
        self.bus = smbus.SMBus(1)
        self.solenoid_lock = True
        self.endstop = Bool()

        self.solenoid_service = rospy.Service('/set_solenoid', Trigger, self.set_solenoid)
        self.endstop_publisher = rospy.Publisher('/endstop', Bool, queue_size=5)

    def set_solenoid(self, req):

        self.bus.write_byte_data(self.DS0_ADDR,  1, 0)

        response = TriggerResponse()
        rospy.loginfo("Solenoid unlocked!")
        response.success = True
        response.message = "Unlocked"
       
        return response

    def i2c_read_data(self, addr, cmd):
        self.bus.write_byte(addr, cmd)
        rospy.sleep(0.1)
        return self.bus.read_byte(addr)

    def run(self):
        rate = rospy.Rate(10) # Rate of loop in [Hz]
        while not rospy.is_shutdown():
            
            try:
                self.endstop.data = self.i2c_read_data(self.DS0_ADDR, 2)
                if self.endstop.data == 128:
                    self.endstop.data = 0;
                elif self.endstop.data == 129:
                    self.endstop.data = 1;
                    
                self.endstop_publisher.publish(self.endstop)
            except IOError:
                rospy.logdebug('Error in communication with arduino in charging station!')
                
            rate.sleep()

    
if __name__ == "__main__":
    station = ChargingStation()
    try:
        station.run()
    except rospy.ROSInterruptException:
        pass
