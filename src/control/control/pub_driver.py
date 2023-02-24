#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import sys
sys.path.append('/home/ubuntu/hexapod_ws/src/control/control')
from IMU import *
from ADC import *
from Control import *
from Buzzer import *
imu = IMU()    
adc=ADC()   
c=Control()
class pub_driver(Node):    
    def __init__(self, name):
        super().__init__(name)
        self.buzzer=Buzzer() 
        self.publisher_imu = self.create_publisher(Point,'imu_data', 0)
        self.publisher_bat = self.create_publisher(Float32MultiArray,'battery_data', 0)
        self.sub_vel = self.create_subscription(Twist,'cmd_vel', self.listener_callback,0)
        self.timer1 = self.create_timer(0.001, self.pub_imu) 
        self.timer2 = self.create_timer(0.1, self.pub_battery) 
        
    def pub_imu(self): # publish imu data
        time.sleep(0.000001)
        r,p,y=imu.imuUpdate() # get the imu data
        self.point = Point()
        self.point.x = r # roll
        self.point.y = p # pitch
        self.point.z = y # yaw
        self.publisher_imu.publish(self.point) # publsih

    def pub_battery(self): # publish battery voltage
        batteryVoltage = adc.batteryPower() # get the battery voltage
        if batteryVoltage[0] < 5.5 or batteryVoltage[1]<6: # Buzzer will run when voltage is too low
            for i in range(3):
                self.buzzer.run("1")
                time.sleep(0.15)
                self.buzzer.run("0")
                time.sleep(0.1)
        self.data = Float32MultiArray()
        self.data.data.append(batteryVoltage[0])
        self.data.data.append(batteryVoltage[1])
        self.publisher_bat.publish(self.data) # publsih
        
    def listener_callback(self, data): #cmd_vel callback
        self.twist = data 
        vel_x = self.twist.linear.x
        vel_y = self.twist.linear.y
        omega_z = self.twist.angular.z
        
        self.data=['CMD_MOVE', '1', vel_x, vel_y, '10', omega_z] # '1'= 3 leg run , '2' = 1 leg run at the same time , '10' = speed
        c.run(self.data) # run the spider.
        
def main(args=None):
    rclpy.init(args=args)
    node = pub_driver("pub_driver")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
    
if __name__== '__main__':
    main()