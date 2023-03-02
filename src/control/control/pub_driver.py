#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import sys
sys.path.append('/home/ubuntu/hexapod_ws/src/control/control/bottom_layer')
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
        self.publisher_imu = self.create_publisher(Imu,'Imu_data', 0)
        self.publisher_bat = self.create_publisher(Float32MultiArray,'battery_data', 0)
        self.sub_vel = self.create_subscription(Twist,'cmd_vel', self.listener_callback,10)
        self.timer1 = self.create_timer(0.001, self.pub_imu) 
        self.timer2 = self.create_timer(0.1, self.pub_battery) 
        
    def pub_imu(self): # publish imu data
        time.sleep(0.000001)
        q0,q1,q2,q3=imu.imuUpdate() # get the imu data
        self.imu_data = Imu()
        self.imu_data.header.stamp = self.get_clock().now().to_msg()
        self.imu_data.header.frame_id = 'IMU'   
        self.imu_data.orientation.x = q0 
        self.imu_data.orientation.y = q1 
        self.imu_data.orientation.z = q2 
        self.imu_data.orientation.w = q3
        self.publisher_imu.publish(self.imu_data) # publsih

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
        x = self.twist.linear.x
        y = self.twist.linear.y
        omega_z = self.twist.angular.z * 10 #測試x, y大小對速度的影響
        vel = 10*((x**2 + y**2)**(1/2))/1.4145
        x = x*25/1
        y = y*25/1
        vel = 8
        print(x, y, vel, omega_z)
        
        self.data=['CMD_MOVE', '1', x, y, vel, omega_z] # '1'= 3 leg run , '2' = 1 leg run at the same time , '10' = speed
        c.run(self.data) # run the spider.
        
def main(args=None):
    rclpy.init(args=args)
    node = pub_driver("pub_driver")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
    
if __name__== '__main__':
    main()