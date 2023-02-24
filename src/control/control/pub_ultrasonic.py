#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import sys
sys.path.append('/home/ubuntu/hexapod_ws/src/control/control')
import time
import RPi.GPIO as GPIO
from Ultrasonic import *
    
ultrasonic = Ultrasonic()
    
class pub_distance(Node):    
    def __init__(self, name):
        super().__init__(name)
        self.publisher_ = self.create_publisher(Int64,'distance', 0)
        self.timer = self.create_timer(0.1, self.pub_distance) 
        
    def pub_distance(self):
        dis = ultrasonic.getDistance()
        print(dis)
        data = Int64()
        data.data = dis
        self.publisher_.publish(data)
    
        
def main(args=None):
    rclpy.init(args=args)
    node = pub_distance("pub_distance")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
    
if __name__== '__main__':
    main()