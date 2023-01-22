#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt

import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from rosgraph_msgs.msg import Clock

class SendCommand(object):
    def __init__(self):
        self.joint1_command_pub = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size=10)
        self.joint2_command_pub = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size=10)
        rospy.sleep(0.1)
        self.start_time = rospy.get_rostime()

    def run(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            now = rospy.get_rostime()
            t = (now - self.start_time).to_sec()

            poly=np.poly1d([3*np.pi/64,-23*np.pi/64,21*np.pi/32,0,0])
            command1=poly(t)
            command2=poly(t)

            self.joint1_command_pub.publish(command1)
            self.joint2_command_pub.publish(command2)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('hw3_send_command')
    sc = SendCommand()
    sc.run()

    rospy.spin()
