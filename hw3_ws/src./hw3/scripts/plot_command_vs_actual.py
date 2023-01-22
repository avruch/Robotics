#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
import time
import os

import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from rosgraph_msgs.msg import Clock

class Plot(object):
    def __init__(self):
        self.joint1_actual_values = []
        self.joint1_actual_timestamps = []
        self.joint1_command_values = []
        self.joint1_command_timestamps = []
        self.joint2_actual_values = []
        self.joint2_actual_timestamps = []
        self.joint2_command_values = []
        self.joint2_command_timestamps = []
        rospy.Subscriber('/rrbot/joint1_position_controller/state', JointControllerState, self.joint1_state_callback)
        rospy.Subscriber('/rrbot/joint1_position_controller/command', Float64, self.joint1_command_callback)
        rospy.Subscriber('/rrbot/joint2_position_controller/state', JointControllerState, self.joint2_state_callback)
        rospy.Subscriber('/rrbot/joint2_position_controller/command', Float64, self.joint2_command_callback)

    def joint1_state_callback(self, msg):
        now = rospy.get_rostime()
        self.joint1_actual_timestamps.append(now.to_nsec())
        self.joint1_actual_values.append(msg.process_value)

    def joint1_command_callback(self, msg):
        now = rospy.get_rostime()
        self.joint1_command_timestamps.append(now.to_nsec())
        self.joint1_command_values.append(msg.data)

    def joint2_state_callback(self, msg):
        now = rospy.get_rostime()
        self.joint2_actual_timestamps.append(now.to_nsec())
        self.joint2_actual_values.append(msg.process_value)

    def joint2_command_callback(self, msg):
        now = rospy.get_rostime()
        self.joint2_command_timestamps.append(now.to_nsec())
        self.joint2_command_values.append(msg.data)

    def save_results(self):
        print ('Saving command-vs-actual plots as png files')

        plt.figure()
        plt.plot(self.joint1_actual_timestamps, self.joint1_actual_values, label='actual')
        plt.plot(self.joint1_command_timestamps, self.joint1_command_values, label='command')
        plt.legend()
        plt.savefig(os.path.join(os.path.expanduser('~'), 'joint1_command_vs_actual.png'))

        plt.figure()
        plt.plot(self.joint2_actual_timestamps, self.joint2_actual_values, label='actual')
        plt.plot(self.joint2_command_timestamps, self.joint2_command_values, label='command')
        plt.legend()
        plt.savefig(os.path.join(os.path.expanduser('~'), 'joint2_command_vs_actual.png'))


if __name__ == '__main__':
    rospy.init_node('hw3_plot_command_vs_actual')
    p = Plot()

    start_time = time.time()
    while time.time() - start_time < 8:
        time.sleep(1)
    p.save_results()
