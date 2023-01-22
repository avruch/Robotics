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
		print("Saving command-vs-actual plots as png files")

		plt.figure()
		plt.ylim(-3, 3)
		plt.plot(self.joint1_actual_timestamps, self.joint1_actual_values, label='actual')
		plt.plot(self.joint1_command_timestamps, self.joint1_command_values, label='command')
		plt.legend()
		plt.savefig(os.path.join(os.path.expanduser('~'), 'joint1_command_vs_actual.png'))

		plt.figure()
		plt.ylim(-3, 3)
		plt.plot(self.joint2_actual_timestamps, self.joint2_actual_values, label='actual')
		plt.plot(self.joint2_command_timestamps, self.joint2_command_values, label='command')
		plt.legend()
		plt.savefig(os.path.join(os.path.expanduser('~'), 'joint2_command_vs_actual.png'))

	def calc_measures(self):
		last_desired_pose_1 = self.joint1_command_values[-1]
		last_desired_pose_2 = self.joint2_command_values[-1]
		error_1 = [abs(a-c) for a, c in zip(self.joint1_actual_values, self.joint1_command_values)]
		error_2 = [abs(b-d) for b, d in zip(self.joint2_actual_values, self.joint2_command_values)]
		final_settling_time_1 = 0
		final_settling_time_2 = 0
		for i in range(len(error_1)):
			if abs(error_1[i] / (-last_desired_pose_1)) <= 0.05:
				final_settling_time_1 = self.joint1_actual_timestamps[i] 
				break
		average_error_1 = sum(error_1)/len(error_1)
		max_error_1 = max(error_1)
		final_settling_time_1_insec= final_settling_time_1/ 1e9
		print("Final settling time in sec: ", final_settling_time_1_insec)
		print("Average tracking error: ", average_error_1)
		print("Maximum tracking error: ", max_error_1)
		for i in range(len(error_2)):
			if error_1[i] <= 0.05*last_desired_pose_2:
				final_settling_time_2 = self.joint2_actual_timestamps[i]
				break
		average_error_2 = sum(error_2)/len(error_2)
		max_error_2 = max(error_2)
		final_settling_time_2_insec= final_settling_time_2/ 1e9
		print("Final settling time in sec: ", final_settling_time_2_insec)
		print("Average tracking error: ", average_error_2)
		print("Maximum tracking error: ", max_error_2)

		# writing the results to a text file
		with open(os.path.join(os.path.expanduser('~'), 'hw3_ws', 'New_PID_results.txt'), 'w')  as file:
			file.write("Final settling time for joint1: {}\n".format(final_settling_time_1_insec))
			file.write("Average tracking error for joint1: {}\n".format(average_error_1))
			file.write("Maximum tracking error for joint1: {}\n".format(max_error_1))
			file.write("Final settling time for joint2: {}\n".format(final_settling_time_2_insec))
			file.write("Average tracking error for joint2: {}\n".format(average_error_2))
			file.write("Maximum tracking error for joint2: {}\n".format(max_error_2))


   

if __name__ == '__main__':
	rospy.init_node('hw3_plot_command_vs_actual')
	p = Plot()

	start_time = time.time()
	while time.time() - start_time < 8:
		time.sleep(1)
	p.calc_measures()
	p.save_results()

	
	
