#!/usr/bin/env python

import numpy as np
import rospy

from sensor_msgs.msg import JointState


class FixedJointsPublisher(object):
    def __init__(self, joint_names, joint_values):
        self.joint_names = joint_names
        self.joint_values = joint_values

        # Initialize the node
        rospy.init_node('fixed_joints_publisher')

        # Create publisher for fixed_joint_states topic to publish a fixed message
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=1)

    def publish_joints(self):
        new_msg = JointState()
        new_msg.name = self.joint_names
        new_msg.position = self.joint_values
        new_msg.header.stamp = rospy.Time.now()

        self.pub.publish(new_msg)


if __name__ == '__main__':
    joint_names = ['base_joint', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3']
    # joint_values = [0., 0., 0., 0., 0., 0., ]
    joint_values = [0.5 * np.pi] * 6
    publisher = FixedJointsPublisher(joint_names, joint_values)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
      publisher.publish_joints()
      rate.sleep()
    rospy.spin()
