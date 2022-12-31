#!/usr/bin/env python  

import rospy
import tf
import numpy as np
import math

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse

class GeometricServices(object):
    def __init__(self):
        self.current_joints = None
        
        # Subscribe to /joint_states topic
        rospy.Subscriber("/joint_states", JointState, self.joints_callback)

        
	# Create service callback to tf translations
	self.tf_listener = tf.TransformListener()
	rospy.Service('get_tf_ee', Trigger, self.get_tf_ee_callback)
	
        # init your own kinematic chain offsets
        self.a_i = [0.1, -0.8 ,0 ,0.05 ,0.3, 0] 	
        self.alpha_i = [0.5*np.pi ,0 ,0.5*np.pi ,-0.5*np.pi, 0.5*np.pi ,0] 
        self.d_i = [0.8, 0.1, 0, 0.3, 0, 0.3] 
        self.nue_i = [0.5*np.pi,np.pi*0.5,np.pi*0.5,np.pi*0.5,np.pi*0.5,np.pi*0.5]


        # Create service callback to ee pose
        self.direct_translation = rospy.Service('get_ee_pose', Trigger, self.get_ee_pose_callback)


    def joints_callback(self, msg):
        self.current_joints= msg

    def get_tf_ee_callback(self, reqt):
        try:
            trans, rot = self.tf_listener.lookupTransform('/base_link','/end-effector-link', rospy.Time(0))
            message = 'translation {}, rotation {}'.format(trans, rot)
            return TriggerResponse(success=True, message=message)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return TriggerResponse(success=False, message='Failed, is TF running?')

    def get_ee_pose_callback(self, reqt):
        _, translation, rotation = self._get_ee_pose(self.current_joints)
        message = 'translation {} rotation {}'.format(translation, rotation)
        return TriggerResponse(success=True, message=message)

    def _get_ee_pose(self, joints):
        mat_total = np.identity(4)
	num_of_trans = len(self.current_joints.position)
	

	base_to_l0_t = self._generate_homogeneous_transformation(0,0,0.1,0)
	l6_to_ee_t = self._generate_homogeneous_transformation(0,0,0.35,0)
        from_base_transform = np.matmul(base_to_l0_t,mat_total)
	print(base_to_l0_t)
	print(l6_to_ee_t)

	for i in range(num_of_trans):
		T_i = self._generate_homogeneous_transformation(self.a_i[i],self.alpha_i[i],self.d_i[i],self.nue_i[i])
		#print (">>>>")
		#print (self.a_i[i],self.alpha_i[i],self.d_i[i],self.nue_i[i])
		#print (T_i)   
		mat_total = np.matmul(mat_total, T_i)

        from_base_transform = np.matmul(mat_total, l6_to_ee_t)
	#print (">>>>")
	#print (from_base_transform)

	translation = from_base_transform[:3, 3]
        rotation = from_base_transform[:3, :3]
        return from_base_transform, translation, rotation

    @staticmethod 
    def _generate_homogeneous_transformation(a, alpha, d, nue):

	si_nu=np.sin(nue)
	co_nu=np.cos(nue)
	si_al=np.sin(alpha)
	co_al=np.cos(alpha)

	A=np.array([[co_nu,-si_nu*co_al,si_nu*si_al,a*co_nu],
            	    [si_nu,co_nu*co_al,-co_nu*si_al,a*si_nu],
                    [0,si_al,co_al,d],
                    [0,0,0,1]])

        return A

    @staticmethod 
    def _rotation_to_quaternion(r):
        phi = np.arccos( (r[0,0]+r[1,1]+r[2,2]-1)/2 )
	real=np.cos(phi/2)
	radius =np.array([ r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1] ])
	if np.sin(phi) !=0:
		radius=radius/(2*np.sin(phi))
	x=np.sin(phi/2)*radius[0]
	y=np.sin(phi/2)*radius[1]
        z=np.sin(phi/2)*radius[2]
        return np.array([x, y, z, real])

    def get_geometric_jacobian(self, joints):
        # TODO8
        return None

    def get_analytical_jacobian(self, joints):
        geometric_jacobian = self.get_geometric_jacobian(joints)
        j_p = geometric_jacobian[:3, :]
        j_o = geometric_jacobian[3:, :]
        # TODO9
        return None

    def compute_inverse_kinematics(self, end_pose, max_iterations, error_threshold, time_step, initial_joints, k=1.):
        # TODO10
        return None

    @staticmethod 
    def _normalize_joints(joints):
        res = [j for j in joints]
        for i in range(len(res)):
            res[i] = res[i] + np.pi
            res[i] = res[i] % (2*np.pi)
            res[i] = res[i] - np.pi
        return np.array(res)



def convert_quanternion_to_zyz(q):
    x, y, z, w = q

    theta = math.atan2(w,x)-math.atan2(y,z)
    phi = math.acos(2*(w**2 + x**2) - 1)
    psi = math.atan2(w,x)+math.atan2(-y,z)

    return [theta , phi , psi]



def solve_ik(geometric_services):
    end_position = [-0.770, 1.562, 1.050]
    end_zyz = convert_quanternion_to_zyz([0.392, 0.830, 0.337, -0.207])
    end_pose = np.concatenate((end_position, end_zyz), axis=0)
    result = gs.compute_inverse_kinematics(end_pose, max_iterations=10000, error_threshold=0.001, time_step=0.001, initial_joints=[0.1]*6)
    print('ik solution {}'.format(result))


if __name__ == '__main__':
    rospy.init_node('hw2_services_node')
    gs = GeometricServices()
    #solve_ik(gs)
    rospy.spin()
