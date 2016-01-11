#!/usr/bin/env python
# Author:Akhil Nagariya
#        RRC IIIT Hyderabad

import rospy
from numpy import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion
from get_bernstein_coeff import *
from get_bernstein_differentials import *
from get_integral_coeff import *
from compute_x_coeff import *
from geometry_msgs.msg import Twist
class GlobalPlanner:

	def __init__(self):
		
		rospy.init_node("global_planner",anonymous = True)
		
		self.x0 = 0.0               #x cordinate of initial position of the robot
		self.y0 = 0.0               #y cordinate of initial position of the robot 

		self.xf = 0.0               #x cordinate of the goal 
		self.yf = 0.0               #y cordinate of the goal

		self.vx0 = 0.0              #x component of initial velocity
		self.ax0 = 0.0              #x component of initial acceleration
		self.vxf = 0.0              #x component of final velocity of the robot

		self.k0 = 0.0               #tangent of the initial heading of the robot
		self.kdot0 = 0.0            #differential of the tangent of the initial heading of the robot
		self.kddot0 = 0.0           #double differential of the tangent of the initial heading of the robot
		self.kf = 0.0               #tangent of the final heading of the robot
		self.kdotf = 0.0            #differential of the tangent of the final heading of the robot 

		self.t0 = rospy.get_param("initial_time", 1.0)  #initial time
		self.tf = rospy.get_param("final_time", 10.0)   #final time
		self.tc = self.t0 + 0.5*(self.tf - self.t0)                     #middle time

		self.xc = rospy.get_param("center_x",1.5)
		self.yc = rospy.get_param("center_y",0.5)
		self.rate = rospy.get_param("rate",1)
		self.planner_rate = rospy.get_param("planner_rate",0.1)

		self.goal_flag = 0
		
		rospy.Subscriber("bernstein_goal",Pose,self.goal_pose_callback);
		rospy.Subscriber("robot_pose",Pose,self.robot_pose_callback);
		self.vel_publisher = rospy.Publisher("cmd_vel", Twist)
		
	def goal_pose_callback(self, msg):
		self.goal_flag = 1
		quat = msg.orientation
		angles = euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])
		
		self.kf = angles[2]
		self.xf = msg.position.x
		self.yf = msg.position.y

#		rospy.loginfo("goal = [x=%f y=%f th=%f]",msg.position.x,msg.position.y,self.kf);   
		return 
	
	def robot_pose_callback(self, msg):
		quat = msg.orientation
		angles = euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])
		
		self.k0 = angles[2]
		self.x0 = msg.position.x
		self.y0 = msg.position.y

#		rospy.loginfo("initial position of the robot = [x=%f y=%f th=%f]",msg.position.x,msg.position.y,self.k0);   
		return
	def move_robot(self):
		Ax = array([self.x0,self.xf,self.vx0,self.vxf,self.ax0,self.xc])
		xcoeff = compute_x_coeff(Ax,self.t0,self.tf)
		x0 = xcoeff[0]
		x1 = xcoeff[1]
		x2 = xcoeff[2]
		x3 = xcoeff[3]
		x4 = xcoeff[4]
		x5 = xcoeff[5]

		#get the coefficients to calculate y at t=tf;
		t = self.tf
		[coef0tf,coef1tf,coef2tf,coef3tf,coef4tf,coef5tf]=get_integral_coeff(x0,x1,x2,x3,x4,x5,self.t0,t,self.tf);
		
		# get the coefficients to calculate y at t=tc;
		t = self.tc
		[coef0tc,coef1tc,coef2tc,coef3tc,coef4tc,coef5tc]=get_integral_coeff(x0,x1,x2,x3,x4,x5,self.t0,t,self.tf);

		# get differentials of bernstein coeffs at t=t0:
		t=self.t0
		[B0doto,B1doto,B2doto,B3doto,B4doto,B5doto,Boddoto,B1ddoto,B2ddoto,B3ddoto,B4ddoto,B5ddoto]=get_bernstein_differentials(self.t0,t,self.tf);
		
		#get differentials of bernstein coeffs at t=tf:
		t=self.tf
		[B0dotf,B1dotf,B2dotf,B3dotf,B4dotf,B5dotf,Boddotf,B1ddotf,B2ddotf,B3ddotf,B4ddotf,B5ddotf]=get_bernstein_differentials(self.t0,t,self.tf);
		A1 = array([[B1doto,B2doto,B3doto,B4doto],[B1dotf,B2dotf,B3dotf,B4dotf],[coef1tf,coef2tf,coef3tf,coef4tf],[coef1tc,coef2tc,coef3tc,coef4tc]])
		C1 = self.kdot0-B0doto*self.k0-B5doto*self.kf
		C2 = self.kdotf-B0dotf*self.k0-B5dotf*self.kf
		C3 = self.yf-self.k0*coef0tf-self.kf*coef5tf-self.y0
		C4 = self.yc-self.k0*coef0tc-self.kf*coef5tc

		C = array([[C1],[C2],[C3],[C4]])
	     
		thcoeff = dot(linalg.matrix_power(A1,-1),C)
		num_points = int((self.tf - self.t0)/self.planner_rate+1)
                
		k0 = self.k0
		k1 = thcoeff[0]
		k2 = thcoeff[1]
		k3 = thcoeff[2]
		k4 = thcoeff[3]
		k5 = self.kf
		
		xt = [None] * num_points
		vxt = [None] * num_points
		kt = [None] * num_points
		kdott = [None] * num_points
		yt = [None] * num_points
		vyt = [None] * num_points
		wt = [None] * num_points	
		velt = [None] * num_points

		t = self.t0
		i = 0
		r = rospy.Rate(10);
		while (t <= self.tf):
			[ B0,B1,B2,B3,B4,B5] = get_bernstein_coeff(self.t0,t,self.tf);
			[Bodot,B1dot,B2dot,B3dot,B4dot,B5dot,Boddot,B1ddot,B2ddot,B3ddot,B4ddot,B5ddot]=get_bernstein_differentials(self.t0,t,self.tf);

			[coefot, coef1t, coef2t, coef3t, coef4t,coef5t ]=get_integral_coeff(x0,x1,x2,x3,x4,x5,self.t0,t,self.tf);  
			xt[i]   = B0*x0+B1*x1+B2*x2+B3*x3+B4*x4+B5*x5 
			vxt[i]  = Bodot*x0+B1dot*x1+B2dot*x2+B3dot*x3+B4dot*x4+B5dot*x5 
			kt[i] = B0*k0+B1*k1+B2*k2+B3*k3+B4*k4+B5*k5
			kdott[i]= Bodot*k0+B1dot*k1+B2dot*k2+B3dot*k3+B4dot*k4+B5dot*k5
			yt[i] = self.y0+k0*coefot+k1*coef1t+k2*coef2t+k3*coef3t+k4*coef4t+k5*coef5t
			vyt[i]=vxt[i]*kt[i]
			wt[i] = kdott[i]/(1+kt[i]**2); 
			velt[i]=sqrt(vxt[i]**2+vyt[i]**2);
			v_command = Twist()
			v_command.linear.x = vxt[i] 
			v_command.linear.y = vyt[i]
			v_command.linear.z = 0.0
			v_command.angular.x = 0.0
			v_command.angular.y = 0.0
			v_command.angular.z = wt[i]
			self.vel_publisher.publish(v_command)
			t = t+0.1
			i = i+1
			r.sleep()
		fig1 = plt.figure()
	        plt.plot(xt,yt,'r-')
		fig2 = plt.figure()
		plt.plot(linspace(self.t0,self.tf,num_points),velt,'g-')
		fig3 = plt.figure()
		plt.plot(linspace(self.t0,self.tf,num_points),wt,'b-')
		plt.show()
	def spin(self):
		r = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			if (self.goal_flag == 0):
				r.sleep()
				continue
			
			self.move_robot()
			return
def main():
	gp = GlobalPlanner()
	gp.spin()

if __name__ == '__main__':
	main()
