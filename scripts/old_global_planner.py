#!/usr/bin/env python
# Author:Akhil Nagariya
#        RRC IIIT Hyderabad

import rospy
from numpy import *
import math
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion
from get_bernstein_coeff import *
from get_bernstein_differentials import *
from get_integral_coeff import *
from compute_x_coeff import *
from geometry_msgs.msg import Twist
from get_way_points import *
from get_robot_pose import *
from pert_traj import *
from calc_scale_single import *
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

		self.t0 = rospy.get_param("initial_time", 0.0)  #initial time
		self.tf = rospy.get_param("final_time", 10.0)   #final time
		self.tc = self.t0 + 0.5*(self.tf - self.t0)                     #middle time

		self.xc = rospy.get_param("center_x",5.0)
		self.yc = rospy.get_param("center_y",5.0)
		self.rate = rospy.get_param("rate",1)
		self.planner_rate = rospy.get_param("planner_rate",0.01)

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
		inp=[self.x0,self.y0,self.xf,self.yf,self.vx0,self.ax0,self.vxf,self.k0,self.kdot0,self.kddot0,self.kf,self.kdotf,self.t0,self.tf,self.xc,self.yc,self.tc];
		[xo,x1,x2,x3,x4,xf,ko,k1,k2,k3,k4,kf ] = get_way_points(inp);
		t=self.t0;
		i=0;
		i_pert=0; 
		npath=21; 
		perturbation_at_time=8.1;
		delt = 0.1
		robot_end = 0;
		t_start_pert = 0;
		time = zeros((self.tf-self.t0)/delt + 1);
		xrobo = zeros((self.tf-self.t0)/delt + 1);
		yrobo = zeros((self.tf-self.t0)/delt + 1);
		krobo = zeros((self.tf-self.t0)/delt + 1);
		xrobodot =zeros((self.tf-self.t0)/delt + 1);
		yrobodot = zeros((self.tf-self.t0)/delt + 1);
		krobodot = zeros((self.tf-self.t0)/delt + 1);
		xroboddot = zeros((self.tf-self.t0)/delt + 1);
		kroboddot = zeros((self.tf-self.t0)/delt + 1);
		wrobo1 = zeros(((self.tf-self.t0)/delt + 1,npath));
		vrobo1 = zeros(((self.tf-self.t0)/delt + 1,npath));
		xrobo1 = zeros(((self.tf-self.t0)/delt + 1,npath));
		yrobo1 = zeros(((self.tf-self.t0)/delt + 1,npath));
		xrobodot1 = zeros(((self.tf-self.t0)/delt + 1,npath));
		xroboddot1 = zeros(((self.tf-self.t0)/delt + 1,npath));
		yrobodot1 = zeros(((self.tf-self.t0)/delt + 1,npath));
		krobo1 = zeros(((self.tf-self.t0)/delt + 1,npath));
		krobodot1 = zeros(((self.tf-self.t0)/delt + 1,npath));
		kroboddot1 = zeros(((self.tf-self.t0)/delt + 1,npath));
		time_perturb_duration = zeros((self.tf-self.t0)/delt + 1);
		l = []
		while(t <= self.tf):
			time[i] = t
			output_robo = get_robot_pose( self.t0,t,self.tf,xo,x1,x2,x3,x4,xf,ko,k1,k2,k3,k4,kf,self.y0);
			xrobo[i]=output_robo[0];yrobo[i]=output_robo[1];krobo[i]=output_robo[2];xrobodot[i]=output_robo[3];yrobodot[i]=output_robo[4];krobodot[i]=output_robo[5];xroboddot[i]=output_robo[6];kroboddot[i]=output_robo[7];
			
			if(abs(t-perturbation_at_time)<0.00000111):
				t_start_pert=t; 
				robo_end=i;  
				bernp=pert_traj(t_start_pert,self.tf,xrobo[robo_end],yrobo[robo_end],self.xf,self.yf,xrobodot[robo_end],xroboddot[robo_end],yrobodot[robo_end],krobo[robo_end],krobodot[robo_end],kroboddot[robo_end],npath);

			if(t > perturbation_at_time ):
				time_perturb_duration[i_pert] = t			
			        output_robo_pert = numpy.zeros((npath,8))
			        for j in range(0,npath):
					output_robo_pert[j] = get_robot_pose(t_start_pert,t,self.tf,bernp[0,j],bernp[1,j],bernp[2,j],bernp[3,j],bernp[4,j],bernp[5,j],bernp[6,j],bernp[7,j],bernp[8,j],bernp[9,j],bernp[10,j],bernp[11,j],self.y0);
				xrobo1[i_pert]=output_robo_pert[:,0];yrobo1[i_pert]=output_robo_pert[:,1];krobo1[i_pert]=output_robo_pert[:,2];xrobodot1[i_pert]=output_robo_pert[:,3];yrobodot1[i_pert]=output_robo_pert[:,4];krobodot1[i_pert]=output_robo_pert[:,5];xroboddot1[i_pert]=output_robo_pert[:,6];kroboddot1[i_pert]=output_robo_pert[:,7];
#				xrobo1[i_pert] = l[1]
#				yrobo1[i_pert] = l[2]
#				xrobodot1[i_pert] = l[3]
#				xroboddot1[i_pert] = l[4]
#				yrobodot1[i_pert] = l[5]
#				krobo1[i_pert] = l[6]
#				krobodot1[i_pert] = l[7]
				wrobo1[i_pert]= [krobodot1[i_pert,j]/(1.0+krobo1[i_pert,j]**2) for j in range(0,npath)]
				vrobo1[i_pert]=[math.sqrt(xrobodot1[i_pert,j]**2+yrobodot1[i_pert,j]**2) for j in range(0,npath)] 
				i_pert=i_pert+1;

			i = i+1
			t = t+0.1
		fig1 = plt.figure()
	        plt.plot(xrobo,yrobo,'r+')
		fig2 = plt.figure()
		plt.plot(xrobo1,yrobo1)
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
