#!/usr/bin/env python 
#Author Akhil Kumar Nagariya
# RRC IIIT Hyderabad 
from numpy import *
import math
from get_bernstein_coeff import *
from get_bernstein_differentials import *
from get_integral_coeff import *
def get_robot_pose(to,t,tf,xo,x1,x2,x3,x4,x5,ko,k1,k2,k3,k4,k5,yo):
	[ B0,B1,B2,B3,B4,B5] = get_bernstein_coeff(to,t,tf)
	[Bodot,B1dot,B2dot,B3dot,B4dot,B5dot,Boddot,B1ddot,B2ddot,B3ddot,B4ddot,B5ddot]=get_bernstein_differentials(to,t,tf)
	[coefot, coef1t, coef2t, coef3t, coef4t,coef5t ]=get_integral_coeff(xo,x1,x2,x3,x4,x5,to,t,tf)  
	xrobo = B0*xo+B1*x1+B2*x2+B3*x3+B4*x4+B5*x5 
	xrobodot=Bodot*xo+B1dot*x1+B2dot*x2+B3dot*x3+B4dot*x4+B5dot*x5 
	xroboddot=Boddot*xo+B1ddot*x1+B2ddot*x2+B3ddot*x3+B4ddot*x4+B5ddot*x5
	krobo=B0*ko+B1*k1+B2*k2+B3*k3+B4*k4+B5*k5
	krobodot=Bodot*ko+B1dot*k1+B2dot*k2+B3dot*k3+B4dot*k4+B5dot*k5 
	kroboddot=Boddot*ko+B1ddot*k1+B2ddot*k2+B3ddot*k3+B4ddot*k4+B5ddot*k5 
	yrobo = yo+ko*coefot+k1*coef1t+k2*coef2t+k3*coef3t+k4*coef4t+k5*coef5t
	yrobodot=xrobodot* krobo
	omega_robo= krobodot/(1+krobo**2) 
	velocity_robo=math.sqrt(xrobodot**2+yrobodot**2)
	output_robo=array([xrobo,yrobo,krobo,xrobodot,yrobodot,krobodot,xroboddot,kroboddot])
	return output_robo
	end
