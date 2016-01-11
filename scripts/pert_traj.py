#!/usr/bin/env python
import math
import numpy 
from get_way_points import *
from get_robot_pose import *
def pert_traj(t_start_pert,tf,xrobo,yrobo,x5,y5,xrobodot,xroboddot,yrobodot,krobo,krobodot,kroboddot,npath):
	xo=xrobo;yo=yrobo;xdoto=((xrobodot));xddoto=xroboddot;ydoto=yrobodot;kf=0;ko=krobo;kdoto=krobodot;kddoto=kroboddot;xdotf=0;kdotf=0;
	to=t_start_pert;
	x_pert1=((xo+x5)/2.0);
	if yo > y5:
		y_pert1=((y5-yo)/2.0);
	else:
		y_pert1=(((y5-yo)/2.0))
	t_pert1=to+0.5*(tf-to);
	t_pert2=to+0.5*(tf-to);
	min_pert = -1.0;
	max_pert = 1.0
	npath = 21;
	x_pert2 = numpy.zeros(npath)
	y_pert2 = numpy.zeros(npath)
	xrc0 =numpy.zeros(npath);xrc1=numpy.zeros(npath);xrc2=numpy.zeros(npath);xrc3=numpy.zeros(npath);xrc4=numpy.zeros(npath);xrc5=numpy.zeros(npath);krc0=numpy.zeros(npath);krc1=numpy.zeros(npath);krc2=numpy.zeros(npath);krc3=numpy.zeros(npath);krc4=numpy.zeros(npath);krc5=numpy.zeros(npath);
#	pert_listx = linspace(min_pert,max_pert,npath)
#	pert_listy = linspace(-4.0,4.0,npath)
#	pert_list = linspace(-1.0,1.0,npath)
	pert_listx = [0]*npath;
	pert_listy = linspace(-1.0,1.0,npath)
	j = 0
#	for i in range(0,npath):
#		x_pert2[j] = x_pert1+pert_listx[i]
#		y_pert2[j] = y_pert1+pert_listy[i]
#		j = j+1
	theta = math.atan2(y5 - yo,x5 - xo);
	for i in range(0,npath):
#		x_pert2[j] = x_pert1-pert_list[i]*math.sin(theta)
#		y_pert2[j] = y_pert1+pert_list[i]*math.cos(theta)
		x_pert2[j] = x_pert1+x_pert2[j]
		y_pert2[j] = y_pert1+pert_listy[i];
#		if yo < y5:
#			y_pert2[j] = abs(y_pert2[j])
			
		j = j+1
	for i1 in range(0,npath):
		input_for_perturbed_trajectories=[xo,yo,x5,y5,xdoto,xddoto,xdotf,ko,kdoto,kddoto,kf,kdotf,to,tf,x_pert2[i1],y_pert2[i1],t_pert2];
		[ xrc0[i1],xrc1[i1],xrc2[i1],xrc3[i1],xrc4[i1],xrc5[i1],krc0[i1],krc1[i1],krc2[i1],krc3[i1],krc4[i1],krc5[i1]]=get_way_points(input_for_perturbed_trajectories); 
	bernst_param=array([xrc0,xrc1,xrc2,xrc3,xrc4,xrc5,krc0,krc1,krc2,krc3,krc4,krc5]);
	return bernst_param
