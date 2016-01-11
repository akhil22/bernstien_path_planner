#!/usr/bin/env python

from numpy import *
from get_integral_coeff import *
from get_bernstein_differentials import *
import matplotlib.pyplot as plt
from compute_x_coeff import *
def  get_way_points(inp):
	xo=float(inp[0]); 
	yo=float(inp[1]); 
	xf=float(inp[2]);  
	yf=float(inp[3]);  
	xdoto=float(inp[4]);  
	xddoto=float(inp[5]); 
	xdotf=float(inp[6]);  
	ko=float(inp[7]);   
	kdoto=float(inp[8]); 
	kddoto=float(inp[9]);
	kf=float(inp[10]);     
	kdotf=float(inp[11]);  
	to=float(inp[12]);      
	tf=float(inp[13]);  
	xc1=float(inp[14]) ; 
	yc1=float(inp[15]); 
	tc=float(inp[16]); 
	Ax = array([xo,xf,xdoto,xdotf,xddoto,xc1]);

	h1 = compute_x_coeff(Ax,to,tf);
	xo = h1[0];
	x1 = h1[1];
	x2 = h1[2];
	x3 = h1[3];
	x4 = h1[4];
	x5 = h1[5];


	t = tf;
	[coefotf,coef1tf,coef2tf,coef3tf,coef4tf,coef5tf]=get_integral_coeff(xo,x1,x2,x3,x4,x5,to,t,tf);
	t = tc;
	[coefotc1,coef1tc1,coef2tc1,coef3tc1,coef4tc1,coef5tc1 ]=get_integral_coeff(xo,x1,x2,x3,x4,x5,to,t,tf);

	t=to;
	[Bodoto,B1doto,B2doto,B3doto,B4doto,B5doto,Boddoto,B1ddoto,B2ddoto,B3ddoto,B4ddoto,B5ddoto]=get_bernstein_differentials(to,t,tf);
	t=tf;
	[Bodotf,B1dotf,B2dotf,B3dotf,B4dotf,B5dotf,Boddotf,B1ddotf,B2ddotf,B3ddotf,B4ddotf,B5ddotf]=get_bernstein_differentials(to,t,tf);


	A1 = array([[B1doto,B2doto,B3doto,B4doto],[B1dotf,B2dotf,B3dotf,B4dotf],[coef1tf,coef2tf,coef3tf,coef4tf],[coef1tc1,coef2tc1,coef3tc1,coef4tc1]]) 

	C1 = kdoto-Bodoto*ko-B5doto*kf;
	C2 = kdotf-Bodotf*ko-B5dotf*kf; 
	C3 = yf-ko*coefotf-kf*coef5tf-yo; 
	C4 = yc1-ko*coefotc1-kf*coef5tc1;

	
	C5 =  kddoto-Boddoto*ko-B5ddoto*kf;
	C = array([[C1],[C2],[C3],[C4]]);
	XX = dot(linalg.matrix_power(A1,-1),C)
	k1 = XX[0]
	k2 = XX[1];
	k3 = XX[2];
	k4 = XX[3];
#	f1 = [xo;x1;x2;x3;x4;xf];
#	f2 = [ko;k1;k2;k3;k4;kf];

#	f = [f1;f2];

	return  [xo,x1,x2,x3,x4,x5,array(ko),k1,k2,k3,k4,array(kf)]
