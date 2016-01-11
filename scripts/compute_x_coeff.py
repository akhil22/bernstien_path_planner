# Author:Akhil Nagariya
#        RRC IIIT Hyderabad
#!/usr/bin/env python
from get_bernstein_coeff import *
from get_bernstein_differentials import *
from numpy import * 
def compute_x_coeff(inputs,t0,tf):
	x0  = float(inputs[0])
	xf  = float(inputs[1])
	xdot0  = float(inputs[2])
	xdotf  = float(inputs[3])
	xddot0  = float(inputs[4])
	xc1  = float(inputs[5])
	t0 = float(t0)
	tf = float(tf)
	tc1 = t0+0.5*(tf-t0)
	t=tc1;
	[Botc1,B1tc1,B2tc1,B3tc1,B4tc1,B5tc1] = get_bernstein_coeff(t0,t,tf)
	
	t=t0
	[Bodoto,B1doto,B2doto,B3doto,B4doto,B5doto,Boddoto,B1ddoto,B2ddoto,B3ddoto,B4ddoto,B5ddoto]=get_bernstein_differentials(t0,t,tf)
		
	t=tf
	[Bodotf,B1dotf,B2dotf,B3dotf,B4dotf,B5dotf,Boddotf,B1ddotf,B2ddotf,B3ddotf,B4ddotf,B5ddotf]=get_bernstein_differentials(t0,t,tf)

	A1 = [[B1tc1,B2tc1,B3tc1,B4tc1],[B1doto,B2doto,B3doto,B4doto],[B1ddoto,B2ddoto,B3ddoto,B4ddoto],[B1dotf,B2dotf,B3dotf,B4dotf]]

	C1 = xc1-x0*Botc1-xf*B5tc1  
	C2 = xdot0-Bodoto*x0-B5doto*xf
	C3 = xddot0-Boddoto*x0-B5ddoto*xf
	C4 = xdotf-Bodotf*x0-B5dotf*xf

	C = [[C1],[C2],[C3],[C4]]

	XX = dot(linalg.matrix_power(A1,-1),C)
	x1 = XX[0]
	x2 = XX[1]
	x3 = XX[2]
	x4 = XX[3]

	return [array([x0]),x1,x2,x3,x4,array([xf])];
