#!/usr/bin/env python
from math import *
def calc_scale_single(x1,y1,x1dot,y1dot,x0,y0,x0dot,y0dot,R):
	a=-(R**2)*(x0dot**2+y0dot**2)+((x0-x1)*y0dot+x0dot*(-y0+y1))**2;
	b=2*(-((x0-x1)*y0dot+x0dot*(-y0+y1))*(x1dot*(-y0+y1)+(x0-x1)*y1dot)+(R**2)*(x0dot*x1dot+y0dot*y1dot));
	c=(x1dot*(y0-y1)+(-x0+x1)*y1dot)**2-(R**2)*(x1dot**2+y1dot**2);
	root=[]
	s1 = 0
	s2 = 0
	if(((b**2)-4.0*a*c)<0):
		root=0;
	else:
		s1=(-b-sqrt(b**2-4.0*a*c))/(2.0*a);
		s2=(-b+sqrt(b**2-4.0*a*c))/(2.0*a);
		if(s1<=0):
			root=s2;
		else:
			root=s1;
	s = root
	return [s1,s2,a,b,c];
		
