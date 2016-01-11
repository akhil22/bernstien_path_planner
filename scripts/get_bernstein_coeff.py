# Author:Akhil Nagariya
#        RRC IIIT Hyderabad
#!/usr/bin/env python
def get_bernstein_coeff(to,t,tf):
	to = float(to)
	t = float(t)
	tf = float(tf)
	B0= (1.0 - (t - to)/(-to + tf))**5
	B1=(5*(t - to)*((1 - (t - to)/(-to + tf)))**4)/(-to + tf)
	B2= (10*(t - to)**2*(1 - (t - to)/(-to + tf))**3)/(-to + tf)**2
	B3= (10*(t - to)**3*(1 - (t - to)/(-to + tf))**2)/(-to + tf)**3
	B4=(5*(t - to)**4*(1 - (t - to)/(-to + tf)))/(-to + tf)**4
	B5= (t - to)**5/(-to + tf)**5
	return [B0,B1,B2,B3,B4,B5]
