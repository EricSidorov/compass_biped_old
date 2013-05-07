#! /usr/bin/env python 
import os
class position(object):
	def __init__(self,x,y,z):
		self.x = x
		self.y = y
		self.z = z

def run(lst):
	os.system('/home/darpa2/fuerte_workspace/compass_biped/src/CB_Controller.py '+''.join(str(k)+' ' for k in lst))

	out = open('last_score.txt','r')
	pos = position(float(out.readline()[0:-1]),float(out.readline()[0:-1]),float(out.readline()[0:-1]))
	std = float(out.readline())
	return pos, std

if __name__ == '__main__':
	run([0.459680177297614, 1.515407604145842, 0.37998494269009325, 8.579124047648097,
    3.8384053826741744, -0.0187234217105575, 4.440453111652037, 0.118002443254526,
    0.8450118602929263, 1.2164286226267065, 0.16455889138828012, 0.14747194401033994])