#! /usr/bin/env python
import roslib; roslib.load_manifest('compass_biped')
import CB_Controller
from GA import *
import copy
import os
import yaml
import rospy
import runner
class file_manager():
    def __init__(self):
        self._base_dir = os.path.join(roslib.packages.get_pkg_dir('compass_biped'),'populations')
        self._dir = self._base_dir
        self._filename = ''

    def set_dir(self):
        ok = False
        while not ok:
            tmp_dir =  raw_input('Enter batch name (saving directory): ')
            full_pth = os.path.join(self._base_dir,tmp_dir)
            if os.path.exists(full_pth):
                answer = raw_input('directory already exists, overwrite?')
                if answer in ['Y','y']:
                    ok = True
            if not os.path.exists(full_pth):
                os.makedirs(full_pth)
                ok = True
        self._dir = full_pth

    def set_file_name(self,name):
        self._filename = str(name)+'.yaml'

    def save(self, obj):
        pth = os.path.join(self._dir,self._filename)
        yaml.dump(obj,file(pth,'w'))

class compass_tester(GAtester):
    def Evaluate(self,seq):
        # print controller
        pos,std = runner.run(seq)
        fit1 = pos.x**2+pos.y**2
        fit2 = (1.0-std/fit1)
        return [fit1,fit2]

fm = file_manager();
fm.set_dir()
rospy.init_node('ga')
tester = compass_tester()
ga = GA(tester)

pop = Population()
nom_seq=[0.46,2,0.4,10,4,-0.02,4,0.12,0.85,0.95,0.18,0.15]
nominal = Genom(nom_seq,[float(k)/10 for k in nom_seq],[[0,2],[0,10],[0.25,0.6],[1,50],[0,10],[-0.1,0.1],[0,15],[0.06,0.2],[0.6,1.5],[0,5],[0.05,0.25],[0.05,0.25]])
pop.append(copy.deepcopy(nominal))
# pop = yaml.load(file('/home/darpa2/fuerte_workspace/compass_biped/populations/2_5_2013/1.yaml'))
for k in xrange(199):
    pop.append(nominal.Mutate())

for k in xrange(25):
    # print tester.Evaluate(nominal.GetSequence())
    print 'iteration: ', k
    prev,pop =  ga.Step(pop,50)
    fm.set_file_name(k)
    fm.save(prev)
    print 'best: ', prev.Best(), 'avg :', prev.Average()
