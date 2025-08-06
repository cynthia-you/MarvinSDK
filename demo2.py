from fx_robot import Marvin_Robot
from structure_data import DCSS
import time
dcss=DCSS()
robot=Marvin_Robot()
robot.connect('192.168.1.190')
sub_data=robot.subscribe(dcss)
print(sub_data)


