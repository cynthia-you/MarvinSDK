from fx_robot import Marvin_Robot
import time
from structure_data import DCSS
dcss=DCSS()

robot=Marvin_Robot()
''' '''
# robot.help()
robot.connect('192.168.1.190')
robot.log_switch('1') #全局日志开关
robot.local_log_switch('0') # 主要日志
robot.SDK_version()

a_code=robot.get_servo_error_code(arm='A')
# print(f'a servo code :{a_code}')
b_code=robot.get_servo_error_code(arm='B')
# print(f'b servo code :{b_code}')
robot.clear_error(arm='A')
robot.clear_error(arm='B')

robot.soft_stop('A')
robot.soft_stop('B')
robot.soft_stop('AB')

''' ####  A arm ###'''
robot.clear_set()
robot.set_state(arm='A',state=3)
robot.set_drag_space(arm='A',dgType=1)
robot.set_impedance_type(arm='A',type=1)
robot.set_pvt_id(arm='A',id=1)
robot.set_card_kd_params(arm='A',K=[3000,3000,3000,60,60,60,0], D =[20,20,20,2,2,2,0], type=1)
robot.set_joint_kd_params(arm='A',K=[3,3,3,1.6, 1, 1, 1], D=[0.6,0.6,0.6,0.4,0.2,0.2,0.2])
robot.set_force_cmd(arm='A',f=1.)
robot.set_force_control_params(arm='A',fcType=0, fxDirection=[0, 0, 1, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                        fcAdjLmt=10.)
robot.set_vel_acc(arm='A',velRatio=1, AccRatio=1)
robot.set_tool(arm='A',kineParams=[0.,0.,0.,0.,0.,0.], dynamicParams=[0.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
robot.set_joint_cmd_pose(arm='A',joints=[0.,0.,0.,0.,0.,0.,6.])
robot.send_cmd()

''' ####  B arm ###'''
robot.clear_set()
robot.set_state(arm='B',state=3)
robot.set_drag_space(arm='B',dgType=1)
robot.set_impedance_type(arm='B',type=1)
robot.set_pvt_id(arm='B',id=1)
robot.set_card_kd_params(arm='B',K=[3000,3000,3000,60,60,60,0], D =[20,20,20,2,2,2,0], type=1)
robot.set_joint_kd_params(arm='B',K=[3,3,3,1.6, 1, 1, 1], D=[0.6,0.6,0.6,0.4,0.2,0.2,0.2])
robot.set_force_cmd(arm='B',f=1.)
robot.set_force_control_params(arm='B',fcType=0, fxDirection=[0, 0, 1, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                        fcAdjLmt=10.)
robot.set_vel_acc(arm='B',velRatio=1, AccRatio=1)
robot.set_tool(arm='B',kineParams=[0.,0.,0.,0.,0.,0.], dynamicParams=[0.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
robot.set_joint_cmd_pose(arm='B',joints=[0.,0.,0.,0.,0.,0.,6.])
robot.send_cmd()


sub_data=robot.subscribe(dcss)
print(sub_data)
robot.release_robot()



