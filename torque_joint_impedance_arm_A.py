from fx_robot import Marvin_Robot
import time
from structure_data import DCSS
import math
import logging
# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

dcss=DCSS()

robot=Marvin_Robot()
robot.connect('192.168.1.190')

robot.log_switch('1') #全局日志开关
robot.local_log_switch('1') # 主要日志

'''设置扭矩模式'''
robot.clear_set()
robot.set_state(arm='A',state=3)#state=3扭矩模式
robot.send_cmd()

'''速度'''
robot.clear_set()
robot.set_vel_acc(arm='A',velRatio=10, AccRatio=10)
robot.send_cmd()

'''阻抗参数'''
robot.clear_set()
robot.set_cart_kd_params(arm='A',K=[3000,3000,3000,100,100,100,20], D =[0.1,0.1,0.1,0.3,0.3,1], type=2) #预设为参数最大上限，供参考。
robot.set_joint_kd_params(arm='A',K=[2,2,2,1.6, 1, 1, 1], D=[0.6,0.6,0.6,0.4,0.2,0.2,0.2])#预设为参数最大上限，供参考。
robot.set_force_control_params(arm='A',fcType=0, fxDirection=[0, 0, 1, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                        fcAdjLmt=10.)
robot.send_cmd()

'''选择阻抗模式'''
robot.clear_set()
robot.set_impedance_type(arm='A',type=1) #type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
robot.send_cmd()


'''订阅数据查看是否设置'''
sub_data=robot.subscribe(dcss)

logger.info(f'current state{sub_data['states'][0]['cur_state']}')
logger.info(f'cmd state:{sub_data['states'][0]['cmd_state']}')
logger.info(f'error code:{sub_data['states'][0]['err_code']}')
logger.info(f'set vel={sub_data["inputs"][0]["joint_vel_ratio"]}, acc={sub_data["inputs"][0]["joint_acc_ratio"]}')
logger.info(f'set card k={sub_data["inputs"][0]["cart_k"][:]}, d={sub_data["inputs"][0]["cart_k"][:]}')
logger.info(f'set joint k={sub_data["inputs"][0]["joint_k"][:]}, d={sub_data["inputs"][0]["joint_d"][:]}')
logger.info(f'set force fcType={sub_data["inputs"][0]["force_type"]}, '
             f'fxDirection={sub_data["inputs"][0]["force_dir"][:]}, '
             f'fcCtrlpara={sub_data["inputs"][0]["force_pidul"][:]}, '
             f'fcAdjLmt={sub_data["inputs"][0]["force_adj_lmt"]}')
logger.info(f'set impedance type={sub_data["inputs"][0]["imp_type"]}')


'''点位1'''
robot.clear_set()
joint_cmd_1=[0.,0.,0.,0.,0.,0.,5.]
robot.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
robot.send_cmd()

time.sleep(5) #预留运动时间
while 1:
    sub_data = robot.subscribe(dcss)

    if sub_data['outputs'][0]['low_speed_flag'] == 1:
        logger.info(
            f'A arm stop sign:{sub_data['outputs'][0]['low_speed_flag']} , now stop at {sub_data["outputs"][0]["fb_joint_pos"]}')
        break
logger.info(f'set joint={sub_data["inputs"][0]["joint_cmd_pos"]}')
logger.info(f'current joint={sub_data["outputs"][0]["fb_joint_pos"]}')


'''点位2'''
robot.clear_set()
joint_cmd_2=[0.,0.,0.,0.,20.,30.,10.]
robot.set_joint_cmd_pose(arm='A',joints=joint_cmd_2)
robot.send_cmd()
time.sleep(0.5) #模拟运动0.5秒后紧急情况需要急停

'''模拟急停和复位'''
robot.soft_stop('A')
time.sleep(2)
robot.clear_error(arm='A')


'''点位2'''
robot.clear_set()
joint_cmd_2=[0.,0.,0.,0.,20.,30.,10.]
robot.set_joint_cmd_pose(arm='A',joints=joint_cmd_2)
robot.send_cmd()
time.sleep(5) #预留运动时间
while 1:
    sub_data = robot.subscribe(dcss)

    if sub_data['outputs'][0]['low_speed_flag'] == 1:
        logger.info(
            f'A arm stop sign:{sub_data['outputs'][0]['low_speed_flag']} , now stop at {sub_data["outputs"][0]["fb_joint_pos"]}')
        break
logger.info(f'set joint={sub_data["inputs"][0]["joint_cmd_pos"]}')
logger.info(f'current joint={sub_data["outputs"][0]["fb_joint_pos"]}')


'''任务完成，下伺服 释放连接'''
robot.clear_set()
robot.set_state(arm='A',state=0)#state=0 下伺服
robot.send_cmd()
robot.release_robot()









