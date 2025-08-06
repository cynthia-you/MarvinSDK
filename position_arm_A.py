from fx_robot import Marvin_Robot
import time
from structure_data import DCSS
import math
import logging
from structure_data import DCSS
dcss=DCSS()


# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级


def are_lists_approximately_equal(list1, list2):
    if len(list1) != len(list2):
        return False
    for a, b in zip(list1, list2):
        rounded_a = round(a, 1)
        rounded_b = round(b, 1)
        if rounded_a != rounded_b:
            return False
    return True

dcss=DCSS()

robot=Marvin_Robot()
robot.connect('192.168.1.190')

robot.log_switch('1') #全局日志开关
robot.local_log_switch('1') # 主要日志

'''设置位置模式和速度'''
robot.clear_set()
robot.set_state(arm='A',state=1)#state=1位置模式
robot.set_vel_acc(arm='A',velRatio=10, AccRatio=10)
robot.send_cmd()


'''订阅数据查看是否设置'''
sub_data=robot.subscribe(dcss)

logger.info(f'current state{sub_data['states'][0]['cur_state']}')
logger.info(f'cmd state:{sub_data['states'][0]['cmd_state']}')
logger.info(f'error code:{sub_data['states'][0]['err_code']}')

logger.info(f'set vel={sub_data["inputs"][0]["joint_vel_ratio"]}, acc={sub_data["inputs"][0]["joint_acc_ratio"]}')

'''点位1'''
robot.clear_set()
joint_cmd_1=[0.,0.,0.,0.,0.,0.,5.]
robot.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
robot.send_cmd()

time.sleep(5) #预留运动时间
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
logger.info(f'set joint={sub_data["inputs"][0]["joint_cmd_pos"]}')
logger.info(f'current joint={sub_data["outputs"][0]["fb_joint_pos"]}')


'''任务完成，下伺服 释放连接'''
robot.clear_set()
robot.set_state(arm='A',state=0)#state=0 下伺服
robot.send_cmd()
robot.release_robot()









