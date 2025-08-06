from fx_robot import Marvin_Robot
import time
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
time.sleep(0.5)

'''机器人运动前开始设置保存数据'''
cols=7
idx=[0,1,2,3,4,5,6,
     0,0,0,0,0,0,0,
     0,0,0,0,0,0,0,
     0,0,0,0,0,0,0,
     0,0,0,0,0,0,0]
rows=1000
robot.collect_data(targetNum=cols,targetID=idx,recordNum=rows)


'''运动'''
robot.clear_set()
joint_cmd_1=[0.,0.,0.,0.,0.,0.,5.]
robot.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
robot.send_cmd()

time.sleep(0.5)# 模拟运动时长

'''停止采集'''
robot.stop_collect_data()

'''保存采集数据的两种格式'''

'''linux'''
# path='/home/fusion/projects/FX_APP_LINUX_MARVIN/MARVIN_SDK_V1003/aa.txt'
'''win'''
path=r"D:\1_Master\9XsenseRobot\aaa.txt'"
robot.save_collected_data_to_path(path)

'''linux'''
# path='/home/fusion/projects/FX_APP_LINUX_MARVIN/MARVIN_SDK_V1003/aa.csv'
'''win'''
path=r"D:\1_Master\9XsenseRobot\aaa.csv'"
robot.save_collected_data_as_csv_to_path(path.encode('utf-8'))


'''释放机器人内存'''
robot.release_robot()

'''下伺服'''
robot.set_state(arm='A',state=0)#state=1位置模式


