from fx_robot import Marvin_Robot
import time
import logging
# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级


robot=Marvin_Robot()
robot.connect('192.168.1.190')

robot.log_switch('1') #全局日志开关
robot.local_log_switch('1') # 主要日志


'''获取端口“C”的485数据'''
# 如果先前无指令写入，读到的信息长度就是0
data_len=robot.get_485_data('A',1)
logger.info(f'data size :{data_len}')

''' 发送485数据前，先清缓存'''
time.sleep(1)
robot.clear_485_cache('A')
time.sleep(1)

'''发送数据到485的“C”端'''
#demo 1: 正常数据
sample_data =  b"\x00" * 2
success = robot.set_485_data('A',sample_data, len(sample_data), 1)
logger.info(f"设置结果: {'成功' if success else '失败'}")


'''获取端口“C”的485数据'''
# 如果先前无指令写入，读到的信息长度就是0
data_len=robot.get_485_data('A',1)
logger.info(f'data size :{data_len}')

# #demo 2: 发送数据到485的“C”端,数据超过63
# try:
#     large_data = b"x" * 300  # 300字节数据
#     robot.set_485_data('A',large_data, len(large_data), 1)
# except ValueError as e:
#     print(f"捕获预期错误: {e}")





