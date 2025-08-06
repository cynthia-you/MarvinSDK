from fx_kine import Marvin_Kine
import os
current_path=os.getcwd()
kk = Marvin_Kine()  # 实例化
kk.help()  # 查看方法
# kk.help('fk')
kk.initial_marvin_config(serial=1,
                         config_path=os.path.join(current_path,
                                                  'MarvinKine/MARVINKINE_CONFIG'))  # 本机绝对路径
'''正逆解可相互验证，一组角度值正解的4*4矩阵逆解出来关节角度不变'''
fk_mat = kk.fk(joints=[10., 20., 30., 40., 50., 60., 70.])  # 正解
ik_joints = kk.ik(mat4x4=fk_mat, ref_joints=[20., 20., 30., 40., 50., 60., 70.], isOutRange=0, Is123Deg=0,
                  Is567Deg=0)  # 逆解
ik_dir_joints = kk.ik_dir(mat4x4=fk_mat, ref_joints=[20., 20., 30., 40., 50., 60., 70.], dir=[1., 0., 0.], isOutRange=0,
                          Is123Deg=0,
                          Is567Deg=0)  # 逆解
'''测试随机角平面旋转的角度下优化的解'''
import random

for i in range(10):
    print(f'iter:{i}')
    nsp_angle = random.uniform(0, 10.)  # 调整10度
    nsp_ik_koints = kk.ik_nsp(nsp_angle=nsp_angle, ref_joints=[20., 20., 30., 40., 50., 60., 70.], isOutRange=0,
                              Is123Deg=0,
                              Is567Deg=0)

bound = kk.ik_range_crosss67(joints67=[60., 70.])
jacob_mat = kk.joints2JacobMatrix(joints=[10., 20., 30., 40., 50., 60., 70.])







