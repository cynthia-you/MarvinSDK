# 孚晞APP-PYTHON - 使用文档
SDK版本: v1003

机型： Marvin 人形双臂

平台： Linux(测试os: ubuntu24.04)

最后更新: 2025-07

FX_APP_MARVIN 是兼容Linux和Windows平台的天机人形机器人专用的python控制接口。

我们为您提供了详细的接口函数和用例，以及专用测试软件。

## API列表
  - SDK_version()
  - clear_485_cache(arm: str)
  - clear_error(arm: str)
  - clear_set()
  - collect_data(targetNum: int, targetID: list, recordNum: int)
  - connect(robot_ip: str)
  - download_sdk_log(log_path: str)
  - get_485_data(arm: str, com: int)
  - get_param(type: str, paraName: str)
  - get_servo_error_code(arm: str)
  - local_log_switch(flag: str)
  - log_switch(flag: str)
  - receive_file(local_path: str, remote_path: str)
  - release_robot()
  - save_collected_data_as_csv_to_path(path: str)
  - save_collected_data_to_path(path: str)
  - save_para_file()
  - send_cmd()
  - send_file(local_path: str, remote_path: str)
  - senf_pvt_file(arm: str, pvt_path: str, id: int)
  - set_485_data(arm: str, data: bytes, size_int: int, com: int)
  - set_cart_kd_params(arm: str, K: list, D: list, type: int)
  - set_drag_space(arm: str, dgType: int)
  - set_force_cmd(arm: str, f: float)
  - set_force_control_params(arm: str, fcType: int, fxDirection: list, fcCtrlpara: list, fcAdjLmt: float)
  - set_impedance_type(arm: str, type: int)
  - set_joint_cmd_pose(arm: str, joints: list)
  - set_joint_kd_params(arm: str, K: list, D: list)
  - set_param(type: str, paraName: str, value: float)
  - set_pvt_id(arm: str, id: int)
  - set_state(arm: str, state: int)
  - set_tool(arm: str, kineParams: list, dynamicParams: list)
  - set_vel_acc(arm: str, velRatio: int, AccRatio: int)
  - soft_stop(arm: str)
  - stop_collect_data()
  - subscribe(dcss)
  - update_SDK(sdk_path: str)


## API用法
### 首先将fx_robot的类函数实例化，
### 然后调用help()方法可一览所有方法，
### help(方法名)可详细了解方法的输入和返回， 里面写的详细！
    tj_robot = Marvin_Robot() #实例化
    tj_robot.help() #一览所有方法
    tj_robot.help('SDK_version') #查看SDK_version详情
    tj_robot.SDK_version() #调用该方法


## 绑定类方法
    一般的方法可以单独调用，但是部分控制指令需要有使用先后逻辑：
    以下指令设置必须在clear_set() 和send_cmd()之间才起效（忽略输入的测试值）：
            set_state(arm='A',state=3)
            set_drag_space(arm='A',dgType=1)
            set_impedance_type(arm='A',type=1)
            set_pvt_id(arm='A',id=1)
            set_card_kd_params(arm='A',K=[3000,3000,3000,60,60,60,0], D =[20,20,20,2,2,2,0], type=1)
            set_joint_kd_params(arm='A',K=[3,3,3,1.6, 1, 1, 1], D=[0.6,0.6,0.6,0.4,0.2,0.2,0.2])
            set_force_cmd(arm='A',f=1.)
            set_force_control_params(arm='A',fcType=0, fxDirection=[0, 0, 1, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
            fcAdjLmt=10.)
            set_vel_acc(arm='A',velRatio=1, AccRatio=1)
            set_tool(arm='A',kineParams=[0.,0.,0.,0.,0.,0.], dynamicParams=[0.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
            set_joint_cmd_pose(arm='A',joints=[0.,0.,0.,0.,0.,0.,6.])


    可以单条指令设置:
    clear_set()
    set_state(state=3)
    send_cmd()

    也可以多个指令一起设置：
    ''' ####  A arm ###'''
    clear_set()
    set_state(arm='A',state=3)
    set_drag_space(arm='A',dgType=1)
    set_impedance_type(arm='A',type=1)
    set_pvt_id(arm='A',id=1)
    set_card_kd_params(arm='A',K=[3000,3000,3000,60,60,60,0], D =[20,20,20,2,2,2,0], type=1)
    set_joint_kd_params(arm='A',K=[3,3,3,1.6, 1, 1, 1], D=[0.6,0.6,0.6,0.4,0.2,0.2,0.2])
    set_force_cmd(arm='A',f=1.)
    set_force_control_params(arm='A',fcType=0, fxDirection=[0, 0, 1, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
    fcAdjLmt=10.)
    set_vel_acc(arm='A',velRatio=1, AccRatio=1)
    set_tool(arm='A',kineParams=[0.,0.,0.,0.,0.,0.], dynamicParams=[0.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
    set_joint_cmd_pose(arm='A',joints=[0.,0.,0.,0.,0.,0.,6.])
    send_cmd()

    ''' ####  B arm ###'''
    clear_set()
    set_state(arm='B',state=3)
    set_drag_space(arm='B',dgType=1)
    set_impedance_type(arm='B',type=1)
    set_pvt_id(arm='B',id=1)
    set_card_kd_params(arm='B',K=[3000,3000,3000,60,60,60,0], D =[20,20,20,2,2,2,0], type=1)
    set_joint_kd_params(arm='B',K=[3,3,3,1.6, 1, 1, 1], D=[0.6,0.6,0.6,0.4,0.2,0.2,0.2])
    set_force_cmd(arm='B',f=1.)
    set_force_control_params(arm='B',fcType=0, fxDirection=[0, 0, 1, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                      fcAdjLmt=10.)
    set_vel_acc(arm='B',velRatio=1, AccRatio=1)
    set_tool(arm='B',kineParams=[0.,0.,0.,0.,0.,0.], dynamicParams=[0.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
    set_joint_cmd_pose(arm='B',joints=[0.,0.,0.,0.,0.,0.,6.])
    send_cmd()




订阅机器人数据见demo2.py, 数据含义见structure_data.py 其中返回数据为嵌套字典：

    result {
        'para_name': ['Marvin_sub_data'], 
        'states': [{'cur_state': 0, 'cmd_state': 0, 'err_code': 0}, {'cur_state': 0, 'cmd_state': 0, 'err_code': 0}],
        'outputs': [
                    {'frame_serial': 0, 
                     'tip_di': b'\x00',
                    'low_speed_flag': b'\x00', 
                     'fb_joint_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                     'fb_joint_vel': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                     'fb_joint_posE': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                     'fb_joint_cmd': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                     'fb_joint_cToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                    'fb_joint_sToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    'fb_joint_them': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                    'est_joint_firc': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                    'est_joint_firc_dot': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                    'est_joint_force': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                    'est_cart_fn': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}, 
                {'frame_serial': 0, 'tip_di': b'\x00', 'low_speed_flag': b'\x00', 'fb_joint_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'fb_joint_vel': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'fb_joint_posE': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'fb_joint_cmd': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'fb_joint_cToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'fb_joint_sToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'fb_joint_them': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'est_joint_firc': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'est_joint_firc_dot': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'est_joint_force': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'est_cart_fn': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
        ], 
        'inputs': [
                {'rt_in_switch': 0, 'imp_type': 0, 'in_frame_serial': 0, 'frame_miss_cnt': 0, 'max_frame_miss_cnt': 0, 'sys_cyc': 0, 'sys_cyc_miss_cnt': 0, 'max_sys_cyc_miss_cnt': 0, 'tool_kine': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'tool_dyn': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'joint_cmd_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'joint_vel_ratio': 0, 'joint_acc_ratio': 0, 'joint_k': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'joint_d': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'drag_sp_type': 0, 'drag_sp_para': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'cart_kd_type': 0, 'cart_k': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'cart_d': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'cart_kn': 0.0, 'cart_dn': 0.0, 'force_fb_type': 0, 'force_type': 0, 'force_dir': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'force_pidul': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'force_adj_lmt': 0.0, 'force_cmd': 0.0, 'set_tags': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 'update_tags': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 'pvt_id': 0, 'pvt_id_update': 0, 'pvt_run_id': 0, 'pvt_run_state': 0}, 
                {'rt_in_switch': 0, 'imp_type': 0, 'in_frame_serial': 0, 'frame_miss_cnt': 0, 'max_frame_miss_cnt': 0, 'sys_cyc': 0, 'sys_cyc_miss_cnt': 0, 'max_sys_cyc_miss_cnt': 0, 'tool_kine': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'tool_dyn': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'joint_cmd_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'joint_vel_ratio': 0, 'joint_acc_ratio': 0, 'joint_k': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'joint_d': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'drag_sp_type': 0, 'drag_sp_para': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'cart_kd_type': 0, 'cart_k': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'cart_d': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'cart_kn': 0.0, 'cart_dn': 0.0, 'force_fb_type': 0, 'force_type': 0, 'force_dir': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'force_pidul': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'force_adj_lmt': 0.0, 'force_cmd': 0.0, 'set_tags': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 'update_tags': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 'pvt_id': 0, 'pvt_id_update': 0, 'pvt_run_id': 0, 'pvt_run_state': 0}], 
        'ParaName': [[]], 
        'ParaType': [0], 
        'ParaIns': [0], 
        'ParaValueI': [0], 
        'ParaValueF': [0.0], 
        'ParaCmdSerial': [0], 
        'ParaRetSerial': [0]
        }


# 实例demo
完整API：[demo1.py](demo1.py)
订阅数据：[demo2.py](demo2.py)
运动学：[demo3.py](demo3.py)

阻抗模式：
    1. 关节阻抗：[torque_joint_impedance_arm_A.py](torque_joint_impedance_arm_A.py)
    2. 笛卡尔阻抗：[torque_card_impedance_arm_A.py](torque_card_impedance_arm_A.py)
    3. 力控阻抗：[torque_force_impedance_arm_A.py](torque_force_impedance_arm_A.py)

位置模式：[position_arm_A.py](position_arm_A.py)

拖动模式：[drag_arm_A.py](drag_arm_A.py)

末端485读取设定：[demo_485_arm_A.py](demo_485_arm_A.py)



# 孚晞运动学 - 使用文档

## 可用方法:

    - fk(joints: list)
    - ik(mat4x4: list, ref_joints: list, isOutRange: int, Is123Deg: int, Is567Deg: int)
    - ik_dir(mat4x4: list, ref_joints: list, dir: list, isOutRange: int, Is123Deg: int, Is567Deg: int)
    - ik_nsp(nsp_angle: float, ref_joints: list, isOutRange: int, Is123Deg: int, Is567Deg: int)
    - ik_range_crosss67(joints67: list)
    - initial_marvin_config(serial: int, config_path: str)
    - joints2JacobMatrix(joints: list)


# 使用
    kk = Marvin_Kine()  # 实例化
    kk.help()  # 查看方法
    # kk.help('fk')
    kk.initial_marvin_config(serial=1,
                             config_path=os.path.join(current_path, 'MarvinKine/MARVINKINE_CONFIG'))  # 本机绝对路径
    '''正逆解可相互验证，一组角度值正解的4*4矩阵逆解出来关节角度不变'''
    fk_mat = kk.fk(joints=[10.,20.,30.,40.,50.,60.,70.]) #正解
    ik_joints = kk.ik(mat4x4=fk_mat, ref_joints=[20., 20., 30., 40., 50., 60., 70.], isOutRange=0, Is123Deg=0,
                      Is567Deg=0) #逆解
    ik_dir_joints = kk.ik_dir(mat4x4=fk_mat, ref_joints=[20., 20., 30., 40., 50., 60., 70.], dir=[1.,0.,0.],isOutRange=0, Is123Deg=0,
                  Is567Deg=0) #逆解
    '''测试随机角平面旋转的角度下优化的解'''
    import random
    for i in range(10):
        print(f'iter:{i}')
        nsp_angle=random.uniform(0,10.)#调整10度
        nsp_ik_koints = kk.ik_nsp(nsp_angle=nsp_angle, ref_joints=[20., 20., 30., 40., 50., 60., 70.], isOutRange=0,
                              Is123Deg=0,
                              Is567Deg=0)

    bound = kk.ik_range_crosss67(joints67=[60., 70.])
    jacob_mat = kk.joints2JacobMatrix(joints=[10., 20., 30., 40., 50., 60., 70.])

或见：[demo3.py](demo3.py)



        



    