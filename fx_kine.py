import ctypes
import inspect
from textwrap import dedent
import os
import math

from object_recognition_msgs.object_recognition_msgs_s__rosidl_typesupport_introspection_c import \
    destroy_ros_message_msg__srv__get_object_information__request

current_path = os.getcwd()


class Marvin_Kine:
    def __init__(self):
        """初始化机器人控制类"""
        import sys
        print(f'user platform: {sys.platform}')
        if sys.platform == 'win32':
            self.kine = ctypes.WinDLL('MarvinKine_dll/libMarvinKine.dll')
        else:
            self.kine = ctypes.CDLL('MarvinKine/libMarvinKine.so')

    def help(self, method_name: str = None) -> None:
        """显示帮助信息
        参数:method_name (str): 可选的方法名，显示特定方法的帮助信息
        """
        print(f"\n{' API 帮助 ':=^50}\n")

        # 获取所有公共方法
        methods = [
            (name, func)
            for name, func in inspect.getmembers(self, inspect.ismethod)
            if not name.startswith('_') and name != 'help'
        ]

        # 如果没有指定方法名，显示所有方法列表
        if method_name is None:
            print("可用方法:")
            for name, func in methods:
                # 获取函数签名
                signature = inspect.signature(func)
                # 获取参数列表
                params = []
                for param in signature.parameters.values():
                    param_str = param.name
                    if param.default is not param.empty:
                        param_str += f"={param.default!r}"
                    if param.annotation is not param.empty:
                        param_str += f": {param.annotation.__name__}"
                    if param.kind == param.VAR_POSITIONAL:
                        param_str = "*" + param_str
                    elif param.kind == param.VAR_KEYWORD:
                        param_str = "**" + param_str
                    elif param.kind == param.KEYWORD_ONLY:
                        param_str = "[kw] " + param_str
                    params.append(param_str)

                param_list = ", ".join(params)
                print(f"  - {name}({param_list})")

            print("\n使用 help('方法名') 获取详细帮助信息")
            print(f"{'=' * 50}")
            return

        # 显示特定方法的帮助
        method_dict = dict(methods)
        if method_name in method_dict:
            func = method_dict[method_name]
            doc = inspect.getdoc(func) or "没有文档说明"

            # 获取函数签名
            signature = inspect.signature(func)

            print(f"方法: {method_name}{signature}")
            print("\n" + dedent(doc))

            # 显示参数详细信息
            print("\n参数详情:")
            for param in signature.parameters.values():
                param_info = f"  {param.name}: "
                if param.annotation is not param.empty:
                    param_info += f"类型: {param.annotation.__name__}, "
                if param.default is not param.empty:
                    param_info += f"默认值: {param.default!r}"
                # param_info += f"类型: {_param_kind_to_str(param.kind)}"
                print(param_info)
        else:
            print(f"错误: 没有找到方法 '{method_name}'")

        print(f"{'=' * 50}")

    def _param_kind_to_str(kind):
        """将参数类型转换为可读字符串"""
        mapping = {
            inspect.Parameter.POSITIONAL_ONLY: "位置参数",
            inspect.Parameter.POSITIONAL_OR_KEYWORD: "位置或关键字参数",
            inspect.Parameter.VAR_POSITIONAL: "可变位置参数(*args)",
            inspect.Parameter.KEYWORD_ONLY: "仅关键字参数",
            inspect.Parameter.VAR_KEYWORD: "可变关键字参数(**kwargs)"
        }
        return mapping.get(kind, "未知参数类型")

    def initial_marvin_config(self, serial: int, config_path: str):
        ''' 初始化机械臂配置信息，解算运动学
        :param serial: 机器人构型：0：SRS，1：CCS
        :param config_path: 本地机械臂配置文件MARVINKINE_CONFIG,绝对路径.
                MARVINKINE_CONFIG如下：
                1007代表SRS， 1017代表CCS。
                以1007配置为例：
                前7行为各个关节的：alpha, a,d,theta, 关节负角度上限， 关节正角度上限， 速度最大值（度/秒）,加速度最大值（度/秒）
                第八行为末端法兰的信息：alpha, a,d,theta
                第九到12行为67关节自干涉的曲线信息，由于SRS构型无6和7关节自干涉，4x3的值都为零；CCS 不为零
                1007
                0.000000,0.000000,185.000000,0.000000,-170.000000,170.000000,180.000000,1800.000000,
                90.000000,0.000000,0.000000,0.000000,-120.000000,120.000000,180.000000,1800.000000,
                -90.000000,0.000000,290.000000,0.000000,-170.000000,170.000000,180.000000,1800.000000,
                90.000000,18.000000,0.000000,0.000000,-160.000000,160.000000,180.000000,1800.000000,
                -90.000000,-18.000000,280.000000,0.000000,-170.000000,170.000000,180.000000,1800.000000,
                90.000000,0.000000,0.000000,0.000000,-120.000000,120.000000,180.000000,1800.000000,
                -90.000000,0.000000,0.000000,0.000000,-170.000000,170.000000,180.000000,1800.000000,
                0.000000,0.000000,160.000000,0.000000,
                0,0,0,
                0,0,0,
                0,0,0,
                0,0,0,
                1017
                0.000000,0.000000,174.5,0.000000,-170.000000,170.000000,180.000000,1800.000000,
                90.000000,0.000000,0.000000,0.000000,-120.000000,120.000000,180.000000,1800.000000,
                -90.000000,0.000000,287.0000,0.000000,-170.000000,170.000000,180.000000,1800.000000,
                90.000000,18.000000,0.000000,180.000000,-160.000000,160.000000,180.000000,1800.000000,
                90.000000,18.000000,314.000000,-180.000000,-170.000000,170.000000,180.000000,1800.000000,
                90.000000,0.000000,0.000000,90.000000,-120.000000,120.000000,180.000000,1800.000000,
                -90.000000,0.000000,0.000000,90.000000,-170.000000,170.000000,180.000000,1800.000000,
                90.000000,0.000000,88.000000,90.000000,
                0.018004,-2.3205,108.4409,
                0.021823,2.5292,107.6665,
                -0.0084307,-1.3321,-100.2068,
                -0.014684,1.8496,-100.247,
        :return:
            bool True False
        '''
        if not os.path.exists(config_path):
            raise ValueError('NO CORRECT CONFIG PATH!')
        serial_long = ctypes.c_int(serial)
        self.marvin_config_path = config_path.encode('utf-8')
        path_char = ctypes.c_char_p(self.marvin_config_path)
        return self.kine.OnInitKine_MARVINKINE(serial_long, path_char);

    def fk(self, joints: list):
        '''关节角度正解到末端TCP位置和姿态XYZABC
        :param joints: list(7,1). 角度值
        :return:
            4x4的位姿矩阵，list(4,4)
        '''
        j0, j1, j2, j3, j4, j5, j6 = joints
        joints_double = (ctypes.c_double * 7)(j0, j1, j2, j3, j4, j5, j6)
        Matrix4x4 = ((ctypes.c_double * 4) * 4)
        pg = Matrix4x4()
        for i in range(4):
            for j in range(4):
                pg[i][j] = 1.0 if i == j else 0.0

        self.kine.OnKine.argtypes = [ctypes.POINTER(ctypes.c_double * 7),
                                     ctypes.POINTER((ctypes.c_double * 4) * 4)]
        self.kine.OnKine(ctypes.byref(joints_double), ctypes.byref(pg))
        fk_mat = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
        for i in range(4):
            for j in range(4):
                fk_mat[i][j] = pg[i][j]
        print('fk result, matrix:', fk_mat)
        return fk_mat

    def ik(self, mat4x4: list, ref_joints: list, isOutRange: int, Is123Deg: int, Is567Deg: int):
        '''末端位置和姿态XYZABC逆解到关节值
        :param mat4x4: list(4,4), 位置姿态4x4list.
        :param ref_joints: list(7,1),参考输入角度，约束构想接近参考解读，防止解出来的构型跳变。
        :param isOutRange:输入位姿是否超过可达空间范围。  0 or 1
        :param Is123Deg:123关节是否奇异。 0 or 1
        :param Is567Deg:567关节是否奇异。 0 or 1
        :return:
            joints(7,1)
        '''

        joints_double = ctypes.c_double * 7
        j0_, j1_, j2_, j3_, j4_, j5_, j6_ = ref_joints
        ref_joints_value = joints_double(j0_, j1_, j2_, j3_, j4_, j5_, j6_)

        target_joints_value = joints_double()

        isOutRange_char = ctypes.c_ubyte(isOutRange)
        Is123Deg_char = ctypes.c_ubyte(Is123Deg)
        Is567Deg_char = ctypes.c_ubyte(Is567Deg)

        # 创建输入矩阵 (4x4)
        pg = ((ctypes.c_double * 4) * 4)()

        for i in range(4):
            for j in range(4):
                pg[i][j] = mat4x4[i][j]

        self.kine.OnInvKine.argtypes = [ctypes.POINTER((ctypes.c_double * 4) * 4),
                                        ctypes.POINTER(ctypes.c_double * 7),
                                        ctypes.POINTER(ctypes.c_double * 7),
                                        ctypes.POINTER(ctypes.c_ubyte),
                                        ctypes.POINTER(ctypes.c_ubyte),
                                        ctypes.POINTER(ctypes.c_ubyte), ]

        self.kine.OnInvKine(ctypes.byref(pg),
                            ctypes.byref(ref_joints_value),
                            ctypes.byref(target_joints_value),
                            ctypes.byref(isOutRange_char),
                            ctypes.byref(Is123Deg_char),
                            ctypes.byref(Is567Deg_char))
        ik_joints = [0.] * 7
        for i in range(7):
            ik_joints[i] = target_joints_value[i]
        print(f'ik result, joints:', ik_joints)
        return ik_joints

    def ik_dir(self, mat4x4: list, ref_joints: list, dir: list, isOutRange: int, Is123Deg: int, Is567Deg: int):
        '''逆解优化：可调整方向
        :param mat4x4: list(4,4), 位置姿态4x4list.
        :param ref_joints: list(7,1),参考输入角度，约束构想接近参考解读，防止解出来的构型跳变。
        :param dir: list(3,1),逆解的调整方向，X，Y，Z三个方向选一个。如调整X方向的[1,0,0]
        :param isOutRange:输入位姿是否超过可达空间范围。  0 or 1
        :param Is123Deg:123关节是否奇异。 0 or 1
        :param Is567Deg:567关节是否奇异。 0 or 1
        :return:
            joints(7,1)
        '''

        joints_double = ctypes.c_double * 7
        j0_, j1_, j2_, j3_, j4_, j5_, j6_ = ref_joints
        ref_joints_value = joints_double(j0_, j1_, j2_, j3_, j4_, j5_, j6_)

        dir_double = ctypes.c_double * 3
        dir_0, dir_1, dir_2 = dir
        dir_value = dir_double(dir_0, dir_1, dir_2)

        target_joints_value = joints_double()

        isOutRange_char = ctypes.c_ubyte(isOutRange)
        Is123Deg_char = ctypes.c_ubyte(Is123Deg)
        Is567Deg_char = ctypes.c_ubyte(Is567Deg)

        # 创建输入矩阵 (4x4)
        pg = ((ctypes.c_double * 4) * 4)()

        for i in range(4):
            for j in range(4):
                pg[i][j] = mat4x4[i][j]

        self.kine.OnInvKineDir.argtypes = [ctypes.POINTER((ctypes.c_double * 4) * 4),
                                            ctypes.POINTER(ctypes.c_double * 7),
                                            ctypes.POINTER(ctypes.c_double * 3),
                                           ctypes.POINTER(ctypes.c_double * 7),
                                           ctypes.POINTER(ctypes.c_ubyte),
                                           ctypes.POINTER(ctypes.c_ubyte),
                                           ctypes.POINTER(ctypes.c_ubyte),]

        self.kine.OnInvKineDir(ctypes.byref(pg),
                               ctypes.byref(ref_joints_value),
                               ctypes.byref(dir_value),
                               ctypes.byref(target_joints_value),
                               ctypes.byref(isOutRange_char),
                               ctypes.byref(Is123Deg_char),
                               ctypes.byref(Is567Deg_char))
        ik_joints = [0.] * 7
        for i in range(7):
            ik_joints[i] = target_joints_value[i]
        print(f'ik_dir result, joints:', ik_joints)
        return ik_joints

    def ik_nsp(self, nsp_angle: float, ref_joints: list, isOutRange: int, Is123Deg: int, Is567Deg: int):
        '''逆解优化：可调臂角
        当IK得到的关节值未到预期，可以调用该接口调整臂角。
        :param nsp_angle: 臂角平面旋转的角度, 值范围0到360度。类似人手臂，手固定不动，大臂和小臂随意转动以避障或达到理想位姿
        :param ref_joints: 参考当前角度姿态解
        :param isOutRange:输入位姿是否超过可达空间范围。  0 or 1
        :param Is123Deg:123关节是否奇异。 0 or 1
        :param Is567Deg:567关节是否奇异。 0 or 1
        :return: 关节值 list(7,1)
        '''
        nsp_angle_double = ctypes.c_double(nsp_angle)
        if len(ref_joints) != 7:
            raise ValueError("ref_joints must have exactly 7 elements")
        ref_joints_array = (ctypes.c_double * 7)(*ref_joints)

        target_joints_array = (ctypes.c_double * 7)()

        isOutRange_char = ctypes.c_ubyte(isOutRange)
        Is123Deg_char = ctypes.c_ubyte(Is123Deg)
        Is567Deg_char = ctypes.c_ubyte(Is567Deg)

        self.kine.OnInvKine_NSP.argtypes = [
            ctypes.c_double,  # nsp_angle (值传递)
            ctypes.POINTER(ctypes.c_double),  # ref_joints (指针传递)
            ctypes.POINTER(ctypes.c_double),  # return_joints (指针传递)
            ctypes.POINTER(ctypes.c_ubyte),  # IsOutRange (指针传递)
            ctypes.POINTER(ctypes.c_ubyte),  # Is123Deg (指针传递)
            ctypes.POINTER(ctypes.c_ubyte)  # Is567Deg (指针传递)
        ]

        self.kine.OnInvKine_NSP(
            nsp_angle_double,  # 值传递 (直接传对象)
            ref_joints_array,  # 数组自动退化为指针
            target_joints_array,  # 数组自动退化为指针
            ctypes.byref(isOutRange_char),
            ctypes.byref(Is123Deg_char),
            ctypes.byref(Is567Deg_char)
        )

        # 将结果转换为Python列表
        ik_joints = [target_joints_array[i] for i in range(7)]
        print(f'ik_nsp result, joints: {ik_joints}')
        return ik_joints

    def ik_range_crosss67(self, joints67: list):
        '''防止67关节超限碰撞和计算阻尼
        CSS十字交叉构型机械臂，六七关节运动时，可能发生碰撞，因此逆解出的关节，将最后两个关节算一下干涉边界关节值。
        :param joints67: list(2,1). 当前六七关节值.
        :return: 当前六七关节边界关节值.
        '''
        double2 = ctypes.c_double * 2

        j0, j1 = joints67
        joints67_double = double2(j0, j1)

        RetBound67 = double2()
        RetBound67[0] = 0.
        RetBound67[1] = 0.

        self.kine.OnInvKineRange_Cross67.argtypes = [ctypes.POINTER(ctypes.c_double * 2),
                                                     ctypes.POINTER(ctypes.c_double * 2)]

        self.kine.OnInvKineRange_Cross67(ctypes.byref(joints67_double),
                                         ctypes.byref(RetBound67))
        bound67 = [RetBound67[0], RetBound67[1]]
        print(f'ik_range_crosss67 result, bound67:', bound67)

        return bound67

    def joints2JacobMatrix(self, joints: list):
        '''当前角度转成雅可比矩阵
        :param joints: list(7,1)
        :return: 雅可比矩阵6*7矩阵
        '''

        joints_double = ctypes.c_double * 7
        j0, j1, j2, j3, j4, j5, j6 = joints
        joints_value = joints_double(j0, j1, j2, j3, j4, j5, j6)

        jacob_mat = ((ctypes.c_double * 7) * 6)()
        for ii in range(6):
            for jj in range(7):
                jacob_mat[ii][jj] = 0

        self.kine.OnJacob.argtypes = [ctypes.POINTER(ctypes.c_double * 7),
                                      ctypes.POINTER((ctypes.c_double * 7) * 6)]

        self.kine.OnJacob(ctypes.byref(joints_value),
                          ctypes.byref(jacob_mat))

        result_jacob_mat = [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]]
        for i in range(6):
            for j in range(7):
                result_jacob_mat[i][j] = jacob_mat[i][j]
        print('joints2JacobMatrix result, jacobin matrix:', result_jacob_mat)
        return result_jacob_mat


def mat2xyzabc(T):  # 4*4位姿矩阵->位姿（x,y,z,rx,ry,rz）
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]

    R = T[0:3, 0:3]
    # 计算旋转角度 (绕 Z, Y, X)
    ry = math.asin(-R[2, 0])  # y的旋转
    if math.cos(ry) != 0:  # 防止除零错误
        rx = math.atan2(R[2, 1] / math.cos(ry), R[2, 2] / math.cos(ry))  # x的旋转
        rz = math.atan2(R[1, 0] / math.cos(ry), R[0, 0] / math.cos(ry))  # z的旋转
    else:
        rz = 0
        if R[2, 0] < 0:
            rx = math.atan2(R[0, 1], R[0, 2])
        else:
            rx = math.atan2(-R[0, 1], -R[0, 2])

    # 将弧度转换为度
    rx = rx * (180 / math.pi)
    ry = ry * (180 / math.pi)
    rz = rx * (180 / math.pi)

    return [x, y, z, rx, ry, rz]


def xyz2mat(pose: list):
    # 角度转弧度
    angx = math.radians(pose[3])  # roll (X轴)
    angy = math.radians(pose[4])  # pitch (Y轴)
    angz = math.radians(pose[5])  # yaw (Z轴)
    # 计算各角度的正弦/余弦
    sa = math.sin(angz)
    sb = math.sin(angy)
    sr = math.sin(angx)
    ca = math.cos(angz)
    cb = math.cos(angy)
    cr = math.cos(angx)

    mat = [[0.0] * 4 for _ in range(4)]
    mat[0][0] = ca * cb
    mat[0][1] = ca * sb * sr - sa * cr
    mat[0][2] = ca * sb * cr + sa * sr
    mat[1][0] = sa * cb
    mat[1][1] = sa * sb * sr + ca * cr
    mat[1][2] = sa * sb * cr - ca * sr
    mat[2][0] = -sb
    mat[2][1] = cb * sr
    mat[2][2] = cb * cr

    mat[0][-1] = pose[0]
    mat[1][-1] = pose[1]
    mat[2][-1] = pose[2]

    return mat


if __name__ == "__main__":
    kk = Marvin_Kine()  # 实例化
    kk.help()  # 查看方法
    # kk.help('fk')
    kk.initial_marvin_config(serial=1,
                             config_path=os.path.join(current_path, '/home/fusion/projects/FX_APP_LINUX_MARVIN/MARVIN_SDK_V1003/MarvinKine/MARVINKINE_CONFIG'))  # 本机绝对路径
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
    exit()

