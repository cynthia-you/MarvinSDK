# 天机-孚晞 机器人工具包 MarvinSDK
## 机器人型号： MARVIN人形双臂
## 版本： 1003
## 支持平台： LINUX 及 WINDOWS
## LINUX支持： 18.04 20.04 24.04
## 更新日期：2025-08

## MarvinSDK为上位机控制机器人（双臂系统）的二次开发工具包，提供以下功能：
(1) 1KHz 通信


(2) 控制状态切换


    ① 下使能
    ② 位置跟随模式
    ③ 位置PVT模式
    ④ 扭矩模式
        1) 关节阻抗控制/关节阻抗控制位置跟随
        2) 坐标阻抗控制/坐标阻抗控制位置跟随
        3) 力控制/力控制位置跟随

(3) 控制状态参数（1KHz）


    ① 参数
        1) 目标跟随速度加速度设定
        2) 关节阻抗参数设定
        3) 坐标阻抗参数设定
        4) 力控制参数设定
        5) 工具运动学/动力学参数设定
    ② 指令
        1) 位置跟随目标指令 
        2) 力控目标指令 

(4) 数据反馈和采集（1KHz）


    ① 实时反馈
        1) 位置
        2) 速度
        3) 外编位置
        4) 电流
        5) 传感器扭矩
        6) 摩檫力
        7) 轴外力
    ② 数据采集
        1) 针对实时反馈数据可选择多达35项数据进行实时采集。

(5) 参数获取和设置


    ① 统一接口以参数名方式获取和设置所有参数。

(6) 计算接口


    ① 运动学参数获取/本地保存/本地导入
    ② 动力学参数获取/本地保存/本地导入
    ③ 工具运动学动力学参数获取/本地保存/本地导入
    ④ 正逆运动学计算（含零空间）
    ⑤ 逆动力学计算


# 接口介绍
## 接口快速全览见[MarvinSDK.h](MarvinLib/MarvinSDK.h)
## 所有左右臂相关接口都是后缀_A或_B表示， _A 为左臂 _B 为右臂
### (1) 连接和释放运行内存
bool OnLinkTo(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4);

    DEMO:  
    OnLinkTo(192, 168,1,190)
    基于UDP 连接并不代表数据已经开始发送，只有在控制器接收到发送数据之后才会向上位机开始1000HZ的周期性状态数据发送。

bool OnRelease();

### (2) 系统及系统更新相关
long OnGetSDKVersion();

    获取SDK版本


bool OnUpdateSystem(char* local_path);

    更新系统，更新文件为本机本机绝对路径

bool OnDownloadLog(char* local_path);

    获取系统日志，下载到本机绝对路径

bool OnSendFile(char* local_file, char* remote_file);

    上传本机绝对路径下的文件到机器人控制器的绝对路径目录

bool OnRecvFile(char* local_file, char* remote_file);

    下载机器人控制器的绝对路径下的文件到指定的本机绝对路径目录
    local_file：本机绝对路径目录
    remote_file：机器人控制器的绝对路径下的文件

### (3) 系统日志开关
void OnLogOn();

    全局日志开， 日志信息将全部打印，包括1000HZ频率日志以及清空待发送数据缓冲区日志信息
void OnLogOff();

    全局日志关
void OnLocalLogOn();

    主要日志开，打印显示主要指令接口信息
void OnLocalLogOff();

    主要日志关

### (4) 急停、获取错误码和清错
void OnEMG_A();

void OnEMG_B();

void OnEMG_AB();

    两条手臂开单独软急停也可同时软急停

void OnGetServoErr_A(long ErrCode[7]);

void OnGetServoErr_B(long ErrCode[7]);

    获取指定手臂的错误码，长度为7

void OnClearErr_A();

void OnClearErr_B();

    清除指定手臂的错误/复位

### (5) 实时订阅机器人数据
bool OnGetBuf(DCSS * ret);

    DCSS结构体及信息细节见FxRtCSDef.h


### (6) 配置机器人参数相关(参数名见robot.ini文件)
#### 读取整形和浮点参数信息：
long OnGetIntPara(char paraName[30],long * retValue);

    DEMO：获取左臂第一关节编码器单圈脉冲
    char name[30];
    long res;
    memset(name, 0, 30);
    sprintf(name, "R.A0.L%d.BASIC.EncRes", 0);
    if (CRobot::OnGetIntPara(name, &res) != 0)
    {
        AfxMessageBox("Get K Err");
        return;
    }
long OnGetFloatPara(char paraName[30],double * retValue);

    DEMO：获取右臂7个关节位置上限
    char name[30];
    long i;
    for ( i = 0; i < 7; i++)
    {
        memset(name, 0, 30);
        sprintf(name, "R.A0.L%d.BASIC.LimitPos",i);
        CRobot::OnGetFloatPara(name, &m_RunLmt[0].m_pos_u[i]);
    }


#### 设置整形和浮点参数信息：
long OnSetIntPara(char paraName[30],long setValue);

    设置整形配置参数
long OnSetFloatPara(char paraName[30], double setValue);

    设置浮点配置参数

#### 保存参数
long OnSavePara();

    返回值说明如下
    return -1/-2,               /////// 保存失败
    return ParaRetSerial,       /////// 保存参数的序号


### (7) 数据采集和保存相关

bool OnStartGather(long targetNum, long targetID[35], long recordNum);

    targetNum采集列数 （1-35列）
    targetID[35] 对应采集数据ID序号  
            左臂序号：
                0-6  	左臂关节位置 
                10-16 	左臂关节速度
                20-26   左臂外编位置
                30-36   左臂关节指令位置
                40-46	左臂关节电流（千分比）
                50-56   左臂关节传感器扭矩NM
                60-66	左臂摩擦力估计值
                70-76	左臂摩檫力速度估计值
                80-85   左臂关节外力估计值
                90-95	左臂末端点外力估计值
            右臂对应 + 100

    recordNum  采集行数 ，小于1000会采集1000行，设置大于一百万行会采集一百万行
    DEMO
    long targetNum = 2;
    long targetID[35] = {11, 31, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0}
    long recordNum = 500
    OnStartGather(targetNum, targetID, recordNum)

bool OnStopGather();

    中途停止采集
    在行数采集满后会自动停止采集,若需要中途停止采集调用本函数并等待1ms之后会停止采集。

bool OnSaveGatherData(char * path);

    path 保存到本机绝对路径
    在停止采集并且存在采集数据的情况下，可以将数据保存到文件
bool OnSaveGatherDataCSV(char* path);

    以CSV格式保存采集数据，保存到本机绝对路径
    在停止采集并且存在采集数据的情况下，可以将数据保存到文件

### (8) 末端485模组指令收发相关
### 注意，之前无指令发送到485，读取返回为0,使用逻辑为： 清缓存---发数据---读数据  或者： 读数据---清缓存---发数据---读数据
#### 清除指定手臂的缓存数据
bool OnClearChDataA();

bool OnClearChDataB();

#### 获取指定手臂的485数据
long OnGetChDataA(unsigned char data_ptr[63], long* ret_ch);

long OnGetChDataB(unsigned char data_ptr[63], long* ret_ch);
    
    data_ptr[63]为数据，最长可收63长度字节
    ret_ch：信息来源。 1：‘C’端; 2：com1; 3:com2
    
#### 设置指定手臂的485的指令数据
bool OnSetChDataA(unsigned char data_ptr[63], long size_int,long set_ch);

bool OnSetChDataB(unsigned char data_ptr[63], long size_int, long set_ch);

    data_ptr[63]：数据
    size_int：数据长度，不能超过63
    set_ch：发送通道。 1：‘C’端; 2：com1; 3:com2


### (9) 上传PVT文件 
bool OnSendPVT_A(char* local_file, long serial);

bool OnSendPVT_B(char* local_file, long serial);

    local_file 本地文件绝对路径
    pvt_id     对应PVT路径号 （1-99）
    PVT文件格式见：PVT文件格式说明.docx

    

### (10) 运动相关指令发送  可以以1000HZ频率进行发送

bool OnClearSet();

    清空待发送数据缓冲区
bool OnSetSend();

    发送指令

    以下指令必须在OnClearSet()和中间OnSetSend()设置生效：
    ////×以下指令可以单条发送，也可以多条一起发送发×/////
    bool OnSetTargetState_A(int state);
    bool OnSetTool_A(double kinePara[6], double dynPara[10]);
    bool OnSetJointLmt_A(int velRatio, int AccRatio);
    bool OnSetJointKD_A(double K[7], double D[7]);
    bool OnSetCartKD_A(double K[7], double D[7], int type);
    bool OnSetDragSpace_A(int dgType);
    bool OnSetForceCtrPara_A(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt);
    bool OnSetJointCmdPos_A(double joint[7]);
    bool OnSetForceCmd_A(double force);
    bool OnSetPVT_A(int id);
    bool OnSetImpType_A(int type);
    bool OnSetTargetState_B(int state);
    bool OnSetTool_B(double kinePara[6], double dynPara[10]);
    bool OnSetJointLmt_B(int velRatio, int AccRatio);
    bool OnSetJointKD_B(double K[7], double D[7]);
    bool OnSetCartKD_B(double K[6], double D[6],int type);
    bool OnSetDragSpace_B(int dgType);
    bool OnSetForceCtrPara_B(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt);
    bool OnSetJointCmdPos_B(double joint[7]);
    bool OnSetForceCmd_B(double force);
    bool OnSetImpType_B(int type);
    bool OnSetPVT_B(int id);
    ////×以下指令可以单条发送，也可以多条一起发送发×/////

    DEMO:
    OnClearSet()   
    OnSetJointCmdPos_A(XXX) // 设置左臂目标关节位置
    OnSetJointCmdPos_B(XXX) // 设置右臂目标关节位置
    OnSetForceCmd_A(XXX)    // 设置左臂力控位置
    OnSetForceCmd_B(XXX)    // 设置右臂力控位置
    OnSetSend()

### (11) 设置指定手臂的目标状态
bool OnSetTargetState_A(int state);

bool OnSetTargetState_B(int state);
    
    state取值如下： 
    0,         //下伺服
    1,	       // 位置跟随
    2,		   // PVT
    3,		   // 扭矩


### (12) 设置指定手臂在扭矩模式下阻抗类型
bool OnSetImpType_A(int type);

bool OnSetImpType_B(int type);

    type取值如下：
    1,       // 关节阻抗
    2,       // 坐标阻抗
    3,       // 力控 
    需要在OnSetTargetState_A（3）状态

### （13）设置指定手臂的关节跟随速度/加速度
bool OnSetJointLmt_A(int velRatio, int AccRatio)

bool OnSetJointLmt_B(int velRatio, int AccRatio)

    velRatio 速度百分比， 全速100, 安全起见，调试期间设为10
    AccRatio 加速度百分比， 全速100, 安全起见，调试期间设为10

### （14）设置指定手臂的工具信息
bool OnSetTool_A(double kinePara[6], double dynPara[10]);

bool OnSetTool_B(double kinePara[6], double dynPara[10]);

    kinePara: 运动学参数 XYZABC 单位毫米和度
    dynPara:  动力学参数分别为 质量M  质心[3]:mx,my,mz 惯量I[6]:XX,XY,XZ,YY,YZ,ZZ

### （15）设置指定手臂的关节阻抗参数
bool OnSetJointKD_A(double K[7], double D[7])

bool OnSetJointKD_B(double K[7], double D[7])

    K 刚度 牛米/度
    D 阻尼 牛米/(度/秒)

### （16）设置指定手臂的坐标阻抗参数
bool OnSetCartKD_A(double K[7], double D[7], int type)

bool OnSetCartKD_B(double K[7], double D[7], int type)

    K[0]-k[2] 牛/毫米
    K[3]-k[6] 牛米/度
    K[6] 零空间总和K系数 牛米/度
    D[0]-D[2] 牛/(毫米/秒）
    D[3]-D[6] 牛米/(度/秒）

### （17）设置指定手臂的力控阻抗参数
bool OnSetForceCtrPara_A(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt)

bool OnSetForceCtrPara_B(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt)

    fcType 力控类型 
        0- -坐标空间力控
        1- 工具空间力控(暂未实现)
    fxDir力控方向，需要控制方向设1，目前只支持 X,Y,Z控制方向。 如控制X方向{1,0,0,0,0,0}
    fcCtrlPara 控制参数, 目前全0
    fcAdjLmt 允许的调节范围, 厘米



### （18）设置指定手臂的关节跟踪指令值
bool OnSetJointCmdPos_A(double joint[7])

bool OnSetJointCmdPos_B(double joint[7])

    joint指令角度  
    在位置跟随和扭矩模式下均有效


### （19）设置指定手臂的力控指令
bool OnSetForceCmd_A(double force)

bool OnSetForceCmd_B(double force)

    force目标力 单位N或者N×M


### （20）设置指定手臂的设置运行PVT指令
bool OnSetPVT_A(int id)

bool OnSetPVT_B(int id)

    id   运行指定id号的pvt路径
    需要在 OnSetTargetState_A（2）状态状态



### （21） 设置指定手臂的拖动空间
bool OnSetDragSpace_A(int dgType);

bool OnSetDragSpace_B(int dgType);

    dgType取值如下
    0,       //退出拖动模式
    1,       //关节空间拖动
    2,       //笛卡尔空间X方向拖动
    3,       //笛卡尔空间Y方向拖动
    4,       //笛卡尔空间Z方向拖动
    5,       //笛卡尔空间旋转方向拖动



# 案例脚本
## C++开发的使用编译见：[API_USAGE.txt](MarvinLib/API_USAGE.txt)
## 位置模式：[position_demo.cpp](MarvinLib/position_demo.cpp)
## 拖动：[drag_demo.cpp](MarvinLib/drag_demo.cpp)
## 扭矩关节阻抗：[torque_joint_impedance_demo.cpp](MarvinLib/torque_joint_impedance_demo.cpp)
## 扭矩迪卡尔阻抗：[torque_cart_impedance_demo.cpp](MarvinLib/torque_cart_impedance_demo.cpp)
## 扭矩力控阻抗：[torque_force_impedance_demo.cpp](MarvinLib/torque_force_impedance_demo.cpp)






# 天机-孚晞 机器人运动学计算接口
## 机器人型号： MARVIN人形双臂
## 支持平台： LINUX 及 WINDOWS
## LINUX支持： 18.04 20.04 24.04
## 更新日期：2025-08


### 工具包主要提供运动学相关功能。

## 接口介绍
## 接口快速全览见[MarvinKine.h](MarvinKine/MarvinKine.h)
###    1. 初始化运动学相关参数
bool OnInitKine_MARVINKINE(long serial, char* MARVINKINE_file)

    • Eg.:OnInitKine_MARVINKINE (1，“3.MARVINKINE”)
    • 进行计算之前需要选择机型并导入机械臂配置相关文件， 为本机绝对路径， 如 /home/fusion/projects/FX_APP_LINUX_MARVIN/MARVIN_SDK_V1003/MarvinKine/MARVINKINE_CONFIG
    • serial=0，SRS机型；serial=1，CCS机型

###    2. 计算正运动学
bool OnKine(double joints[7], double pg[4][4])

    • 输入七关节角度，输出为4*4的法兰末端位姿矩阵
###    3. 计算逆运动学
bool OnInvKine(double pg[4][4], double ref_joints[7], double return_joints[7], unsigned char* IsOutRange, unsigned char* Is123Deg, unsigned char* Is567Deg)

    • 输入目标点末端的位姿矩阵及当前关节角度作为参考教，输出逆运动学解。同时输出三个参考数值，分别用于判断是否超出位置可达空间、123关节是否发生奇异及567关节是否发生奇异。

###    4. 计算末端位姿不变、改变空间方向的逆运动学
bool OnInvKineDir(double pg[4][4], double ref_joints[7], double dir[3],double return_joints[7], unsigned char* IsOutRange, unsigned char* Is123Deg, unsigned char* Is567Deg);

    • 若使用3接口得到的逆运动学解的臂角不满足当前选解需求，可以通过该接口修改机械手臂的旋转方向。dir[3]为X Y Z 三个方向可调， 一次仅可调一个方向， 如调整X方向[1,0,0]。
    • 该接口不能单独使用，使用前需要先调用3接口。

###    5. 计算末端位姿不变、改变零空间的逆运动学
bool OnInvKine_NSP(double nsp_angle, double ref_joints[7], double return_joints[7], unsigned char* IsOutRange, unsigned char* Is123Deg, unsigned char* Is567Deg)

    • 若使用3接口得到的逆运动学解的臂角不满足当前选解需求，可以通过该接口修改臂角平面方向。输入nsp_angle的单位为度。
    • 该接口不能单独使用，使用前需要先调用3接口。

###    6. 计算CCS构型下六七关节距离自干涉边界上的最近六七关节角度
void OnInvKineRange_Cross67(double Joint67[2],double RetBound67[2])

    • 输入当前6、7关节角度，得到距离自干涉边界上最近的六七关节角度。

###    7. 计算雅可比矩阵
bool OnJacob(double joints[7], double jacob[6][7])

    • 输入关节角度，输出为6*7的雅可比矩阵

# 案例脚本
## C++开发的使用编译见：[API_USAGE.txt](MarvinKine/API_USAGE.txt)
## DEMO：[kine_demo.cpp](MarvinKine/kine_demo.cpp)[position_demo.cpp](MarvinLib/position_demo.cpp)
