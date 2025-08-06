#ifndef FX_RTCS_DEF_H_
#define FX_RTCS_DEF_H_

#include "FxType.h"

typedef enum
{
    ARM_STATE_IDLE = 0,
    ARM_STATE_POSITION = 1,
    ARM_STATE_DRAG_JOINT = 2,
    ARM_STATE_DRAG_CART = 3,
    ARM_STATE_DRAG_ZERO = 4,
    ARM_STATE_IMPD_JOINT = 5,
    ARM_STATE_IMPD_CART = 6,
    ARM_STATE_HYBRID = 7,
    ARM_STATE_PVT = 8,

    ARM_STATE_ERROR = 100,
    ARM_STATE_TRANS_TO_POSITION = 101,
    ARM_STATE_TRANS_TO_DRAG_JOINT = 102,
    ARM_STATE_TRANS_TO_DRAG_CART = 103,
    ARM_STATE_TRANS_TO_DRAG_ZERO = 104,
    ARM_STATE_TRANS_TO_IMPD_JOINT = 105,
    ARM_STATE_TRANS_TO_IMPD_CART = 106,
    ARM_STATE_TRANS_TO_HYBRID = 107,
    ARM_STATE_TRANS_TO_PVT = 108,
    ARM_STATE_TRANS_TO_IDLE = 109,
}ArmState;

typedef struct 
{
	FX_INT32   m_CurState;	///* 当前状态 */ ArmState
	FX_INT32   m_CmdState;	///* 指令状态 */ DCSSCmdType 0
	FX_INT32   m_ERRCode;	///* 错误码   */
}StateCtr ;

typedef struct 
{	
	FX_INT32 	m_OutFrameSerial;   	///* 输出帧序号   0 -  1000000 取模*/
	FX_FLOAT    m_FB_Joint_Pos[7];		///* 反馈关节位置 */   
	FX_FLOAT    m_FB_Joint_Vel[7];		///* 反馈关节速度 */
	FX_FLOAT    m_FB_Joint_PosE[7];		///* 反馈关节位置(外编) */
	FX_FLOAT    m_FB_Joint_Cmd[7];		///* 位置关节指令 */
	FX_FLOAT    m_FB_Joint_CToq[7];		///* 反馈关节电流 */
	FX_FLOAT    m_FB_Joint_SToq[7];		///* 反馈关节扭矩 */
	FX_FLOAT    m_FB_Joint_Them[7];		///* 反馈关节温度 */
	FX_FLOAT    m_EST_Joint_Firc[7];	///* 关节摩檫力估计值 */
	FX_FLOAT    m_EST_Joint_Firc_Dot[7];	///* 关节力扰动估计值微分 */
	FX_FLOAT    m_EST_Joint_Force[7];	///* 关节力扰动估计值 */
	FX_FLOAT    m_EST_Cart_FN[6];		///* 末端扰动估计值 */
}RT_OUT;

typedef struct 
{
	FX_INT32 m_RtInSwitch;  	 	///* 实时输入开关 用户实时数据 进行开关设置 0 -  close rt_in ;1- open rt_in*/

	FX_INT32 m_InFrameSerial;    	///* 输入帧序号   0 -  1000000 取模*/
	FX_INT16 m_FrameMissCnt;    	///* 丢帧计数*/
	FX_INT16 m_MaxFrameMissCnt;		///* 开 启 后 最 大 丢 帧 计 数 */

	FX_INT32 m_SysCyc;    			///* 0 -  1000000 */
	FX_INT16 m_SysCycMissCnt;		///* 实 时 性  Miss 计 数*/
	FX_INT16 m_MaxSysCycMissCnt;	///* 开 启 后 最 大 实 时 性Miss 计 数 */

	FX_FLOAT m_ToolKine[6];			///* 工 具 运 动 学 参 数 */ 1
	FX_FLOAT m_ToolDyn[10];			///* 工 具 动 力 学 参 数 */ 1
	
	FX_FLOAT m_Joint_CMD_Pos[7];	///* 关 节 位 置 指 令 */         7     
	FX_INT16 m_Joint_Vel_Ratio;		///* 关 节 速 度 限 制 百分比*/        2
	FX_INT16 m_Joint_Acc_Ratio;		///* 关 节 加 速 度 限 制  百分比*/    2
	
	FX_FLOAT m_Joint_K[7]; 			///* 关节阻抗刚度K指令*///3
	FX_FLOAT m_Joint_D[7]; 			///* 关节阻抗阻尼D指令*///3

	FX_INT32 m_ZeroSpType; 			///* 零空间类型*///5
	FX_FLOAT m_ZeroSpPara[8]; 		///* 零空间参数类型*///5
	
	FX_FLOAT m_Cart_K[6]; 			///* 坐标阻抗刚度K指令*///4
	FX_FLOAT m_Cart_D[6]; 			///* 坐标阻抗阻尼D指令*///4
    FX_FLOAT m_Cart_KN;             /// 4
    FX_FLOAT m_Cart_DN;             /// 4	

	FX_INT32  m_Force_FB_Type;		///* 力控反馈源类型*/
	FX_INT32  m_Force_Type;			///* 力控类型*///6
	FX_FLOAT  m_Force_Dir[6];		///* 力控方向6维空间方向*///6
	FX_FLOAT  m_Force_PIDUL[7];		///* 力控pid*///6
	FX_FLOAT  m_Force_AdjLmt;		///* 允许调节最大范围*///6
	
	FX_FLOAT  m_Force_Cmd;			///* 力控指令*///8
	
	FX_UCHAR m_SET_Tags[16];
	FX_UCHAR m_Update_Tags[16];

    FX_UCHAR m_PvtID;
    FX_UCHAR m_PvtID_Update;
    FX_UCHAR m_Pvt_RunID;    // 0: no pvt file; 1~249: user ; 250: TOOL_CALIB.txt; 251: LEFT_ARM_CALIB.txt; 252: RIGHT_ARM_CALIB.txt 
    FX_UCHAR m_Pvt_RunState; // 0: idle; 1: loading ; 2: running; 3: error

}RT_IN;

typedef struct 
{
	StateCtr m_State[2];
	RT_IN    m_In[2];
	RT_OUT	 m_Out[2];

	FX_CHAR m_ParaName[30]; // section_name.param_name
	FX_UCHAR m_ParaType; // 0: FX_INT32; 1: FX_DOUBLE; 2: FX_STRING
	FX_UCHAR m_ParaIns;  // DCSSCfgOperationType
	FX_INT32 m_ParaValueI; // FX_INT32 value
	FX_FLOAT m_ParaValueF; // FX_FLOAT value
	FX_INT16 m_ParaCmdSerial; // from PC
	FX_INT16 m_ParaRetSerial; // working: 0; finish: cmd serial; error cmd_serial + 100
}DCSS;

typedef enum
{
    DCSS_CMD_ARM0_TRANS_STATE = 101,
    DCSS_CMD_ARM0_SET_TOOL = 102,
    DCSS_CMD_ARM0_SET_JOINT_VA = 103,
    DCSS_CMD_ARM0_SET_JOINT_KD = 104,
    DCSS_CMD_ARM0_SET_CART_KD = 105,
    DCSS_CMD_ARM0_SET_ZERO_SP = 106,
    DCSS_CMD_ARM0_SET_FORCE_PIDUL = 107,
    DCSS_CMD_ARM0_SET_JOINT_CMD = 108,
    DCSS_CMD_ARM0_SET_FORCE_CMD = 109,
    DCSS_CMD_ARM0_SET_PVT_CMD = 110,

    DCSS_CMD_CFG_OPERATION = 150,

    DCSS_CMD_ARM1_TRANS_STATE = 201,
    DCSS_CMD_ARM1_SET_TOOL = 202,
    DCSS_CMD_ARM1_SET_JOINT_VA = 203,
    DCSS_CMD_ARM1_SET_JOINT_KD = 204,
    DCSS_CMD_ARM1_SET_CART_KD = 205,
    DCSS_CMD_ARM1_SET_ZERO_SP = 206,
    DCSS_CMD_ARM1_SET_FORCE_PIDUL = 207,
    DCSS_CMD_ARM1_SET_JOINT_CMD = 208,
    DCSS_CMD_ARM1_SET_FORCE_CMD = 209,
    DCSS_CMD_ARM1_SET_PVT_CMD = 210,

}DCSSCmdType;

typedef enum
{
    DCSS_CFG_OP_SET_INT32 = 101,
    DCSS_CFG_OP_SET_DOUBLE = 102,
    DCSS_CFG_OP_GET_INT32 = 103,
    DCSS_CFG_OP_GET_DOUBLE = 104,
    DCSS_CFG_OP_SAVE = 105,
}DCSSCfgOperationType;

#endif


