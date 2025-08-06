#ifndef FX_SDKIF_H_ 
#define FX_SDKIF_H_
#include "Robot.h"

#ifdef CMPL_WIN
#define FX_DLL_EXPORT __declspec(dllexport) 
#endif

#ifdef CMPL_LIN
#define FX_DLL_EXPORT  
#endif
#ifdef __cplusplus
extern "C" {
#endif
	//////////////////////////////////////////////////////////////////////
	FX_DLL_EXPORT bool OnLinkTo(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4);
	FX_DLL_EXPORT bool OnRelease();
	//////////////////////////////////////////////////////////////////////
	
	FX_DLL_EXPORT long OnGetSDKVersion();
	FX_DLL_EXPORT bool OnUpdateSystem(char* local_path);
	FX_DLL_EXPORT bool OnDownloadLog(char* local_path);
	FX_DLL_EXPORT bool OnSendFile(char* local_file, char* remote_file);
	FX_DLL_EXPORT bool OnRecvFile(char* local_file, char* remote_file);
	FX_DLL_EXPORT bool OnGetBuf(DCSS * ret);
	
	FX_DLL_EXPORT void OnEMG_A();
	FX_DLL_EXPORT void OnEMG_B();
	FX_DLL_EXPORT void OnEMG_AB();
	////////////////////////////////////////////////////////////////////////////////////////////////
	FX_DLL_EXPORT void OnGetServoErr_A(long ErrCode[7]);
	FX_DLL_EXPORT void OnGetServoErr_B(long ErrCode[7]);
	////////////////////////////////////////////////////////////////////////////////////////////////
	FX_DLL_EXPORT void OnClearErr_A();
	FX_DLL_EXPORT void OnClearErr_B();
	FX_DLL_EXPORT void OnLogOn();
	FX_DLL_EXPORT void OnLogOff();
	FX_DLL_EXPORT void OnLocalLogOn();
	FX_DLL_EXPORT void OnLocalLogOff();
	////////////////////////////////////////////////////////////////////////////////////////////////
	FX_DLL_EXPORT bool OnSendPVT_A(char* local_file, long serial);
	FX_DLL_EXPORT bool OnSendPVT_B(char* local_file, long serial);
	FX_DLL_EXPORT long OnSetIntPara(char paraName[30],long setValue);
	FX_DLL_EXPORT long OnSetFloatPara(char paraName[30], double setValue);
	FX_DLL_EXPORT long OnGetIntPara(char paraName[30],long * retValue);
	FX_DLL_EXPORT long OnGetFloatPara(char paraName[30],double * retValue);
	FX_DLL_EXPORT long OnSavePara();

	FX_DLL_EXPORT bool OnStartGather(long targetNum, long targetID[35], long recordNum);
	FX_DLL_EXPORT bool OnStopGather();
	FX_DLL_EXPORT bool OnSaveGatherData(char * path);
	FX_DLL_EXPORT bool OnSaveGatherDataCSV(char* path);

	
	////////////////////////////////////////////////////////////////////////////////////////////////
	FX_DLL_EXPORT bool OnClearSet();
	
	FX_DLL_EXPORT bool OnSetTargetState_A(int state);
	FX_DLL_EXPORT bool OnSetTool_A(double kinePara[6], double dynPara[10]);
	FX_DLL_EXPORT bool OnSetJointLmt_A(int velRatio, int AccRatio);
	FX_DLL_EXPORT bool OnSetJointKD_A(double K[7], double D[7]);
	FX_DLL_EXPORT bool OnSetCartKD_A(double K[7], double D[7], int type);
	FX_DLL_EXPORT bool OnSetDragSpace_A(int dgType);
	FX_DLL_EXPORT bool OnSetForceCtrPara_A(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt);
	FX_DLL_EXPORT bool OnSetJointCmdPos_A(double joint[7]);
	FX_DLL_EXPORT bool OnSetForceCmd_A(double force);
	FX_DLL_EXPORT bool OnSetPVT_A(int id);
	FX_DLL_EXPORT bool OnSetImpType_A(int type);
	FX_DLL_EXPORT bool OnSetTargetState_B(int state);
	FX_DLL_EXPORT bool OnSetTool_B(double kinePara[6], double dynPara[10]);
	FX_DLL_EXPORT bool OnSetJointLmt_B(int velRatio, int AccRatio);
	FX_DLL_EXPORT bool OnSetJointKD_B(double K[7], double D[7]);
	FX_DLL_EXPORT bool OnSetCartKD_B(double K[6], double D[6],int type);
	FX_DLL_EXPORT bool OnSetDragSpace_B(int dgType);
	FX_DLL_EXPORT bool OnSetForceCtrPara_B(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt);
	FX_DLL_EXPORT bool OnSetJointCmdPos_B(double joint[7]);
	FX_DLL_EXPORT bool OnSetForceCmd_B(double force);
	FX_DLL_EXPORT bool OnSetImpType_B(int type);
	FX_DLL_EXPORT bool OnSetPVT_B(int id);

	FX_DLL_EXPORT bool OnSetSend();

	FX_DLL_EXPORT bool OnClearChDataA();
	FX_DLL_EXPORT bool OnClearChDataB();

	FX_DLL_EXPORT long OnGetChDataA(unsigned char data_ptr[63], long* ret_ch);
	FX_DLL_EXPORT bool OnSetChDataA(unsigned char data_ptr[63], long size_int,long set_ch);

	FX_DLL_EXPORT long OnGetChDataB(unsigned char data_ptr[63], long* ret_ch);
	FX_DLL_EXPORT bool OnSetChDataB(unsigned char data_ptr[63], long size_int, long set_ch);



#ifdef __cplusplus
}
#endif

#endif


