#include "MarvinKine.h"
#include "stdio.h"
#include "stdlib.h"

int main()
{
	long serial = 1;
	char* path = "/home/fusion/projects/FX_APP_LINUX_MARVIN/MARVIN_SDK_V1003/MarvinKine/MARVINKINE_CONFIG";
	OnInitKine_MARVINKINE(serial, path);

	double joints[7]={10.,20.,30.,40.,50.,60.,70.};
	double pg[4][4]={{0}};


	OnKine(joints,pg);

	printf("aaa\n");
	printf("FK mat=[[%.3f,%.3f,%.3f,%.3f],[%.3f,%.3f,%.3f,%.3f],[%.3f,%.3f,%.3f,%.3f],[%.3f,%.3lf,%.3f,%.3f]]\n",
		pg[0][0],pg[0][1],pg[0][2],pg[0][3],
		pg[1][0],pg[1][1],pg[1][2],pg[1][3],
		pg[2][0],pg[2][1],pg[2][2],pg[2][3],
		pg[3][0],pg[3][1],pg[3][2],pg[3][3]);


	printf("1");
	double ref_joints[7]={10.,20.,30.,40.,50.,60.,70.};
	double return_joints[7]={0,0,0,0,0,0,0};
	unsigned char IsOutRange=0;
	unsigned char Is123Deg=0;
	unsigned char Is567Deg=0;
	double mat4x4[4][4];
    memcpy(mat4x4, pg, sizeof(pg));
	
	printf("2");

	OnInvKine(mat4x4, ref_joints, return_joints, &IsOutRange, &Is123Deg, &Is567Deg);
	printf("ik joints=[%.3f, %.3f,%.3f,%.3f,%.3f,%.3f,%.3f]\n",
	return_joints[0],return_joints[1],return_joints[2],return_joints[3],return_joints[4],return_joints[5],return_joints[6]);

	// ik optimize  1:可调整方向
	double dir[3]={1,0,0};
	OnInvKineDir(mat4x4, ref_joints, dir,return_joints, &IsOutRange, &Is123Deg, &Is567Deg);
	printf("ikDir joints=[%.3f, %.3f,%.3f,%.3f,%.3f,%.3f,%.3f]\n",
	return_joints[0],return_joints[1],return_joints[2],return_joints[3],return_joints[4],return_joints[5],return_joints[6]);


	// ik optimize  2：可调臂角
	double nsp_angle = 10;
	OnInvKine_NSP(nsp_angle, ref_joints,return_joints, &IsOutRange, &Is123Deg, &Is567Deg);
	printf("ik_nsp joints=[%.3f, %.3f,%.3f,%.3f,%.3f,%.3f,%.3f]\n",
	return_joints[0],return_joints[1],return_joints[2],return_joints[3],return_joints[4],return_joints[5],return_joints[6]);

	// 防止67关节超限碰撞和计算阻尼
	double Joint67[2]={170,170};
	double RetBound67[2]={0,0};
	OnInvKineRange_Cross67(Joint67,RetBound67);
	printf("ik bound 6 7 joints=[%.3f, %.3f]\n",RetBound67[0],RetBound67[1]);


	double oints[7]={10.,20.,30.,40.,50.,60.,70.};
	double jacob[6][7]={{0}};
	OnJacob(joints,jacob);
	printf("jacob mat=[[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]]\n",
		jacob[0][0],jacob[0][1],jacob[0][2],jacob[0][3],jacob[0][4],jacob[0][5],jacob[0][6],
		jacob[1][0],jacob[1][1],jacob[1][2],jacob[1][3],jacob[1][4],jacob[1][5],jacob[1][6],
		jacob[2][0],jacob[2][1],jacob[2][2],jacob[2][3],jacob[2][4],jacob[2][5],jacob[2][6],
		jacob[3][0],jacob[3][1],jacob[3][2],jacob[3][3],jacob[3][4],jacob[3][5],jacob[3][6],
		jacob[4][0],jacob[4][1],jacob[4][2],jacob[4][3],jacob[4][4],jacob[4][5],jacob[4][6],
		jacob[5][0],jacob[5][1],jacob[5][2],jacob[5][3],jacob[5][4],jacob[5][5],jacob[5][6]);


	return 1;
}
