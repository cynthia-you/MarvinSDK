#ifndef _MARVIN_KINE_H_
#define _MARVIN_KINE_H_

#include "DCAL.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#ifdef __cplusplus
extern "C" {
#endif

  bool OnInitKine_MARVINKINE(long serial, char* MARVINKINE_file);
  bool OnKine(double joints[7], double pg[4][4]);
  bool OnInvKine(double pg[4][4], double ref_joints[7], double return_joints[7], unsigned char* IsOutRange, unsigned char* Is123Deg, unsigned char* Is567Deg);
  bool OnInvKineDir(double pg[4][4], double ref_joints[7], double dir[3],double return_joints[7], unsigned char* IsOutRange, unsigned char* Is123Deg, unsigned char* Is567Deg);
  bool OnInvKine_NSP(double nsp_angle, double ref_joints[7], double return_joints[7], unsigned char* IsOutRange, unsigned char* Is123Deg, unsigned char* Is567Deg);
  bool OnInvKine_NSP_test(double pg[4][4],double nsp_angle, double ref_joints[7], double return_joints[7], unsigned char* IsOutRange, unsigned char* Is123Deg, unsigned char* Is567Deg);
  void OnInvKineRange_Cross67(double Joint67[2],double RetBound67[2]);
  bool OnJacob(double joints[7], double jacob[6][7]);



#ifdef __cplusplus
}
#endif


















#endif

