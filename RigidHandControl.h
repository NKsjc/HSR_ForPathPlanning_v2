/*
 * RigidHandControl.h
 *
 *  Created on: 2014-9-5
 *      Author: alvin
 */

#ifndef RIGIDHANDCONTROL_H_
#define RIGIDHANDCONTROL_H_

#include "ModuleControl.h"
#include "PeakCan.h"
class RigidHandControl: public ModuleControl {
private:
	double cur_value[4];
	double ref_value[4];
	double enc_value[4];
	double home_offset[4];
	int home_flag[4];
	PeakCan channel_hand;
	VirtualFlag channelflag;
public:
	RigidHandControl();
	virtual ~RigidHandControl();
	RigidHandControl(VirtualFlag channelflag);
	ReturnCode HandServeOn();//serve on
	ReturnCode HandSyncMove();//sync move

	ReturnCode HandServeOff();//serve off
	void HandSetOffset();//set home offset
	ReturnCode HandClearPosition();//clear encoder
	ReturnCode HandHome();//auto home

	VirtualFlag GetVirtualFlag();
	ReturnCode InitModule(InitPhase phase);//interface inherit from super
	void RecvFeedback();//interface inherit from super
	void SetReference(int flag,double ref_value[]);//interface inherit from super
	void GetCurrent(int & flag,double cur_value[]);//interface inherit from super
};

#endif /* RIGIDHANDCONTROL_H_ */
