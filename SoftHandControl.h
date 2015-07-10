/*
 * SoftHandControl.h
 *
 *  Created on: 2014-9-5
 *      Author: alvin
 */

#ifndef SOFTHANDCONTROL_H_
#define SOFTHANDCONTROL_H_

#include "ModuleControl.h"
#include "PeakCan.h"
class SoftHandControl: public ModuleControl {
private:
	double cur_value[4];
	double ref_value[4];
	double enc_value[4];
	double home_offset[4];
	int home_flag[4];
	PeakCan channel_hand;
	VirtualFlag channelflag;
public:
	SoftHandControl();
	SoftHandControl(VirtualFlag channelflag);
	virtual ~SoftHandControl();

	ReturnCode HandInitCtrl();//Init serve
	ReturnCode HandSyncMove();//sync move
	ReturnCode HandServeOn();//enable serve
	ReturnCode HandServeOff();//disable serve
	ReturnCode HandPDOOn();
	ReturnCode HandHome();
	ReturnCode HandClearPosition();

	VirtualFlag GetVirtualFlag();
	ReturnCode InitModule(InitPhase phase);//interface inherit from super
	void RecvFeedback();//interface inherit from super
	void SetReference(int flag,double ref_value[]);//interface inherit from super
	void GetCurrent(int & flag,double cur_value[]);//interface inherit from super
};

#endif /* SOFTHANDCONTROL_H_ */
