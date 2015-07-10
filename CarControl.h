/*
 * CarControl.h
 *
 *  Created on: 2014-9-5
 *      Author: alvin
 */

#ifndef CARCONTROL_H_
#define CARCONTROL_H_

#include "ModuleControl.h"
#include "PeakCan.h"
class CarControl: public ModuleControl {
private:
	PeakCan channel_car;
	double cur_value[2];//1-left 2-right
	double ref_value[2];
	VirtualFlag channelflag;
public:
	CarControl();
	CarControl(VirtualFlag channelflag);
	virtual ~CarControl();

	ReturnCode CarInitCtrl();//Init Car drive module
	ReturnCode CarInitMotor(int num);//Init motor
	ReturnCode CarPDOOn();//broadcast open pdo
	ReturnCode CarSyncMove();//sync move after send reference

	VirtualFlag GetVirtualFlag();
	ReturnCode InitModule(InitPhase phase);//interface inherit from super
	void RecvFeedback();//interface inherit from super
	void SetReference(int flag,double ref_value[]);//interface inherit from super
	void GetCurrent(int & flag,double cur_value[]);//interface inherit from super
};

#endif /* CARCONTROL_H_ */
