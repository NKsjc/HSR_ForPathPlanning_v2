/*
 * PVUControl.h
 *
 *  Created on: 2014-9-5
 *      Author: alvin
 */

#ifndef PVUCONTROL_H_
#define PVUCONTROL_H_

#include "ModuleControl.h"
#include "PeakCan.h"
class PVUControl: public ModuleControl {
private:
	PeakCan channel_pvu;
	int flag;
	double encoder1;
	double encoder2;
	double gyro;
	double dencoder2;
	VirtualFlag channelflag;
public:
	PVUControl();
	PVUControl(VirtualFlag channelflag);
	virtual ~PVUControl();
	VirtualFlag GetVirtualFlag();
	ReturnCode InitModule(InitPhase phase);//interface inherit from super
	void RecvFeedback();//interface inherit from super
	void SetReference(int flag,double ref_value[]);//interface inherit from super
	void GetCurrent(int & flag,double cur_value[]);//interface inherit from super
};

#endif /* PVUCONTROL_H_ */
