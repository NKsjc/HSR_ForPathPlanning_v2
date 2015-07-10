/*
 * ModuleControl.h
 *
 *  Created on: 2014-9-4
 *      Author: alvin
 */

#ifndef MODULECONTROL_H_
#define MODULECONTROL_H_
#include "ProjectCommon.h"
class ModuleControl {
public:
	virtual ReturnCode InitModule(InitPhase phase)=0;
	virtual void RecvFeedback()=0;
	virtual void SetReference(int flag,double ref_value[])=0;
	virtual void GetCurrent(int & flag,double cur_value[])=0;
	virtual VirtualFlag GetVirtualFlag()=0;
	ModuleControl(){};
	virtual ~ModuleControl(){};
};

#endif /* MODULECONTROL_H_ */
