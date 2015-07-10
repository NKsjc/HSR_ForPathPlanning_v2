/*
 * TrajectoryPlan.h
 *
 *  Created on: 2014-9-5
 *      Author: alvin
 */

#ifndef TRAJECTORYPLAN_H_
#define TRAJECTORYPLAN_H_
#include "ProjectCommon.h"
class TrajectoryPlan {
public:
	TrajectoryPlan(){};
	virtual ~TrajectoryPlan(){};
	virtual void AddTask(int taskid,double tra_info[])=0;
	virtual void CleanTask()=0;
	virtual void GetPeriodRef(int & flag,double ref_value[],double cur_info[])=0;
};

#endif /* TRAJECTORYPLAN_H_ */
