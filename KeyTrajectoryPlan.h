/*
 * KeyTrajectoryPlan.h
 *
 *  Created on: 2014-9-7
 *      Author: alvin
 */

#ifndef KEYTRAJECTORYPLAN_H_
#define KEYTRAJECTORYPLAN_H_

#include "TrajectoryPlan.h"

class KeyTrajectoryPlan: public TrajectoryPlan {
private:
	ModuleId id;
	int moving_joint;
	MovingDirection moving_flag;
	int count_key;
	double ref_value[4];
	double step_joint[4];
	double acc_wheel[2];
public:
	KeyTrajectoryPlan(ModuleId type);
	virtual ~KeyTrajectoryPlan();
	void AddTask(int taskid,double tra_info[]);
	void CleanTask();
	void GetPeriodRef(int & returnflag,double ref_value[],double cur_info[]);
};

#endif /* KEYTRAJECTORYPLAN_H_ */
