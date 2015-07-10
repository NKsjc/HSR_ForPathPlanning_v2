/*
 * EMGTrajectoryPlan.h
 *
 *  Created on: 2014-11-21
 *      Author: alvin
 */

#ifndef EMGTRAJECTORYPLAN_H_
#define EMGTRAJECTORYPLAN_H_

/*2014-11-21${file_loc}下午2:58:58
 *
 */
#include "TrajectoryPlan.h"

class EMGTrajectoryPlan: public TrajectoryPlan {
private:
	double sonar_dis[3];//超声距离
	double acc_wheel[2];//加速度设定
	double vel_wheel[2];//轮速设定
	MovingDirection moving_flag;
public:
	EMGTrajectoryPlan();
	virtual ~EMGTrajectoryPlan();
	void AddTask(int taskid,double tra_info[]);
	void CleanTask();
	void GetPeriodRef(int & flag,double ref_value[],double cur_info[]);
};

#endif /* EMGTRAJECTORYPLAN_H_ */
