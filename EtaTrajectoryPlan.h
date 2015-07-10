/*
 * EtaTrajectoryPlan.h
 *
 *  Created on: 2014-9-6
 *      Author: alvin
 */

#ifndef ETATRAJECTORYPLAN_H_
#define ETATRAJECTORYPLAN_H_

#include "TrajectoryPlan.h"
#include <list>
#include <vector>
using namespace std;
typedef struct _eta_path
{
	double begin_x;
	double begin_y;
	double begin_st;
	double end_x;
	double end_y;
	double end_st;
	double eta[6];
}eta_path;
typedef struct _vel_wheel
{
	double v_l;double v_r;
}vel_wheel;
typedef vector<vel_wheel> contrl_buff;
typedef list<eta_path> queue_eta_path;

class EtaTrajectoryPlan: public TrajectoryPlan {
private:
	contrl_buff eta_buff;
	queue_eta_path tra_path;
	int flag;

public:
	EtaTrajectoryPlan();
	virtual ~EtaTrajectoryPlan();
	void Cal_Tra(eta_path new_path);

	void AddTask(int taskid,double tra_info[]);
	void CleanTask();
	void GetPeriodRef(int & returnflag,double ref_value[],double cur_info[]);
};

#endif /* ETATRAJECTORYPLAN_H_ */
