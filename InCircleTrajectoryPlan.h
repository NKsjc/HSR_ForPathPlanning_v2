/*
 * InCircleTrajectoryPlan.h
 *
 *  Created on: 2014-9-5
 *      Author: alvin
 */

#ifndef INCIRCLETRAJECTORYPLAN_H_
#define INCIRCLETRAJECTORYPLAN_H_

#include "TrajectoryPlan.h"
#include <complex>
#include <list>
using namespace std;
typedef struct _trajectory_info
{
	int tra_type;//0-line，1-circle
	int rel_type;//result type,line，+circle，-circle
	double Tmax;//tar time
	double current_t;//current time
	double s_t1;//const acc time
	double s_t2;//const vel time
	double l_t[7];
	double l_v[7];
	double l_a2;
	double l_a6;
	double l_k[4];
	double Vmax;//max vel
	double amax;//max acc
	double w_b;//begin vel
}trajectory_info;//tra info

typedef struct _routine_info
{
	double target_x,target_y,target_st,target_left,target_right;
	int end_flag;

}routine_info;//task info

typedef list<routine_info> queue_of_routine;//queue of task
typedef list<trajectory_info> queue_of_tra;//queue of tra info

class InCircleTrajectoryPlan: public TrajectoryPlan {
private:
	queue_of_routine hsr_routine;//queue of task
	queue_of_tra hsr_tra;//queue of tra info

	double best_vel;//求出下一个入弯的合适的速度
	double best_dis;//求出下一个入弯的距离
	double log_x;
	double log_y;
	double log_st;//用于迭代规划值，对于实际的位姿
public:
	InCircleTrajectoryPlan();
	virtual ~InCircleTrajectoryPlan();
	double GetCarTrajectory(trajectory_info &sul);//获取轨迹规划的点
	ReturnCode GetWheelVel(trajectory_info & sul,double &v_left,double &v_right);//从轨迹规划的结果里计算设定速度
	void Kine_Move(trajectory_info temp_sul,double x_i,double y_i,double st_i,double &x_e,double &y_e,double &st_e);//用于运动学迭代
	void Cal_Negative_Tmin(double &dth,double &th20,double &th30,double &tinc2,double &am,double &k,double &v0,
		double &vf,double &a6,double &a2,double &tmin,double &tup3,double &tup4,double &aainc);
	void Cal_Positive_Tmin(double &dth,double &th2,double &th3,double &tinc2,double &am,double &k,double &v0,
		double &vf,double &a6,double &a2,double &tmin,double &tup3,double &tup4,double &aainc);
	double GetDTheta(double k[4],double t[7],double v0);
	void SolveN(double a,double b,double c,double d,double e,complex<double> x[4]);
	void MATP(int type,double v0, double vf,  double tf, double k, double am, double vm,double ke[4],double t[7], double param[5]);
	int Line_Trajectory(int tra_type,double st_b,double st_e,double v0,double vf,double delta_s,trajectory_info &sul);
	void Cal_Circle_Vel(double current_x,double current_y,double current_st);//根据当前队列的情况计算下一个弯的入弯速度
	int Is_Valid_Tra(int tra_type,double st_b,double st_e,double w_b,double w_e,double delta_s,trajectory_info &sul);//判断给定的轨迹序列是否可以达到
	int Cal_Trajectory(int tra_type,double st_b,double st_e,double w_b,double w_e,double delta_s,trajectory_info &sul );//计算轨迹规划
	int Pop_New_Routine(double current_x,double current_y,double current_st,double current_left,double current_right);//取指令队列数据进行轨迹规划


	void AddTask(int taskid,double tra_info[]);
	void CleanTask();
	void GetPeriodRef(int & flag,double ref_value[],double cur_info[]);
};

#endif /* INCIRCLETRAJECTORYPLAN_H_ */
