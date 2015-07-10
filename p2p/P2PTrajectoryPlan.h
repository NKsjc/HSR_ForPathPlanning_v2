/*
 * P2PTrajectoryPlan.h
 *
 *  Created on: 2014-9-6
 *      Author: alvin
 */

#ifndef P2PTRAJECTORYPLAN_H_
#define P2PTRAJECTORYPLAN_H_

#include "TrajectoryPlan.h"
#include <complex>
#include <list>
using namespace std;
typedef struct _hand_trajectory_info
{
	double Tmax;//轨迹段实现时间
	double current_t;//当前执行的时间
	int flag;
	double s_t1;//曲线匀加速时间
	double s_t2;//曲线匀速时间
	double l_t[7];
	double l_v[7];
	double l_a2;
	double l_a6;
	double l_k[4];
	double Vmax;//轨迹中最大角速度
	double amax;//轨迹中最大角加速度
	double w_b;//轨迹开始段的初始角速度
	double synctime;//normalize
}hand_trajectory_info;//轨迹规划的结果

typedef struct _hand_routine_info
{
	double st[4];//目标位置的关节角度
	int end_flag;

}hand_routine_info;//上位机给出的路径点结构体

typedef list<hand_routine_info> queue_of_hand_routine;//路径点队列
typedef list<hand_trajectory_info> queue_of_hand_tra;//轨迹规划结果队列

class P2PTrajectoryPlan: public TrajectoryPlan {
private:
	queue_of_hand_routine hsr_hand_routine;
	hand_trajectory_info hand_tra[4];
	int last_tra_end;//上一段轨迹执行结束
	double hand_positive_limit[4];
	double hand_negtive_limit[4];
public:
	P2PTrajectoryPlan();
	virtual ~P2PTrajectoryPlan();

	void Cal_Positive_Tmin(double &dth,double &th2,double &th3,double &tinc2,double &am,double &k,double &v0,
			double &vf,double &a6,double &a2,double &tmin,double &tup3,double &tup4,double &aainc);
	void Cal_Negative_Tmin(double &dth,double &th20,double &th30,double &tinc2,double &am,double &k,double &v0,
		double &vf,double &a6,double &a2,double &tmin,double &tup3,double &tup4,double &aainc);
	void MATP(int type,double v0, double vf,  double tf, double k, double am, double vm,double ke[4],double t[7], double param[5]);
	double GetDTheta(double k[4],double t[7],double v0);
	void SolveN(double a,double b,double c,double d,double e,complex<double> x[4]);
	int Cal_Trajectory(double st_b,double st_e,double v0,double vf,hand_trajectory_info &sul);
	int Trajectory_From_Target(double st_b[4], double st_e[4],double w_b[4],double w_e[4]);   //由始末状态进行规划
	double Get_Hand_Trajectory(hand_trajectory_info &sul);//获取轨迹规划的点
	int Check_Collision(double current_st[4]);//检查当前轨迹是否会干涉机器人自身

	void AddTask(int taskid,double tra_info[]);
	void CleanTask();
	void GetPeriodRef(int & flag,double ref_value[],double cur_info[]);
};

#endif /* P2PTRAJECTORYPLAN_H_ */
