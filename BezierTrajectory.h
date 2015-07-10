#include<iostream>
#include "TrajectoryPlan.h"
using namespace std;

struct Wheel{
		double w_r;
		double w_l;
	};

class BezierPlan: public TrajectoryPlan {
private:
	Wheel wheel_v;
	int LENGTH;
	double *v_max,*w_max,*a_v,*a_w,*d_t;
	double t_total;
	double tt_total;
	double v;
	double w;
	int num_waypoint;
	int call_num;
	int flag ;
	int n_temp;
public:
	BezierPlan();
	virtual ~BezierPlan();


	void AddTask(int taskid,double tra_info[]);     //tra_info[]����ĵ�һ�����ǻ����˵�ǰ����̬��
	void CleanTask();
	void GetPeriodRef(int & returnflag,double ref_value[],double cur_info[]);
};
