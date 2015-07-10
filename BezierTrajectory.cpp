#include"BezierTrajectory.h"
#include <math.h>
//#include<fstream>
#ifndef pi
const double pi=3.1415926535897;
#endif
#ifndef r
const double r=0.195472;
#endif
#ifndef d
const double d=0.23985;
#endif

BezierPlan::BezierPlan() {
	LENGTH=8000;
	v_max=new double[LENGTH];
	w_max=new double[LENGTH];
	a_v=new double [LENGTH];
	a_w=new double[LENGTH];
	d_t=new double[LENGTH];
}

BezierPlan::~BezierPlan() {
	delete[]v_max;
	delete[]w_max;
	delete[]a_v;
	delete[]a_w;
	delete[]d_t;

}
void BezierPlan::AddTask(int taskid, double tra_info[]) {
	flag = 0; //flag=0�����������أ�flag=1���֮ǰ����켣����֮����û�м��ع�����
	//num_waypoint = taskid;
	num_waypoint=4;
	double v_limit = 0.53;
	//double a_c_max = 0.1;
	double v_wheel_max = app_setting.get_setting_param().car_tra_max_vel;
	double v1, v2, v3;
	//double a_max = 0.1, a_w_max = 3;


//	double V_MAX=app_setting.get_setting_param().car_tra_max_vel;
//	double A_MAX=app_setting.get_setting_param().car_tra_max_acc;

	struct Point {
		double x;
		double y;
	} waypoint[10]; //waypoint�����·����

	/*for (int i = 0; i < num_waypoint; i++) {
		waypoint[i].x = tra_info[i + 3];
		waypoint[i].y = tra_info[i + num_waypoint + 3];
	}*/
	waypoint[0].x=1;
	waypoint[0].y=1;
	waypoint[1].x=5.03;
	waypoint[1].y=1.155;
	waypoint[2].x=5.785;
	waypoint[2].y=2.37;
	waypoint[3].x=6;
	waypoint[3].y=6;

	//double T_start = tra_info[0];
	double T_start =0.0384;
	double T_Theta[10];
	double alpha[10];
	double theta_next[10];
	double theta_pre[10];
	double length_T[10];
	double arc_length[10];
	T_Theta[0] = T_start;
	double e[4] = {17.0827,2.5795,1.6186,13.3318};
	double a1, a2, a3;
	for (int i = 1; i < num_waypoint - 1; i++) {
		a1 = sqrt(
				pow(waypoint[i - 1].x - waypoint[i + 1].x, 2)
						+ pow(waypoint[i - 1].y - waypoint[i + 1].y, 2));
		a2 = sqrt(
				pow(waypoint[i + 1].x - waypoint[i].x, 2)
						+ pow(waypoint[i + 1].y - waypoint[i].y, 2));
		a3 = sqrt(
				pow(waypoint[i - 1].x - waypoint[i].x, 2)
						+ pow(waypoint[i - 1].y - waypoint[i].y, 2));
		alpha[i - 1] = acos(
				(pow(a2, 2) + pow(a3, 2) - pow(a1, 2)) / (2 * a2 * a3));
		if (fabs(waypoint[i].x - waypoint[i + 1].x) < 0.0000001) {
			if (waypoint[i + 1].y > waypoint[i].y)
				theta_next[i - 1] = 90 * pi / 180;
			else
				theta_next[i - 1] = -90 * pi / 180;
		} else
			theta_next[i - 1] = atan2(waypoint[i + 1].y - waypoint[i].y,
					waypoint[i + 1].x - waypoint[i].x);

		if (fabs(waypoint[i].x - waypoint[i - 1].x) < 0.000000001) {
			if (waypoint[i].y > waypoint[i - 1].y)
				theta_pre[i - 1] = 90 * pi / 180;
			else
				theta_pre[i - 1] = -90 * pi / 180;
		} else
			theta_pre[i - 1] = atan2(waypoint[i].y - waypoint[i - 1].y,
					waypoint[i].x - waypoint[i - 1].x);

		if (theta_pre[i - 1] <= 90 * pi / 180 && theta_pre[i - 1] >= 0) {
			if (theta_next[i - 1] <= 90 * pi / 180
					&& theta_next[i - 1] > theta_pre[i - 1])
				T_Theta[i] = theta_next[i - 1] - (90 * pi / 180 - alpha[i - 1] / 2);
			else if (theta_next[i - 1] < theta_pre[i - 1]
					&& theta_next[i - 1] >= 0)
				T_Theta[i] = theta_next[i - 1] + (90 * pi / 180 - alpha[i - 1] / 2);
			else if (theta_next[i - 1] < 0
					&& theta_next[i - 1] > -90 * pi / 180)
				T_Theta[i] = theta_pre[i - 1] - (90 * pi / 180 - alpha[i - 1] / 2);
			else if (theta_next[i - 1] > 90 * pi / 180)
				T_Theta[i] = theta_pre[i - 1] + (90 * pi / 180 - alpha[i - 1] / 2);
		} else if (theta_pre[i - 1] > 90 * pi / 180 && theta_pre[i - 1] <= pi) {
			if (theta_next[i - 1] < theta_pre[i - 1]
					&& theta_next[i - 1] >= 90 * pi / 180)
				T_Theta[i] = theta_next[i - 1] + (90 * pi / 180 - alpha[i - 1] / 2);
			else if (theta_next[i - 1] > theta_pre[i - 1]
					&& theta_next[i - 1] < pi
					|| fabs(theta_next[i - 1] - pi) < 0.00001)
				T_Theta[i] = theta_next[i - 1] - (90 * pi / 180 - alpha[i - 1] / 2);
			else if (theta_next[i - 1] < 90 * pi / 180
					|| fabs(theta_next[i - 1] - pi / 2) < 0.00001
							&& theta_next[i - 1] > 0)
				T_Theta[i] = theta_next[i - 1] + (90 * pi / 180 - alpha[i - 1] / 2);
			else if (theta_next[i - 1] <= -90 * pi / 180)
				T_Theta[i] = theta_next[i - 1] - (90 * pi / 180 - alpha[i - 1] / 2);
		} else if (theta_pre[i - 1] <= 0
				&& theta_pre[i - 1] >= -90 * pi / 180) {
			if (theta_next[i - 1] > theta_pre[i - 1] && theta_next[i - 1] < 0)
				T_Theta[i] = theta_next[i - 1] - (90 * pi / 180 - alpha[i - 1] / 2);
			else if (theta_next[i - 1] < theta_pre[i - 1]
					&& theta_next[i - 1] > -90 * pi / 180)
				T_Theta[i] = theta_next[i - 1] + (90 * pi / 180 - alpha[i - 1] / 2);
			else if (theta_next[i - 1] >= 0
					&& theta_next[i - 1] < 90 * pi / 180)
				T_Theta[i] = theta_next[i - 1] - (90 * pi / 180 - alpha[i - 1] / 2);
			else if (theta_next[i - 1] <= -90 * pi / 180)
				T_Theta[i] = theta_next[i - 1] + (90 * pi / 180 - alpha[i - 1] / 2);
		} else if (theta_pre[i - 1] < -90 * pi / 180 && theta_pre[i - 1] >= -pi) {
			if (theta_next[i - 1] > theta_pre[i - 1]
					&& theta_next[i - 1] <= -90 * pi / 180)
				T_Theta[i] = theta_next[i - 1] - (90 * pi / 180 - alpha[i - 1] / 2);
			else if (theta_next[i - 1] < theta_pre[i - 1])
				T_Theta[i] = theta_next[i - 1] + (90 * pi / 180 - alpha[i - 1] / 2);
			else if (theta_next[i - 1] > 90 * pi / 180&& theta_next[i-1]<=pi)T_Theta[i]=theta_pre[i-1]-(90*pi/180-alpha[i-1]/2);
			else if(theta_next[i-1]<0 && theta_next[i-1]>-90*pi/180)
			T_Theta[i]=theta_next[i-1]-(90*pi/180-alpha[i-1]/2);
			else if (theta_next[i-1]>0 && theta_next[i-1]<=90*pi/180)
			T_Theta[i]=theta_next[i-1]+(90*pi/180-alpha[i-1]/2);
		}
		arc_length[i - 1] = a3;
		if (i == 1)
			length_T[i - 1] = a3;
		if (a2 < a3)
			length_T[i] = a2;
		else
			length_T[i] = a3;

		if (i == num_waypoint - 2) {
			length_T[num_waypoint - 1] = a2;
			arc_length[num_waypoint - 2] = a2;
		}

	}
	T_Theta[num_waypoint - 1] = atan2(
			waypoint[num_waypoint - 1].y - waypoint[num_waypoint - 2].y,
			waypoint[num_waypoint - 1].x - waypoint[num_waypoint - 2].x);
	Point ctrlpoint3[10][4];
	for (int i = 0; i < num_waypoint - 1; i++) {
		ctrlpoint3[i][0].x = waypoint[i].x;
		ctrlpoint3[i][1].x = waypoint[i].x + e[i]  * cos(T_Theta[i]) / 6;
		ctrlpoint3[i][2].x = waypoint[i + 1].x
				- e[i+1]  * cos(T_Theta[i + 1]) / 6;
		ctrlpoint3[i][3].x = waypoint[i + 1].x;
		ctrlpoint3[i][0].y = waypoint[i].y;
		ctrlpoint3[i][1].y = waypoint[i].y + e[i]  * sin(T_Theta[i]) / 6;
		ctrlpoint3[i][2].y = waypoint[i + 1].y
				- e [i+1] * sin(T_Theta[i + 1]) / 6;
		ctrlpoint3[i][3].y = waypoint[i + 1].y;
	}
	Point A_start[10];
	Point A_end[10];
	for (int i = 0; i < num_waypoint - 1; i++) {
		A_start[i].x = 6
				* (ctrlpoint3[i][2].x - 2 * ctrlpoint3[i][1].x
						+ ctrlpoint3[i][0].x);
		A_start[i].y = 6
				* (ctrlpoint3[i][2].y - 2 * ctrlpoint3[i][1].y
						+ ctrlpoint3[i][0].y);
		A_end[i].x = 6
				* (ctrlpoint3[i][3].x - 2 * ctrlpoint3[i][2].x
						+ ctrlpoint3[i][1].x);
		A_end[i].y = 6
				* (ctrlpoint3[i][3].y - 2 * ctrlpoint3[i][2].y
						+ ctrlpoint3[i][1].y);
	}
	Point A_join[10];
	A_join[0].x = A_start[0].x;
	A_join[0].y = A_start[0].y;
	A_join[num_waypoint - 1].x = A_end[num_waypoint - 2].x;
	A_join[num_waypoint - 1].y = A_end[num_waypoint - 2].y;
	for (int i = 1; i < num_waypoint - 1; i++) {
		A_join[i].x = (A_end[i - 1].x 
				+ A_start[i].x )
				/ 2;
		A_join[i].y = (A_end[i - 1].y 
				+ A_start[i].y )
				/ 2;
	}

	Point ctrlpoint5[10][6];
	for (int i = 0; i < num_waypoint - 1; i++) {
		ctrlpoint5[i][0].x = waypoint[i].x;
		ctrlpoint5[i][1].x = waypoint[i].x + e[i]  * cos(T_Theta[i]) / 10;
		ctrlpoint5[i][2].x = A_join[i].x / 20 + 2 * ctrlpoint5[i][1].x
				- waypoint[i].x;
		;
		ctrlpoint5[i][4].x = waypoint[i + 1].x
				- e[i+1]  * cos(T_Theta[i + 1]) / 10;
		ctrlpoint5[i][3].x = A_join[i + 1].x / 20 + 2 * ctrlpoint5[i][4].x
				- waypoint[i + 1].x;
		ctrlpoint5[i][5].x = waypoint[i + 1].x;
		ctrlpoint5[i][0].y = waypoint[i].y;
		ctrlpoint5[i][1].y = waypoint[i].y + e[i]  * sin(T_Theta[i]) / 10;
		ctrlpoint5[i][2].y = A_join[i].y / 20 + 2 * ctrlpoint5[i][1].y
				- waypoint[i].y;
		;
		ctrlpoint5[i][4].y = waypoint[i + 1].y
				- e[i+1]  * sin(T_Theta[i + 1]) / 10;
		ctrlpoint5[i][3].y = A_join[i + 1].y / 20 + 2 * ctrlpoint5[i][4].y
				- waypoint[i + 1].y;
		ctrlpoint5[i][5].y = waypoint[i + 1].y;
	}

	double u, u_temp, nu;
	int index;
	int j = 0;
	Point *pathpoint =new Point[LENGTH];
	Point *First=new Point[LENGTH];
	Point *Second=new Point[LENGTH];
	double *c=new double[LENGTH];

	for (u = 0; u < num_waypoint - 1; u = u + 0.001) {
		u_temp = u - floor(u);
		nu = floor(u);
		index = int(nu);
		pathpoint[j].x = pow((1 - u_temp), 5) * ctrlpoint5[index][0].x
				+ 5 * pow((1 - u_temp), 4) * u_temp * ctrlpoint5[index][1].x
				+ 10 * pow((1 - u_temp), 3) * pow(u_temp, 2)
						* ctrlpoint5[index][2].x
				+ 10 * pow((1 - u_temp), 2) * pow(u_temp, 3)
						* ctrlpoint5[index][3].x
				+ 5 * (1 - u_temp) * pow(u_temp, 4) * ctrlpoint5[index][4].x
				+ pow(u_temp, 5) * ctrlpoint5[index][5].x;
		pathpoint[j].y = pow((1 - u_temp), 5) * ctrlpoint5[index][0].y
				+ 5 * pow((1 - u_temp), 4) * u_temp * ctrlpoint5[index][1].y
				+ 10 * pow((1 - u_temp), 3) * pow(u_temp, 2)
						* ctrlpoint5[index][2].y
				+ 10 * pow((1 - u_temp), 2) * pow(u_temp, 3)
						* ctrlpoint5[index][3].y
				+ 5 * (1 - u_temp) * pow(u_temp, 4) * ctrlpoint5[index][4].y
				+ pow(u_temp, 5) * ctrlpoint5[index][5].y;
		First[j].x = 5
				* ((ctrlpoint5[index][1].x - ctrlpoint5[index][0].x)
						* pow((1 - u_temp), 4)
						+ 4 * (ctrlpoint5[index][2].x - ctrlpoint5[index][1].x)
								* u_temp * pow((1 - u_temp), 3)
						+ 6 * (ctrlpoint5[index][3].x - ctrlpoint5[index][2].x)
								* pow(u_temp, 2) * pow((1 - u_temp), 2)
						+ 4 * (ctrlpoint5[index][4].x - ctrlpoint5[index][3].x)
								* pow(u_temp, 3) * (1 - u_temp)
						+ (ctrlpoint5[index][5].x - ctrlpoint5[index][4].x)
								* pow(u_temp, 4));
		First[j].y = 5
				* ((ctrlpoint5[index][1].y - ctrlpoint5[index][0].y)
						* pow((1 - u_temp), 4)
						+ 4 * (ctrlpoint5[index][2].y - ctrlpoint5[index][1].y)
								* u_temp * pow((1 - u_temp), 3)
						+ 6 * (ctrlpoint5[index][3].y - ctrlpoint5[index][2].y)
								* pow(u_temp, 2) * pow((1 - u_temp), 2)
						+ 4 * (ctrlpoint5[index][4].y - ctrlpoint5[index][3].y)
								* pow(u_temp, 3) * (1 - u_temp)
						+ (ctrlpoint5[index][5].y - ctrlpoint5[index][4].y)
								* pow(u_temp, 4));
		Second[j].x = 5
				* (-4 * (ctrlpoint5[index][1].x - ctrlpoint5[index][0].x)
						* pow((1 - u_temp), 3)
						+ 4 * (ctrlpoint5[index][2].x - ctrlpoint5[index][1].x)
								* (pow((1 - u_temp), 3)
										- 3 * u_temp * pow((1 - u_temp), 2))
						+ 6 * (ctrlpoint5[index][3].x - ctrlpoint5[index][2].x)
								* (2 * u_temp * pow((1 - u_temp), 2)
										- 2 * pow(u_temp, 2) * (1 - u_temp))
						+ 4 * (ctrlpoint5[index][4].x - ctrlpoint5[index][3].x)
								* (3 * pow(u_temp, 2) - 4 * pow(u_temp, 3))
						+ 4 * (ctrlpoint5[index][5].x - ctrlpoint5[index][4].x)
								* pow(u_temp, 3));
		Second[j].y = 5
				* (-4 * (ctrlpoint5[index][1].y - ctrlpoint5[index][0].y)
						* pow((1 - u_temp), 3)
						+ 4 * (ctrlpoint5[index][2].y - ctrlpoint5[index][1].y)
								* (pow((1 - u_temp), 3)
										- 3 * u_temp * pow((1 - u_temp), 2))
						+ 6 * (ctrlpoint5[index][3].y - ctrlpoint5[index][2].y)
								* (2 * u_temp * pow((1 - u_temp), 2)
										- 2 * pow(u_temp, 2) * (1 - u_temp))
						+ 4 * (ctrlpoint5[index][4].y - ctrlpoint5[index][3].y)
								* (3 * pow(u_temp, 2) - 4 * pow(u_temp, 3))
						+ 4 * (ctrlpoint5[index][5].y - ctrlpoint5[index][4].y)
								* pow(u_temp, 3));
		c[j] = (First[j].x * Second[j].y - First[j].y * Second[j].x)
				/ pow((First[j].x * First[j].x + First[j].y * First[j].y), 1.5);
		j = j + 1;
	}
	int last = (num_waypoint - 1) * 1000;
	pathpoint[last].x = waypoint[num_waypoint - 1].x;
	pathpoint[last].y = waypoint[num_waypoint - 1].y;
	First[last].x = 5
			* (ctrlpoint5[num_waypoint - 2][5].x
					- ctrlpoint5[num_waypoint - 2][4].x);
	First[last].y = 5
			* (ctrlpoint5[num_waypoint - 2][5].y
					- ctrlpoint5[num_waypoint - 2][4].y);
	Second[last].x = 20
			* (ctrlpoint5[num_waypoint - 2][5].x
					- 2 * ctrlpoint5[num_waypoint - 2][4].x
					+ ctrlpoint5[num_waypoint - 2][3].x);
	Second[last].y = 20
			* (ctrlpoint5[num_waypoint - 2][5].y
					- 2 * ctrlpoint5[num_waypoint - 2][4].y
					+ ctrlpoint5[num_waypoint - 2][3].y);
	c[last] = (First[last].x * Second[last].y - First[last].y * Second[last].x)
			/ pow(
					(First[last].x * First[last].x
							+ First[last].y * First[last].y), 1.5);

	double d_x, d_y;
	double *d_s=new double[8000];
	for (int i = 1; i <= last; i++) {
		d_x = pathpoint[i].x - pathpoint[i - 1].x;
		d_y = pathpoint[i].y - pathpoint[i - 1].y;
		d_s[i - 1] = sqrt(d_x * d_x + d_y * d_y);
	}



	for (int i = 0; i <= last; i++) {
		v1 = v_limit;
		//v2 = sqrt(a_c_max / fabs(c[i]));
		v2 = r * v_wheel_max / (1 + fabs(d * c[i]));
		v_max[i] = v1;
		if (v2 < v_max[i])
			v_max[i] = v2;
		//if (v3 < v_max[i])
		//	v_max[i] = v3;

	}

	//v_max[0] = r * (tra_info[1] + tra_info[2]) / 2;
	v_max[0] = 0;
	double *v_a_max=new double [8000];
	double vv_min, vv_max, vv1, vv11, vv2, vv22;
	v_a_max[0] = v_max[0];
	double atmax=0.2;
	double armax=0.4;
	double *at=new double [8000];
	for (int i = 1; i <= last; i++) {
		at[i-1]=sqrt((1-(v_max[i-1]*v_max[i-1]*c[i-1])*(v_max[i-1]*v_max[i-1]*c[i-1])/(armax*armax))*atmax*atmax);
		v_a_max[i] = sqrt(
				v_a_max[i - 1] * v_a_max[i - 1] + 2 * at[i-1] * d_s[i - 1]);
		if (v_a_max[i] < v_max[i])
			v_max[i] = v_a_max[i];
		else
			v_a_max[i] = v_max[i];

	}

	v_max[last] = 0;
	v_a_max[last] = v_max[last];
	for (int i = last - 1; i >= 0; i--) {
		at[i]=sqrt((1-(v_max[i+1]*v_max[i+1]*c[i+1])*(v_max[i+1]*v_max[i+1]*c[i+1])/(armax*armax))*atmax*atmax);
		v_a_max[i] = sqrt(v_a_max[i + 1] * v_a_max[i + 1] + 2 * at[i] * d_s[i]);
		if (v_a_max[i] < v_max[i])
			v_max[i] = v_a_max[i];
		else
			v_a_max[i] = v_max[i];

	}

	for (int i = 0; i <= last; i++)
		w_max[i] = v_max[i] * c[i];
	t_total = 0;
	for (int i = 1; i <= last; i++) {
		d_t[i - 1] = 2 * d_s[i - 1] / (v_max[i] + v_max[i - 1]);
		t_total = t_total + d_t[i - 1];
	}

	for (int i = 0; i <= last - 1; i++) {
		a_v[i] = (v_max[i + 1] - v_max[i]) / d_t[i];
		a_w[i] = (w_max[i + 1] - w_max[i]) / d_t[i];
	}
#if 0
	ofstream outfile10("F:/BezierTrajectroy1/v.txt");
	for (int i = 0; i <= last; i++)
		outfile10 << v_max[i] << endl;
#endif
	call_num = 1;
	n_temp = 0;
	delete []v_a_max;
	delete[]pathpoint;
	delete[]First;
	delete[]Second;
	delete[]c;
	delete[]d_s;
	delete[]at;

}

void BezierPlan::GetPeriodRef(int & returnflag, double ref_value[],
		double cur_info[]) {
	int num_total = floor(t_total * 1000) + 1;
	if(call_num>=num_total)
	{
		flag=1;
	}
	returnflag = flag;
	if (flag == 0) {

		double t_left;

		if (call_num == 1) {
			v = 0;
			w = 0;
			wheel_v.w_r = (v + d * w) / r;
			wheel_v.w_l = (v - d * w) / r;
			tt_total = 0;
			tt_total = tt_total + d_t[n_temp];
			call_num++;
		} else {
			if (call_num * 0.001 <= tt_total) {
				v = v + a_v[n_temp] * 0.001;
				w = w + a_w[n_temp] * 0.001;
				wheel_v.w_r = (v + d * w) / r;
				wheel_v.w_l = (v - d * w) / r;
				call_num++;
			} else {
				t_left = tt_total - (call_num - 1) * 0.001;
				n_temp++;
				tt_total = tt_total + d_t[n_temp];
				if (call_num > 1 && call_num < num_total) {
					v = v + a_v[n_temp - 1] * t_left
							+ a_v[n_temp] * (0.001 - t_left);
					w = w + a_w[n_temp - 1] * t_left
							+ a_w[n_temp] * (0.001 - t_left);
					wheel_v.w_r = (v + d * w) / r;
					wheel_v.w_l = (v - d * w) / r;
					call_num++;
				} else {
					v = v + a_v[n_temp - 1] * t_left;
					w = w + a_w[n_temp - 1] * t_left;
					wheel_v.w_r = (v + d * w) / r;
					wheel_v.w_l = (v - d * w) / r;
					returnflag = 1;
				}
			}
		}
	} else if (flag == 1) {
		wheel_v.w_r = 0;
		wheel_v.w_l = 0;
	}
	ref_value[0] = wheel_v.w_r;
	ref_value[1] = wheel_v.w_l;
}

void BezierPlan::CleanTask() {
	flag = 1;
}

