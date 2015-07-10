/*
 * InCircleTrajectoryPlan.cpp
 *
 *  Created on: 2014-9-5
 *      Author: alvin
 */

#include "InCircleTrajectoryPlan.h"
#include <string.h>
InCircleTrajectoryPlan::InCircleTrajectoryPlan() {
	// TODO Auto-generated constructor stub
	this->log_x = this->log_y = this->log_st = 0;
}

InCircleTrajectoryPlan::~InCircleTrajectoryPlan() {
	// TODO Auto-generated destructor stub
	this->hsr_tra.clear();
	this->hsr_routine.clear();
}
void InCircleTrajectoryPlan::AddTask(int taskid,double tra_info[]){
	routine_info temp;
	temp.target_x=tra_info[0];
	temp.target_y=tra_info[1];
	temp.target_st=tra_info[2];
	//此时先去除路径点中意义不大的点，然后规划
	double st1,st2;
	double d_l;
	queue_of_routine::iterator second;
	if (this->hsr_routine.size()>=1)
	{
		routine_info last=this->hsr_routine.back();
		d_l=sqrt(pow(last.target_x-temp.target_x,2)+pow(last.target_y-temp.target_y,2));
		if (this->hsr_routine.size()>=2)
		{
			second=this->hsr_routine.end();
			second--;
			second--;
			st1=atan2(last.target_y-second->target_y,last.target_x-second->target_x);
			st2=atan2(last.target_y-temp.target_y,last.target_x-temp.target_x);
		}
		if (d_l<0.1||(this->hsr_routine.size()>=2&&(fabs(fabs(st1)-fabs(st2))<0.005)))
		{
			this->hsr_routine.pop_back();
//			cout<<"lllllllllllllllllllllllll"<<temp.target_x<<"  "<<temp.target_y<<"  "<<last.target_x<<"  "<<last.target_y<<"  "
//			<<second->target_x<<"  "<<second->target_y<<"  "<<st1<<"  "<<st2<<endl;
		}
	}
	this->hsr_routine.push_back(temp);
	hsrlog.Print(_Debug,"new target:  %4f  %4f  %4f\n",temp.target_x,temp.target_y,temp.target_st);
}
void InCircleTrajectoryPlan::CleanTask(){
	this->hsr_routine.clear();
	this->hsr_tra.clear();
}
void InCircleTrajectoryPlan::GetPeriodRef(int & flag,double ref_value[],double cur_info[]){
	//ref_value[0]=v_left,ref_value[1]=v_right;
	//if no tra,flag=0;if new tra,flag=1;if error flag=-1,if ok,flag=2;
	double &v_left=ref_value[0];
	double &v_right=ref_value[1];
	double & current_x=cur_info[0];
	double & current_y=cur_info[1];
	double & current_st=cur_info[2];
	double & current_left=cur_info[3];
	double & current_right=cur_info[4];

	if(this->hsr_tra.empty()){
		if(this->hsr_routine.empty())
		{
			v_left=v_right=0;
			flag=0;
			return ;
		}
		else
		{
			hsrlog.Print(_Debug,"hsr tra is empty but path queue is ok ,so begin new tra plan\n");
			int result=Pop_New_Routine(current_x,current_y,current_st,
			current_left,current_right);
			if(result==-1)
			{
//				cout<<"******************************************************"<<endl;
//				cout<<" path error"<<endl;
				hsrlog.Print(_Debug,"***********************Path error********************");
				this->hsr_tra.clear();
				this->hsr_routine.clear();
				v_left=v_right=0;
				flag=-1;
				return ;
			}
			else
			{
				v_left=current_left;
				v_right=current_right;
				flag=1;
				return ;
			}
		}
	}
	else
	{
		queue_of_tra::iterator temp=this->hsr_tra.begin();
		if (SUCCEED!=GetWheelVel(*temp,v_left,v_right))
		{
			hsrlog.Print(_Debug,"the last tra is all used ,drop it\n");
			this->hsr_tra.pop_front();
			//这个节拍没有取到数，所以还采用上个节拍的数据
			v_left=current_left;
			v_right=current_right;
		}

	}
	flag=2;
}



int InCircleTrajectoryPlan::Pop_New_Routine(double current_x,double current_y,double current_st,double current_left,double current_right)//取指令队列数据进行轨迹规划
{
	double V_MAX=app_setting.get_setting_param().car_tra_max_vel;
	double A_MAX=app_setting.get_setting_param().car_tra_max_acc;
	this->log_x=current_x;
	this->log_y=current_y;
	this->log_st=current_st;
	//此时规划时先去除路径点中意义不大的点，然后规划
	queue_of_routine::iterator i=this->hsr_routine.begin();
	double st1,st2;
	trajectory_info temp_sul={0};
	routine_info temp=this->hsr_routine.front();
	double delta_s,v_b;
	//检测当前车子方向与位移方向是否一直，不一致则先旋转,或者是第一个点给出时小车是静止的
	double delta_st=atan2(temp.target_y-log_y,temp.target_x-log_x)-log_st;
	if((delta_st)>pi)delta_st-=2*pi;
	if((delta_st)<-pi)delta_st+=2*pi;
//	cout<<"current_x,y,st  target_x,y,st"<<
//		current_x<<"   "<<current_y<<"  "<<current_st<<"  "<<temp.target_x<<"  "<<temp.target_y<<"  "<<temp.target_st<<endl;


	hsrlog.Print(_Debug,"this tra begin with  info :current:%4f  %4f   %4f  target: %4f   %4f   %4f\n",current_x,current_y,current_st,temp.target_x,temp.target_y,temp.target_st);

	if(sqrt(pow((temp.target_x-current_x),2)+pow((temp.target_y-current_y),2))<0.001)//发现当前路径点与当前位置太近则直接跳过
	{
		hsrlog.Print(_Debug,"the new target is too close ,skip it\n");
		this->hsr_routine.pop_front();
		return 0;
	}
	if(fabs(current_st-atan2((temp.target_y-current_y),(temp.target_x-current_x)))>0.001&&fabs(current_left)<0.001)
	{
		hsrlog.Print(_Debug,"current status is stop ,so start with a status adjust\n");
		if(Is_Valid_Tra(CIRCLE_TRA,0,delta_st,0,0,0,temp_sul)==0)
		{
			if(Cal_Trajectory(CIRCLE_TRA,0,delta_st,0,0,0,temp_sul)==-1)return -1;
			//规划成功加入到队列
			this->hsr_tra.push_back(temp_sul);
			//此处是为了保证初始情况下转动后，后期运动学迭代保持一致
			Kine_Move(temp_sul,this->log_x,this->log_y,this->log_st,this->log_x,this->log_y,this->log_st);
		}
		else
		{
			//路径点无法实现，程序处理错误
			return -1;
		}

	}

	if(this->hsr_routine.size()==1)
	{
		//只有一个目标点时规划为直线
		hsrlog.Print(_Debug,"the path is only one point,paln only a line\n");
		delta_s=sqrt(pow((temp.target_x-current_x),2)+pow((temp.target_y-current_y),2));
		v_b=(current_left+current_right)/2*r;
#if 1
		if (Is_Valid_Tra(LINE__TRA,0,0,v_b/r,0,delta_s,temp_sul)==0)
		{
			if(Cal_Trajectory(LINE__TRA,0,0,v_b/r,0,delta_s,temp_sul)==-1)
			{
				return -1;
			}
			else
			{
				this->hsr_tra.push_back(temp_sul);
				Kine_Move(temp_sul,this->log_x,this->log_y,this->log_st,this->log_x,this->log_y,this->log_st);
			}
		}
#endif
		else
		{
			//路径点无法实现，程序处理错误
			return -1;
		}
		//add the last posture
		hsrlog.Print(_Debug,"add the last posture \n");
		if(Is_Valid_Tra(CIRCLE_TRA,0,temp.target_st-this->log_st,0,0,0,temp_sul)==0)
		{
			if(Cal_Trajectory(CIRCLE_TRA,0,temp.target_st-this->log_st,0,0,0,temp_sul)==-1)return -1;
			//规划成功加入到队列
			this->hsr_tra.push_back(temp_sul);
			//此处是为了保证初始情况下转动后，后期运动学迭代保持一致
			Kine_Move(temp_sul,this->log_x,this->log_y,this->log_st,this->log_x,this->log_y,this->log_st);
		}
		else
		{
			//路径点无法实现，程序处理错误
			return -1;
		}

	}
	else
	{
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		hsrlog.Print(_Debug,"there are more than 1 point ,plan with incircle\n");
		queue_of_routine::iterator first,second;

		trajectory_info circle_sul={0};
		first=this->hsr_routine.begin();
		second=first;
		second++;

		st1=atan2(first->target_y-this->log_y,first->target_x-this->log_x);
		st2=atan2(second->target_y-first->target_y,second->target_x-first->target_x);
		delta_st=st2-st1;//此处st1和st2在前期验算时已经求出来了
		if((delta_st)>pi)delta_st-=2*pi;
		if((delta_st)<-pi)delta_st+=2*pi;
		if (fabs(delta_st)<0.001)//这2个点基本在一条直线上
		{
			hsrlog.Print(_Debug,"there nearst 2 points are on the line ,so keep line");
			this->best_dis=sqrt(pow(first->target_x-log_x,2)+pow(first->target_y-log_y,2));
			this->best_vel=V_MAX-0.1;
		}
		else
		{
			hsrlog.Print(_Debug,"there nearst 2 points are not on the line ,so cal the best incircle point and vel\n");
			Cal_Circle_Vel(log_x,log_y,log_st);

		}
		if (this->best_vel>=0&&this->best_dis>0)//已经求出合适的入弯速度和入弯距离
		{
			if (Is_Valid_Tra(LINE__TRA,0,0,(current_left+current_right)/2,this->best_vel,this->best_dis,temp_sul)==0)
			{
				if(Cal_Trajectory(LINE__TRA,0,0,(current_left+current_right)/2,this->best_vel,this->best_dis,temp_sul)==-1)
				{
					return -1;
				}
				else
				{
					this->hsr_tra.push_back(temp_sul);
					Kine_Move(temp_sul,this->log_x,this->log_y,this->log_st,this->log_x,this->log_y,this->log_st);
				}
			}
			else
			{
				//路径点无法实现，程序处理错误
				return -1;
			}
			if (fabs(delta_st)>=0.001)//这2个点不在一条直线上
			{
				//内切圆阶段的规划
				if (Is_Valid_Tra(CIRCLE_TRA,0,delta_st,this->best_vel,this->best_vel,0,circle_sul)==0)
				{
					if(Cal_Trajectory(CIRCLE_TRA,0,delta_st,this->best_vel,this->best_vel,0,circle_sul)==-1)return -1;

					this->hsr_tra.push_back(circle_sul);
					Kine_Move(temp_sul,this->log_x,this->log_y,this->log_st,this->log_x,this->log_y,this->log_st);
				}
				else
				{
					//路径点无法实现，程序处理错误
					return -1;
				}
			}
		}
		else//轨迹规划失败
		{
			return -1;
		}
	}
	//轨迹规划结束后弹出路径点
	this->hsr_routine.pop_front();
	return 0;
}
int InCircleTrajectoryPlan::Cal_Trajectory(int tra_type,double st_b,double st_e,double w_b,double w_e,double delta_s,trajectory_info &sul )//计算轨迹规划
{
	double V_MAX=app_setting.get_setting_param().car_tra_max_vel;
	double A_MAX=app_setting.get_setting_param().car_tra_max_acc;
	switch(tra_type)
	{
	case LINE__TRA:
		{
			sul.tra_type=LINE__TRA;

			int ret=Line_Trajectory(tra_type,st_b,st_e,w_b,w_e,delta_s,sul);
			if (ret!=0)
			{
				return -1;
			}
		}
		break;
	case CIRCLE_TRA:
		{
			sul.tra_type=CIRCLE_TRA;
			//求取速度限制,此处是在加速度不需要添加矩形情况下
			if (sul.s_t1==0)
			{
				double v_max,v_sin_max,t2,K;
				double delta_w=V_MAX-w_b;
				double delta_st=fabs(st_e-st_b);
				if(r*delta_w*T/2/Le>=delta_st)
				{
					//速度积分大于所需的角度变化
					K=delta_st/(r*delta_w*T/2/Le);
					delta_w=delta_st*2*Le/r/T;
					v_sin_max=delta_w/2;
					v_max=delta_w+w_b;
					t2=0;
					/////////
					sul.s_t2=t2;
					sul.Vmax=v_max;
					sul.amax*=K;
					sul.Tmax=sul.s_t1*2+sul.s_t2+T;
					return 0;
				}
				else
				{
					//此时说明速度积分不够角度的偏移，需要添加矩形补充
					K=1;
					v_max=V_MAX;
					if(fabs(v_max-w_b)<0.005)return -1;
					t2=(delta_st-r*delta_w*T/2/Le)/(v_max-w_b)*Le/r;
					v_sin_max=delta_w/2;
					sul.s_t2=t2;
					//		su->v_sin_max=v_sin_max;
					sul.Vmax=v_max;
					sul.Tmax=sul.s_t1*2+sul.s_t2+T;
					return 0;
				}
			}else	if (sul.s_t1>0)
			{
				//求取速度限制，此处是在加速度添加矩形后的速度积分
				double v_max,K,t2;
				////////////////
				double temp_st=0;
				double delta_st=fabs(st_e-st_b);
				temp_st=(T*T/2/pi+T*sul.s_t1/pi+T*sul.s_t1/2+sul.s_t1*sul.s_t1)*sul.amax*r/Le;
				K=1;
				if (temp_st>=delta_st)
				{
					//此处采用的速度积分超过所需的转角
					K=delta_st/temp_st;
					//				v_sin_max=a_max*T/2/pi*K;
					v_max=(T/pi*sul.amax+sul.amax*sul.s_t1)*K+w_b;
					t2=0;
					//					su->v_sin_max=v_sin_max;
					sul.Vmax=v_max;
					sul.s_t2=t2;
					sul.amax*=K;
					sul.Tmax=sul.s_t1*2+sul.s_t2+T;
					return 0;
				}
				else
				{
					//此处速度积分不够，需要添加矩形块
					K=1;
					//					v_sin_max=a_max*T/2/pi*K;
					v_max=(T/pi*sul.amax+sul.amax*sul.s_t1)*K+w_b;
					if(fabs(v_max-w_b)<0.005)return -1;
					t2=(delta_st-temp_st)/(v_max-w_b)*Le/r;
					//					su->v_sin_max=v_sin_max;
					sul.Vmax=v_max;
					sul.s_t2=t2;
					sul.amax*=K;
					sul.Tmax=sul.s_t1*2+sul.s_t2+T;
					return 0;
				}
			}

		}
		break;
	default:return -1;
	}
	return 0;
}
int InCircleTrajectoryPlan::Is_Valid_Tra(int tra_type,double st_b,double st_e,double w_b,double w_e,double delta_s,trajectory_info &sul)//判断给定的轨迹序列是否可以达到
{
	double V_MAX=app_setting.get_setting_param().car_tra_max_vel;
	double A_MAX=app_setting.get_setting_param().car_tra_max_acc;
	memset(&sul,0,sizeof(sul));
	int flag=0;
	switch (tra_type)
	{
	case LINE__TRA:
		{

		}
		break;
	case CIRCLE_TRA:
		{
			sul.tra_type=CIRCLE_TRA;
			sul.rel_type=(st_b>=st_e?TURN_RIGHT:TURN_LEFT);
			sul.w_b=w_b;
			double a_max ,t1;
			if (T/pi*A_MAX>=V_MAX-w_b)
			{
				//说明加速度性能足够V_MAX
				a_max=(V_MAX-w_b)*pi/T;
				t1=0;
				sul.amax=a_max;
				sul.s_t1=t1;
				flag =0;
			}
			else
			{
				//加速度性能无法达到V_MAX
				//求取需要添加矩形的时间t1
				a_max=A_MAX;
				t1=(V_MAX-w_b-T/pi*A_MAX)/A_MAX;
				sul.s_t1=t1;
				sul.amax=a_max;
				flag =0;
			}
		}
		break;
	default:
		return -1;//轨迹类型不合法
	}
	return flag;
}

void InCircleTrajectoryPlan::Cal_Circle_Vel(double current_x,double current_y,double current_st)//根据当前队列的情况计算下一个弯的入弯速度
{
	double V_MAX=app_setting.get_setting_param().car_tra_max_vel;
	double A_MAX=app_setting.get_setting_param().car_tra_max_acc;
	double W_E=V_MAX;
	queue_of_routine::iterator first,second,third;
	double dst01,dst12;
	double d01,d12;
	trajectory_info temp_sul;

	if (this->hsr_routine.size()>=2)//仅有一个弯道
	{
		first=hsr_routine.begin();
		second=first;
		second++;
		dst01=atan2(second->target_y-first->target_y,second->target_x-first->target_x)-atan2(first->target_y-current_y,first->target_x-current_x);
		if((dst01)>pi)dst01-=2*pi;
		if((dst01)<-pi)dst01+=2*pi;
		d01=sqrt(pow(first->target_x-current_x,2)+pow(first->target_y-current_y,2));
		d12=sqrt(pow(first->target_x-second->target_x,2)+pow(first->target_y-second->target_y,2));
		double K_Low=0.5;
		this->best_vel=W_E/K_Low-0.1;
//		this->best_vel=0;
		//内切圆阶段的规划
		double x=0,y=0,st=0;
		do
		{

			best_vel*=K_Low;
			if (Is_Valid_Tra(CIRCLE_TRA,0,dst01,best_vel,best_vel,0,temp_sul)==0)
			{
				if(Cal_Trajectory(CIRCLE_TRA,0,dst01,best_vel,best_vel,0,temp_sul)==-1)
				{
					continue;
				}
			}
			else
			{
				//路径点无法实现，程序处理错误
				best_dis=-1;
				best_vel=0;
				return;
			}
			//验算试验点
			x=y=st=0;
			Kine_Move(temp_sul,x,y,st,x,y,st);

			best_dis=fabs(y/sin(st));

		} while (!(this->best_dis<=0.5*d01&&this->best_dis<=0.5*d12)&&best_vel>0.001);//路径点可以实现
//		cout<<"loop count:"<<count<<endl;
		this->best_dis=d01-this->best_dis;
	}
	if(this->hsr_routine.size()<0)//至少有2个弯道
	{
		first=hsr_routine.begin();
		second=first;
		second++;
		third=second;
		third++;
		dst01=atan2(second->target_y-first->target_y,second->target_x-first->target_x)-atan2(first->target_y-current_y,first->target_x-current_x);
		if((dst01)>pi)dst01-=2*pi;
		if((dst01)<-pi)dst01+=2*pi;
		dst12=atan2(third->target_y-second->target_y,third->target_x-second->target_x)-atan2(second->target_y-first->target_y,second->target_x-first->target_x);
		if((dst12)>pi)dst12-=2*pi;
		if((dst12)<-pi)dst12+=2*pi;
		d01=sqrt(pow(first->target_x-current_x,2)+pow(first->target_y-current_y,2));
		d12=sqrt(pow(first->target_x-second->target_x,2)+pow(first->target_y-second->target_y,2));

		//内切圆阶段的规划
		double x=0,y=0,st=0;
		double constd12;
		if (Is_Valid_Tra(CIRCLE_TRA,0,dst12,W_E,W_E,0,temp_sul)==0)
		{
			Cal_Trajectory(CIRCLE_TRA,0,dst12,W_E,W_E,0,temp_sul);
		}
		else
		{
			//路径点无法实现，程序处理错误
			best_dis=-1;
			best_vel=0;
			return;
		}
		//验算试验点
		x=y=st=0;
		Kine_Move(temp_sul,x,y,st,x,y,st);
		constd12=fabs(y/sin(st));
		this->best_vel=W_E/0.8f;
		do
		{
			best_vel*=0.8f;
			if (Is_Valid_Tra(CIRCLE_TRA,0,dst01,best_vel,best_vel,0,temp_sul)==0)
			{
				Cal_Trajectory(CIRCLE_TRA,0,dst01,best_vel,best_vel,0,temp_sul);
			}
			else
			{
				//路径点无法实现，程序处理错误
				best_dis=-1;
				best_vel=0;
				return;
			}
			//验算试验点
			x=y=st=0;
			Kine_Move(temp_sul,x,y,st,x,y,st);
			best_dis=fabs(y/sin(st));
		} while (!(this->best_dis<=0.5*d01&&this->best_dis<=0.5*d12&&this->best_dis+constd12<d12));//路径点可以实现
		this->best_dis=d01-this->best_dis;
	}
	return;
}
int InCircleTrajectoryPlan::Line_Trajectory(int tra_type, double st_b, double st_e,
	double v0, double vf, double delta_s, trajectory_info &sul) {
	double V_MAX=app_setting.get_setting_param().car_tra_max_vel;
	double A_MAX=app_setting.get_setting_param().car_tra_max_acc;
	double a0, af, am, vm, k, th0, thf, dth;
	a0 = 0;
	af = 0;
	am = A_MAX;
	vm = V_MAX;
	k = 1.5;
	th0 = 0;
	thf = delta_s / r;
	dth = thf - th0;
	//////////////////////////////////////////////////////////////////////////
	double tvb1 = am / k;
	double tvb2 = (vf - v0) / am;
	double tinf2 = tvb1 + tvb2;
	double tinf3 = tvb1 - tvb2;
	double tinf = tinf2 >= tinf3 ? tinf2 : tinf3;
	if ((vm - v0) < 0) {
		v0 = vm;
	}
	if ((vm - vf) < 0) {
		vf = vm;
	}

	double a2t = sqrt(k * (-v0 + vm));
	double a6t = -sqrt(k * (-vf + vm));
	double a2 = a2t < am ? a2t : am;
	double a6 = a6t > -am ? a6t : -am;
	double t4 = 0;

	double tinc2 = (vf - v0) / am;
	double tinc3 = am / k;
	double aainc = am * k * tinc2;

	double tup1 = (a2 + pow(a2t, 2) / a2 - a6 - pow(a6t, 2) / a6) / k;
	double tup2 = fabs(tinc2) + 4 * tinc3;
	double deltas = tinc3 - fabs(tinc2);
	//////////////////////////////////////////////////////////////////////////
	double tup3, tup4;
	if (deltas < 0) {
		tup3 = 0;
		tup4 = 0;
	} else {
		tup3 = 2 * tinc3 + 2 * sqrt(deltas * tinc3);
		tup4 = 2 * tinc3 - 2 * sqrt(deltas * tinc3);
	}

	if (tup2 > tup1)
		tup2 = tup1;
	if (tup3 > tup2)
		tup3 = tup2;
	if (tup3 <= tinf)
		tup3 = tinf;
	if (tup4 >= tup3)
		tup4 = tup3;
	if (tup4 <= tinf)
		tup4 = tinf;
	//////////////////////////////////////////////////////////////////////////
	double tempk[4], tempt[7], tempp[6];
	double k1, k2, k3, k4;
	double t1, t2, t3, t5, t6, t7;
	MATP(1, v0, vf, tinf, k, am, vm, tempk, tempt, tempp);
	double th1 = GetDTheta(tempk, tempt, v0);

	MATP(1, v0, vf, tup4, k, am, vm, tempk, tempt, tempp);
	double th2 = GetDTheta(tempk, tempt, v0);

	MATP(1, v0, vf, tup3, k, am, vm, tempk, tempt, tempp);
	double th3 = GetDTheta(tempk, tempt, v0);

	MATP(1, v0, vf, tup2, k, am, vm, tempk, tempt, tempp);
	double th4 = GetDTheta(tempk, tempt, v0);

	MATP(1, v0, vf, tup1, k, am, vm, tempk, tempt, tempp);
	double th5 = GetDTheta(tempk, tempt, v0);
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	double tmin;
	if (dth >= th1) {
		if (dth >= th5) {
			tmin = tup1 + (dth - th5) / vm;
		} else {
			if (dth >= th4) {
				double coE3 = am / 4;
				double coE4 = am * (3 * am + k * (tinc2 - 4 * tinc3)) / 2 / k
						+ v0;
				double coE5 = -dth - (104 * pow(am, 3) + 24 * pow(am, 2) * k
						* (tinc2 - 13 * tinc3) - 128 * pow(k, 3)
						* pow(tinc3, 3) + 3 * am * pow(k, 2) * (pow(tinc2, 2)
						- 8 * tinc2 * tinc3 + 112 * pow(tinc3, 2))) / (12.
						* pow(k, 2));
				double m = coE4 * coE4 - 4 * coE3 * coE5;
				if (m <= 0)
					m = 0;
				double tmine[2];
				double eps = 1e-15;
				tmine[0] = 2 * (-coE4 - sqrt(m)) / am;
				tmine[1] = 2 * (-coE4 + sqrt(m)) / am;
				for (int i = 0; i < 2; i++) {
					if (tmine[i] >= tup2 - eps && tmine[i] <= tup1 + eps) {
						tmin = tmine[i];
						break;
					}
				}
			} else {
				//求取Tmin
				Cal_Positive_Tmin(dth, th2, th3, tinc2, am, k, v0, vf, a6, a2,
						tmin, tup3, tup4, aainc);

			}

		}
	}
	MATP(1, v0, vf, tmin, k, am, vm, tempk, tempt, tempp);
	k1 = tempk[0];
	k2 = tempk[1];
	k3 = tempk[2];
	k4 = tempk[3];
	t1 = tempt[0];
	t2 = tempt[1];
	t3 = tempt[2];
	t4 = tempt[3];
	t5 = tempt[4];
	t6 = tempt[5];
	t7 = tempt[6];
	a2 = tempp[3];
	a6 = tempp[4];

	/*各个变量求得完毕*/

	/*以上为正向规划！*/
	double th10, th20, th30, th40, th50;
	//////////////////////////////////////////////////////////////////////////
	if (dth <= th1) {
		/*tinf,tup4,tup3,tup2,tup1依次表示各边界的时间*/
		tvb1 = am / k;
		tvb2 = (vf - v0) / am;
		tinf2 = tvb1 - tvb2;
		tinf3 = tvb1 + tvb2;
		tinf = tinf2 >= tinf3 ? tinf2 : tinf3;

		a2t = sqrt(k * (vf + vm));
		a6t = -sqrt(k * (v0 + vm));
		a2 = a2t < am ? a2t : am;
		a6 = a6t > -am ? a6t : -am;
		t4 = 0;
		/*中间变量*/
		tinc2 = (vf - v0) / am;
		tinc3 = am / k;
		aainc = am * k * tinc2;
		tup1 = (a2 + pow(a2t, 2) / a2 - a6 - pow(a6t, 2) / a6) / k;
		tup2 = fabs(tinc2) + 4 * tinc3;
		deltas = tinc3 - fabs(tinc2);

		if (deltas < 0) {
			tup3 = 0;
			tup4 = 0;
		} else {
			tup3 = 2 * tinc3 + 2 * sqrt(deltas * tinc3);
			tup4 = 2 * tinc3 - 2 * sqrt(tinc3 * deltas);
		}
		if (tup2 > tup1)
			tup2 = tup1;
		if (tup3 > tup2)
			tup3 = tup2;
		if (tup3 <= tinf)
			tup3 = tinf;
		if (tup4 >= tup3)
			tup4 = tup3;
		if (tup4 <= tinf)
			tup4 = tinf;
		MATP(-1, -vf, -v0, tinf, k, am, vm, tempk, tempt, tempp);
		th10 = -GetDTheta(tempk, tempt, v0);

		MATP(-1, -vf, -v0, tup4, k, am, vm, tempk, tempt, tempp);
		th20 = -GetDTheta(tempk, tempt, v0);

		MATP(-1, -vf, -v0, tup3, k, am, vm, tempk, tempt, tempp);
		th30 = -GetDTheta(tempk, tempt, v0);

		MATP(-1, -vf, -v0, tup2, k, am, vm, tempk, tempt, tempp);
		th40 = -GetDTheta(tempk, tempt, v0);

		MATP(-1, -vf, -v0, tup1, k, am, vm, tempk, tempt, tempp);
		th50 = -GetDTheta(tempk, tempt, v0);

		dth = -dth;
		/*求总时间*/
		if (dth >= th50) {
			tmin = tup1 + (dth - th50) / vm;
		} else {
			if (dth > th40) {
				double coE3 = am / 4;
				double coE4 = am * (3 * am + k * (tinc2 - 4 * tinc3)) / 2 / k
						- vf;
				double coE5 = -dth - (104 * pow(am, 3) + 24 * pow(am, 2) * k
						* (tinc2 - 13 * tinc3) - 128 * pow(k, 3)
						* pow(tinc3, 3) + 3 * am * pow(k, 2) * (pow(tinc2, 2)
						- 8 * tinc2 * tinc3 + 112 * pow(tinc3, 2))) / (12.
						* pow(k, 2));
				double m = coE4 * coE4 - 4 * coE3 * coE5;
				if (m <= 0)
					m = 0;
				double tmine[2];
				double eps = 1e-15;
				tmine[0] = 2 * (-coE4 - sqrt(m)) / am;
				tmine[1] = 2 * (-coE4 + sqrt(m)) / am;
				for (int i = 0; i < 2; i++) {
					if (tmine[i] >= tup2 - eps && tmine[i] <= tup1 + eps) {
						tmin = tmine[i];
						break;
					}
				}

			} else {
				//求取Tmin
				Cal_Negative_Tmin(dth, th2, th3, tinc2, am, k, v0, vf, a6, a2,
						tmin, tup3, tup4, aainc);
			}
		}
		MATP(-1, -vf, -v0, tmin, k, am, vm, tempk, tempt, tempp);
		k1 = tempk[0];
		k2 = tempk[1];
		k3 = tempk[2];
		k4 = tempk[3];
		t1 = tempt[0];
		t2 = tempt[1];
		t3 = tempt[2];
		t4 = tempt[3];
		t5 = tempt[4];
		t6 = tempt[5];
		t7 = tempt[6];
		a2 = tempp[3];
		a6 = tempp[4];

		dth = -dth;
	}
	if (dth >= -th10 && dth <= th1) {
		if (dth == 0 && v0 == vf) {
			memset(&sul, 0, sizeof(sul));
			return 0;
		}
//		cout << "无法规划！！！" << endl;
		return -1;
	} else {

		sul.w_b = v0;
		sul.l_v[0] = ((a0 + a2) * t1) / 2. + v0;
		sul.l_v[1] = sul.l_v[0] + t2 * a2;
		sul.l_v[2] = a2 * t3 + (k2 * pow(t3, 2)) / 2. + sul.l_v[1];
		sul.l_v[3] = sul.l_v[2];
		sul.l_v[4] = a6 * t5 - (k3 * pow(t5, 2)) / 2. + sul.l_v[3];
		sul.l_v[5] = sul.l_v[4] + t6 * a6;
		sul.l_v[6] = a6 * t7 + (k4 * pow(t7, 2)) / 2. + sul.l_v[5];
		sul.l_a2 = a2;
		sul.l_a6 = a6;
		sul.l_k[0] = k1;
		sul.l_k[1] = k2;
		sul.l_k[2] = k3;
		sul.l_k[3] = k4;

		sul.l_t[0] = t1;
		sul.l_t[1] = t2;
		sul.l_t[2] = t3;
		sul.l_t[3] = t4;
		sul.l_t[4] = t5;
		sul.l_t[5] = t6;
		sul.l_t[6] = t7;
		sul.Tmax = t1 + t2 + t3 + t4 + t5 + t6 + t7;
		return 0;
	}
}
void InCircleTrajectoryPlan::MATP(int type, double v0, double vf, double tf,
		double k, double am, double vm, double ke[4], double t[7],
		double param[6]) {
	double a2t = sqrt(k * (-v0 + vm));
	double a6t = -sqrt(k * (-vf + vm));
	double a2 = a2t < am ? a2t : am;
	double a6 = a6t > -am ? a6t : -am;
	double t4 = 0;

	double tinc2 = (vf - v0) / am;
	double tinc3 = am / k;
	double aainc = am * k * tinc2;

	double tup1 = (a2 + pow(a2t, 2) / a2 - a6 - pow(a6t, 2) / a6) / k;
	double tup2;
	tup2 = fabs(tinc2) + 4.0f * tinc3;
	double deltas;
	deltas = tinc3 - fabs(tinc2);
	//////////////////////////////////////////////////////////////////////////
	double tup3, tup4;
	if (deltas < 0) {
		tup3 = 0;
		tup4 = 0;
	} else {
		tup3 = 2 * tinc3 + 2 * sqrt(deltas * tinc3);
		tup4 = 2 * tinc3 - 2 * sqrt(deltas * tinc3);
	}
	if (tf >= tup1) {
		t4 = tf - tup1;
		tf = tup1;
	}
	double t2 = 0;
	double t6 = 0;
	a2 = am;
	a6 = -am;
	double ainc;
	if (tf >= tup2) {
		t2 = (tf + tinc2 - 4 * tinc3) / 2;
		t6 = (tf - tinc2 - 4 * tinc3) / 2;

	} else {
		if (tf >= tup3 || tf <= tup4) {
			if (tinc2 > 0) {
				a6 = am - sqrt(am * k * (tf - tinc2));
				t2 = tf - 2 * tinc3 + 2 * a6 / k;
			} else {
				a2 = -am + sqrt(am * k * (tf + tinc2));
				t6 = tf - 2 * tinc3 - 2 * a2 / k;
			}
		} else {
			ainc = k * (tf) / 2;
			aainc = am * k * tinc2;
			a2 = (aainc / ainc + ainc) / 2;
			a6 = (aainc / ainc - ainc) / 2;
		}
	}
	double t1, t7, t3, t5;
	t1 = a2 / k;
	t7 = -a6 / k;
	if (t4 > 0) {
		t3 = a2 / k;
		t5 = -a6 / k;
	} else {
		t3 = (a2 - a6) / 2 / k;
		t5 = t3;
	}
	if (type > 0) {
		ke[0] = k;
		ke[1] = -k;
		ke[2] = -k;
		ke[3] = k;
		t[0] = t1;
		t[1] = t2;
		t[2] = t3;
		t[3] = t4;
		t[4] = t5;
		t[5] = t6;
		t[6] = t7;
		param[0] = 0;
		param[1] = v0;
		param[2] = 0;
		param[3] = a2;
		param[4] = a6;
		param[5] = 0;
	} else {
		ke[0] = -k;
		ke[1] = k;
		ke[2] = k;
		ke[3] = -k;
		t[0] = t7;
		t[1] = t6;
		t[2] = t5;
		t[3] = t4;
		t[4] = t3;
		t[5] = t2;
		t[6] = t1;
		param[0] = 0;
		param[1] = -vf;
		param[2] = 0;
		param[3] = a6;
		param[4] = a2;
		param[5] = 0;
	}
}
double InCircleTrajectoryPlan::GetDTheta(double k[4], double t[7], double v0) {
	double t10 = t[0];
	double t20 = t[1] + t10;
	double t30 = t[2] + t20;
	double t40 = t[3] + t30;
	double t50 = t[4] + t40;
	double t60 = t[5] + t50;
	double tf0 = t[6] + t60;
	return (k[0] * pow(t10, 3) + k[1] * (-pow(t20, 3) + pow(t30, 3)) + k[2]
			* (-pow(t40, 3) + pow(t50, 3)) - k[3] * pow(t60, 3)) / 6. + ((k[0]
			* t10 + k[1] * (-t20 + t30) + k[2] * (-t40 + t50) - k[3] * t60)
			* pow(tf0, 2)) / 2. + (k[3] * pow(tf0, 3)) / 6. + (tf0 * (-(k[0]
			* pow(t10, 2)) + k[1] * (pow(t20, 2) - pow(t30, 2)) + k[2] * (pow(
			t40, 2) - pow(t50, 2)) + k[3] * pow(t60, 2) + 2 * v0)) / 2.;
}
void InCircleTrajectoryPlan::SolveN(double a, double b, double c, double d,
		double e, complex<double> x[4]) {
	complex<double> B(c * c - 3 * b * d + 12 * a * e);
	complex<double> A;
	complex<double> temp(-4 * pow(pow(c, 2) - 3 * b * d + 12 * a * e, 3) + pow(
			2 * pow(c, 3) - 9 * b * c * d + 27 * a * pow(d, 2) + 27 * pow(b, 2)
					* e - 72 * a * c * e, 2));
	A = pow(complex<double> (2 * pow(c, 3) - 9 * b * c * d + 27 * a * pow(d, 2)
			+ 27 * pow(b, 2) * e - 72 * a * c * e) + sqrt(temp),
			0.3333333333333333);
	complex<double> part1(-b / 4 / a);

	complex<double> part2;
	part2 = 0.5 * sqrt(complex<double> (b * b / 4 / a / a - 2 * c / 3 / a)
			+ complex<double> (pow(2, 0.333333) / 3 / a) * B / A + A / complex<
			double> (3 * pow(2, 0.333333) * a));

	complex<double> temp2;
	temp2 = complex<double> (b * b / 2 / a / a - 4 * c / 3 / a) - complex<
			double> (pow(2, 0.333333) / 3 / a) * B / A - A / complex<double> (3
			* pow(2, 0.333333) * a);
	complex<double> temp3;
	temp3 = complex<double> ((-b * b * b / a / a / a + 4 * b * c / a / a - 8
			* d / a) / 8.0f) / part2;
	complex<double> part3 = 0.5 * sqrt(temp2 - temp3);
	complex<double> part4 = 0.5 * sqrt(temp2 + temp3);
	x[0] = part1 - part2 - part3;
	x[1] = part1 - part2 + part3;
	x[2] = part1 + part2 - part4;
	x[3] = part1 + part2 + part4;

}

void InCircleTrajectoryPlan::Cal_Positive_Tmin(double &dth, double &th2,
		double &th3, double &tinc2, double &am, double &k, double &v0,
		double &vf, double &a6, double &a2, double &tmin, double &tup3,
		double &tup4, double &aainc) {
	if (dth >= th3 || dth <= th2) {
		if (tinc2 > 0) {
			complex<double> a60[4];
			double coE1 = 1 / (2 * am * k * k);
			double coE2 = -(1 / k / k);
			double coE3 = (am * am + 2 * k * vf) / (2 * am * k * k);
			double coE4 = (-2 * vf) / k;
			double coE5 = 1 / (24 * am * k * k) * (12 * k * (v0 + vf) * (am
					* am - k * v0 + k * vf)) - dth;
			SolveN(coE1, coE2, coE3, coE4, coE5, a60);
			for (int i = 0; i < 4; i++) {
				if (a60[i].imag() == 0 && a60[i].real() >= -am && a60[i].real()
						<= am) {
					a6 = a60[i].real();
					break;
				}
			}
			tmin = pow(-a6 + am, 2) / (am * k) + tinc2;
			//////////////////////////////////////////////////////////////////////////
		} else {
			complex<double> a20[4];
			double coE1 = 1 / (2 * am * k * k);
			double coE2 = (1 / k / k);
			double coE3 = (am * am + 2 * k * v0) / (2 * am * k * k);
			double coE4 = (2 * v0) / k;
			double coE5 = 1 / (24 * am * k * k) * (12 * k * (v0 + vf) * (am
					* am + k * v0 - k * vf)) - dth;
			SolveN(coE1, coE2, coE3, coE4, coE5, a20);
			//////////////////////////////////////////////////////////////////////////
			for (int i = 0; i < 4; i++) {
				if (a20[i].imag() == 0 && a20[i].real() >= -am && a20[i].real()
						<= am) {
					a2 = a20[i].real();
					break;
				}
			}
			tmin = pow(a2 + am, 2) / (am * k) - tinc2;
		}
	} else {
		complex<double> tmin0[4];

		double coE1 = k / 32;
		double coE2 = 0;
		double coE3 = aainc / (2 * k) + v0;
		double coE4 = -dth;
		double coE5 = -(aainc * aainc / (2 * k * k * k));
		SolveN(coE1, coE2, coE3, coE4, coE5, tmin0);
		double eps = 1e-15;
		for (int i = 0; i < 4; i++) {
			if (tmin0[i].imag() == 0 && tmin0[i].real() >= tup4 - eps
					&& tmin0[i].real() <= tup3 + eps) {
				tmin = tmin0[i].real();
				break;
			}
		}
	}

}
void InCircleTrajectoryPlan::Cal_Negative_Tmin(double &dth, double &th20,
		double &th30, double &tinc2, double &am, double &k, double &v0,
		double &vf, double &a6, double &a2, double &tmin, double &tup3,
		double &tup4, double &aainc) {
	if (dth > th30 || dth < th20) {
		if (tinc2 > 0) {
			complex<double> a60[4];
			double coE1 = 1 / (2 * am * k * k);
			double coE2 = -(1 / k / k);
			double coE3 = (am * am + 2 * k * v0) / (2 * am * k * k);
			double coE4 = (2 * v0) / k;
			double coE5 = 1 / (24 * am * k * k) * (-12 * k * (v0 + vf) * (am
					* am - k * v0 + k * vf)) - dth;
			//////////////////////////////////////////////////////////////////////////
			SolveN(coE1, coE2, coE3, coE4, coE5, a60);
			for (int i = 0; i < 4; i++) {
				if (a60[i].imag() == 0 && a60[i].real() >= -am && a60[i].real()
						<= am) {
					a6 = a60[i].real();
					break;
				}
			}
			tmin = pow(-a6 + am, 2) / (am * k) + tinc2;
		} else {
			complex<double> a20[4];
			double coE1 = 1 / (2 * am * k * k);
			double coE2 = (1 / k / k);
			double coE3 = (am * am - 2 * k * vf) / (2 * am * k * k);
			double coE4 = (-2 * vf) / k;
			double coE5 = 1 / (24 * am * k * k) * (-12 * k * (v0 + vf) * (am
					* am + k * v0 - k * vf)) - dth;
			SolveN(coE1, coE2, coE3, coE4, coE5, a20);
			for (int i = 0; i < 4; i++) {
				if (a20[i].imag() == 0 && a20[i].real() >= -am && a20[i].real()
						<= am) {
					a2 = a20[i].real();
					break;
				}
			}
			tmin = pow(a2 + am, 2) / (am * k) - tinc2;
		}

	} else {
		complex<double> tmin0[4];
		double coE1 = k / 32;
		double coE2 = 0;
		double coE3 = aainc / (2 * k) - vf;
		double coE4 = -dth;
		double coE5 = -(aainc * aainc / (2 * k * k * k));
		SolveN(coE1, coE2, coE3, coE4, coE5, tmin0);
		double eps = 1e-15;
		for (int i = 0; i < 4; i++) {
			if (tmin0[i].imag() == 0 && tmin0[i].real() >= tup4 - eps
					&& tmin0[i].real() <= tup3 + eps) {
				tmin = tmin0[i].real();
				break;
			}
		}
	}
}
void InCircleTrajectoryPlan::Kine_Move(trajectory_info temp_sul, double x_i,
		double y_i, double st_i, double &x_e, double &y_e, double &st_e) {//用于运动学迭代
	double w_l, w_r;
	x_e = x_i;
	y_e = y_i;
	st_e = st_i;
	while (SUCCEED == GetWheelVel(temp_sul, w_l, w_r)) {
		st_e += 0.001 * (w_r - w_l) * r / Le / 2;//st
		if (st_e >= pi)
			st_e -= 2 * pi;
		if (st_e <= -pi)
			st_e += 2 * pi;
		x_e += 0.001 * cos(st_e) * (w_l + w_r) * r / 2;//x
		y_e += 0.001 * sin(st_e) * (w_l + w_r) * r / 2;//x
	}
}
ReturnCode InCircleTrajectoryPlan::GetWheelVel(trajectory_info & sul,
		double &v_left, double &v_right) {//从轨迹规划的结果里计算设定速度
	if (sul.current_t >= sul.Tmax - 0.001f) {
		return FAILED;
	} else {
		sul.current_t += 0.001f;
		if (sul.tra_type == LINE__TRA) {
			v_left = GetCarTrajectory(sul);
			v_right = v_left;
		}
		if (sul.rel_type == TURN_LEFT) {
			v_left = sul.w_b - GetCarTrajectory(sul);
			v_right = sul.w_b + GetCarTrajectory(sul);
		}
		if (sul.rel_type == TURN_RIGHT) {
			v_left = sul.w_b + GetCarTrajectory(sul);
			v_right = sul.w_b - GetCarTrajectory(sul);
		}
		return SUCCEED;
	}
}
double InCircleTrajectoryPlan::GetCarTrajectory(trajectory_info &sul) {
	double value = 0;
	double t = sul.current_t;
	switch (sul.tra_type) {
	case LINE__TRA: {
		//加加速阶段
		if (t >= 0 && t < sul.l_t[0]) {
			value = (sul.l_k[0] * pow(t, 2)) / 2. + sul.w_b;
		}
		//匀加速阶段
		if (t >= sul.l_t[0] && t < sul.l_t[0] + sul.l_t[1]) {
			value = sul.l_v[0] + sul.l_a2 * (t - sul.l_t[0]);
		}
		//减加速阶段
		if (t >= sul.l_t[0] + sul.l_t[1] && t < sul.l_t[0] + sul.l_t[1]
				+ sul.l_t[2]) {
			value = sul.l_a2 * (t - sul.l_t[0] - sul.l_t[1]) + (sul.l_k[1]
					* pow(t - sul.l_t[0] - sul.l_t[1], 2)) / 2. + sul.l_v[1];
		}
		//匀速阶段
		if (t >= sul.l_t[0] + sul.l_t[1] + sul.l_t[2] && t < sul.l_t[0]
				+ sul.l_t[1] + sul.l_t[2] + sul.l_t[3]) {
			value = sul.l_v[2];
		}
		//加减速阶段
		if (t >= sul.l_t[0] + sul.l_t[1] + sul.l_t[2] + sul.l_t[3] && t
				< sul.l_t[0] + sul.l_t[1] + sul.l_t[2] + sul.l_t[3]
						+ sul.l_t[4]) {
			value = (sul.l_k[2] * pow(t - sul.l_t[0] - sul.l_t[1] - sul.l_t[2]
					- sul.l_t[3], 2)) / 2. + (t - sul.l_t[0] - sul.l_t[1]
					- sul.l_t[2] - sul.l_t[3]) * (sul.l_a6 - sul.l_k[2]
					* sul.l_t[4]) + sul.l_v[3];
		}
		//匀减速阶段
		if (t >= sul.l_t[0] + sul.l_t[1] + sul.l_t[2] + sul.l_t[3] + sul.l_t[4]
				&& t < sul.l_t[0] + sul.l_t[1] + sul.l_t[2] + sul.l_t[3]
						+ sul.l_t[4] + sul.l_t[5]) {
			value = sul.l_v[4] + sul.l_a6 * (t - sul.l_t[0] - sul.l_t[1]
					- sul.l_t[2] - sul.l_t[3] - sul.l_t[4]);
		}
		//减减速阶段
		if (t >= sul.l_t[0] + sul.l_t[1] + sul.l_t[2] + sul.l_t[3] + sul.l_t[4]
				+ sul.l_t[5] && t <= sul.l_t[0] + sul.l_t[1] + sul.l_t[2]
				+ sul.l_t[3] + sul.l_t[4] + sul.l_t[5] + sul.l_t[6]) {
			value = sul.l_a6 * (t - (sul.l_t[0] + sul.l_t[1] + sul.l_t[2]
					+ sul.l_t[3] + sul.l_t[4] + sul.l_t[5])) + (sul.l_k[3]
					* pow(t - (sul.l_t[0] + sul.l_t[1] + sul.l_t[2]
							+ sul.l_t[3] + sul.l_t[4] + sul.l_t[5]), 2)) / 2.
					+ sul.l_v[5];
		}
	}
		break;
	case CIRCLE_TRA: {
		if (t >= 0 && t < T / 4) {
			value = sul.amax * T / 2 / pi * (1 - cos(2 * pi / T * t));
		}
		if (t >= T / 4 && t < sul.s_t1 + T / 4) {
			value = sul.amax * (t - T / 4) + T * sul.amax / 2 / pi;
		}
		if (t >= sul.s_t1 + T / 4 && t < sul.s_t1 + T / 2) {
			value = sul.amax * T / 2 / pi * (1 - cos(2 * pi / T
					* (t - sul.s_t1))) + sul.amax * sul.s_t1;
		}

		if (t >= sul.s_t1 + T / 2 && t < sul.s_t1 + T / 2 + sul.s_t2) {
			value = sul.amax * (T / pi + sul.s_t1);
		}
		if (t >= sul.s_t1 + sul.s_t2 + T / 2 && t < sul.s_t1 + sul.s_t2 + T * 3
				/ 4) {
			value = sul.amax * T / 2 / pi * (1 - cos(2 * pi / T * (t - sul.s_t1
					- sul.s_t2))) + sul.amax * sul.s_t1;
		}
		if (t >= sul.s_t1 + sul.s_t2 + T * 3 / 4 && t < sul.s_t2 + 2 * sul.s_t1
				+ T * 3 / 4)

		////////////////////////////////////////////////////////////////
		{
			value = -sul.amax * (t - T * 3 / 4 - sul.s_t1 - sul.s_t2) + T
					* sul.amax / 2 / pi + sul.amax * sul.s_t1;
		}
		if (t >= sul.s_t2 + 2 * sul.s_t1 + T * 3 / 4 && t <= sul.s_t2 + 2
				* sul.s_t1 + T) {
			value = sul.amax * T / 2 / pi * (1 - cos(2 * pi / T * (t - 2
					* sul.s_t1 - sul.s_t2)));
		}
	}
		break;
	default:
		return 0;
	}
	return value;
}
