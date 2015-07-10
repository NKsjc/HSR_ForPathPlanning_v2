/*
 * EtaTrajectoryPlan.cpp
 *
 *  Created on: 2014-9-6
 *      Author: alvin
 */

#include "EtaTrajectoryPlan.h"
#include <math.h>
EtaTrajectoryPlan::EtaTrajectoryPlan() {
	// TODO Auto-generated constructor stub
	flag=0;
}

EtaTrajectoryPlan::~EtaTrajectoryPlan() {
	// TODO Auto-generated destructor stub
	eta_buff.clear();
	tra_path.clear();
}
void EtaTrajectoryPlan::AddTask(int taskid,double tra_info[]){
	eta_path temp_path;
	temp_path.begin_x=tra_info[0];
	temp_path.begin_y=tra_info[1];
	temp_path.begin_st=tra_info[2];
	temp_path.end_x=tra_info[3];
	temp_path.end_y=tra_info[4];
	temp_path.end_st=tra_info[5];
	for(int i=0;i<6;i++)temp_path.eta[i]=tra_info[6+i];
	tra_path.push_back(temp_path);
}
void EtaTrajectoryPlan::CleanTask(){
	eta_buff.clear();
	tra_path.clear();
}
void EtaTrajectoryPlan::GetPeriodRef(int & returnflag,double ref_value[],double cur_info[]){
	if((unsigned int)flag<eta_buff.size())
	{
		vel_wheel temp=eta_buff[flag];
		cur_info[0]=temp.v_l;
		cur_info[1]=temp.v_r;
		flag++;
		returnflag=0;
		return ;
	}
	else
	{
		flag=0;
		eta_buff.clear();
		if(tra_path.empty())
		{
			returnflag=1;
			return ;
		}
		else
		{
			eta_path temp=tra_path.front();
			Cal_Tra(temp);
			tra_path.pop_front();
			returnflag=2;
			return ;
		}
	}
}
void EtaTrajectoryPlan::Cal_Tra(eta_path new_path)
{


	const double d_t=0.001;double fac=1;
	double xa,ya,t_a,xb,yb,t_b,n1,n2,n3,n4,n5,n6;
	double ka=0,d_ka=0,kb=0,d_kb=0;
	double a0,a1,a2,a3,a4,a5,a6,a7,b0,b1,b2,b3,b4,b5,b6,b7;
	xa=new_path.begin_x;
	ya=new_path.begin_y;
	t_a=new_path.begin_st;
	xb=new_path.end_x;
	yb=new_path.end_y;
	t_b=new_path.end_st;
	n1=new_path.eta[0];n2=new_path.eta[1];n3=new_path.eta[2];n4=new_path.eta[3];n5=new_path.eta[4];n6=new_path.eta[5];
//	cout<<xa<<" "<<ya<<"  "<<t_a<<"  "<<xb<<" "<<yb<<"  "<<t_b<<" "<<n1<<" "<<n2<<" "<<n3<<" "<<n4<<" "<<n5<<" "<<n6<<endl;
//////////////////////////////////////////////
	a0=xa;
	a1=n1*cos(t_a);
	a2=0.5*n3*cos(t_a)-0.5*pow(n1,2)*ka*sin(t_a);
	a3=1.0f/6*n5*cos(t_a)-1.0f/6*(pow(n1,3)*d_ka+3*n1*n3*ka)*sin(t_a);
	a4=35*(xb-xa)-(20*n1+5*n3+2.0f/3*n5)*cos(t_a)+(5*pow(n1,2)*ka+2.0f/3*pow(n1,3)*d_ka+2*n1*n3*ka)*sin(t_a)-(15*n2-2.5*n4+1.0f/6*n6)*cos(t_b)-(2.5*pow(n2,2)*kb-1.0f/6*pow(n2,3)*d_kb-0.5*n2*n4*kb)*sin(t_b);
	a5=-84*(xb-xa)+(45*n1+10*n3+n5)*cos(t_a)-(10*pow(n1,2)*ka+pow(n1,3)*d_ka+3*n1*n3*ka)*sin(t_a)+(39*n2-7*n4+0.5*n6)*cos(t_b)+(7*pow(n2,2)*kb-0.5*pow(n2,3)*d_kb-1.5*n2*n4*kb)*sin(t_b);
	a6=70*(xb-xa)-(36*n1+7.5*n3+2.0f/3*n5)*cos(t_a)+(7.5*pow(n1,2)*ka+2.0f/3*pow(n1,3)*d_ka+2*n1*n3*ka)*sin(t_a)-(34*n2-6.5*n4+0.5*n6)*cos(t_b)-(6.5*pow(n2,2)*kb-0.5*pow(n2,3)*d_kb-1.5*n2*n4*kb)*sin(t_b);
	a7=-20*(xb-xa)+(10*n1+2*n3+1.0f/6*n5)*cos(t_a)-(2*pow(n1,2)*ka+1.0f/6*pow(n1,3)*d_ka+0.5*n1*n3*ka)*sin(t_a)+(10*n2-2*n4+1.0f/6*n6)*cos(t_b)+(2*pow(n2,2)*kb-1.0f/6*pow(n2,3)*d_kb-0.5*n2*n4*kb)*sin(t_b);
	b0=ya;
	b1=n1*sin(t_a);
	b2=0.5*n3*sin(t_a)+0.5*pow(n1,2)*ka*cos(t_a);
	b3=1.0f/6*n5*sin(t_a)+1.0f/6*(pow(n1,3)*d_ka+3*n1*n3*ka)*cos(t_a);
	b4=35*(yb-ya)-(20*n1+5*n3+2.0f/3*n5)*sin(t_a)-(5*pow(n1,2)*ka+2.0f/3*pow(n1,3)*d_ka+2*n1*n3*ka)*cos(t_a)-(15*n2-2.5*n4+1.0f/6*n6)*sin(t_b)+(2.5*pow(n2,2)*kb-1.0f/6*pow(n2,3)*d_kb-0.5*n2*n4*kb)*cos(t_b);
	b5=-84*(yb-ya)+(45*n1+10*n3+n5)*sin(t_a)+(10*pow(n1,2)*ka+pow(n1,3)*d_ka+3*n1*n3*ka)*cos(t_a)+(39*n2-7*n4+0.5*n6)*sin(t_b)-(7*pow(n2,2)*kb-0.5*pow(n2,3)*d_kb-1.5*n2*n4*kb)*cos(t_b);
	b6=70*(yb-ya)-(36*n1+7.5*n3+2.0f/3*n5)*sin(t_a)-(7.5*pow(n1,2)*ka+2.0f/3*pow(n1,3)*d_ka+2*n1*n3*ka)*cos(t_a)-(34*n2-6.5*n4+0.5*n6)*sin(t_b)+(6.5*pow(n2,2)*kb-0.5*pow(n2,3)*d_kb-1.5*n2*n4*kb)*cos(t_b);
	b7=-20*(yb-ya)+(10*n1+2*n3+1.0f/6*n5)*sin(t_a)+(2*pow(n1,2)*ka+1.0f/6*pow(n1,3)*d_ka+1.0f/2*n1*n3*ka)*cos(t_a)+(10*n2-2*n4+1.0f/6*n6)*sin(t_b)-(2*pow(n2,2)*kb-1.0f/6*pow(n2,3)*d_kb-0.5*n2*n4*kb)*cos(t_b);
//	cout<<a0<<"  "<<a1<<"  "<<a2<<"  "<<a3<<"  "<<a4<<"  "<<a5<<"  "<<a6<<"  "<<a7<<endl;
//	cout<<b0<<"  "<<b1<<"  "<<b2<<"  "<<b3<<"  "<<b4<<"  "<<b5<<"  "<<b6<<"  "<<b7<<endl;
//////////////////////////////////////////////
	eta_buff.clear();
	vel_wheel temp_vel;
	double d_x,d_y,v_x,v_y,st,vel,w,la_st;
	eta_buff.push_back(temp_vel);
	la_st=atan2(b1,a1);
	double t;
	double max=app_setting.get_setting_param().car_tra_max_vel;
	for(double tm=d_t;tm<3.1415926*fac;tm+=d_t)
	{
		t=0.5*cos(1/fac*tm-pi)+0.5;
		d_x=a1+2*a2*pow(t,1)+3*a3*pow(t,2)+4*a4*pow(t,3)+5*a5*pow(t,4)+6*a6*pow(t,5)+7*a7*pow(t,6);
		d_y=b1+2*b2*pow(t,1)+3*b3*pow(t,2)+4*b4*pow(t,3)+5*b5*pow(t,4)+6*b6*pow(t,5)+7*b7*pow(t,6);
		v_x=-d_x*sin(1/fac*tm-pi)/fac/2;
		v_y=-d_y*sin(1/fac*tm-pi)/fac/2;
		vel=sqrt(v_x*v_x+v_y*v_y);
		st=atan2(d_y,d_x);
		w=(st-la_st)/d_t;
		temp_vel.v_l=(vel-w*Le)/r;
		temp_vel.v_r=(vel+w*Le)/r;
		la_st=st;
		if(fabs(temp_vel.v_l)>=max||fabs(temp_vel.v_r)>=max)
		{
			eta_buff.clear();
			fac+=1;
			tm=d_t;
			la_st=atan2(b1,a1);
		}
		else
		{
			eta_buff.push_back(temp_vel);
		}
	}


}
