/*
 * P2PTrajectoryPlan.cpp
 *
 *  Created on: 2014-9-6
 *      Author: alvin
 */

#include "P2PTrajectoryPlan.h"
#include <string.h>
#include <math.h>
#define DELTAST 0.5
P2PTrajectoryPlan::P2PTrajectoryPlan() {
	// TODO Auto-generated constructor stub
	memset(hand_tra,0,sizeof(hand_tra));
	last_tra_end=1;
}

P2PTrajectoryPlan::~P2PTrajectoryPlan() {
	// TODO Auto-generated destructor stub
	this->CleanTask();
}
void P2PTrajectoryPlan::AddTask(int taskid,double tra_info[]){
//	cout<<"new target st";
	hand_routine_info temp_routine;
	temp_routine.st[0]=tra_info[0];
	temp_routine.st[1]=tra_info[1];
	temp_routine.st[2]=tra_info[2];
	temp_routine.st[3]=tra_info[3];
	temp_routine.end_flag=0;
//	cout<<tra_info[0]<<"  "<<tra_info[1]<<"  "<<tra_info[2]<<"  "<<tra_info[3]<<endl;
	this->hsr_hand_routine.push_back(temp_routine);
}
void P2PTrajectoryPlan::CleanTask(){
	this->hsr_hand_routine.clear();
	this->last_tra_end=1;
	for(int i=0;i<4;i++)memset(&hand_tra[i],0,sizeof(hand_tra[i]));
}

void P2PTrajectoryPlan::GetPeriodRef(int & flag,double ref_value[],double cur_info[]){

	for(int i=0;i<4;i++)
	{
		if(hand_tra[i].Tmax==0)
		{
			ref_value[i]=0;
			flag|=(1<<i);
			
		}
		else
		{
			ref_value[i]=Get_Hand_Trajectory(hand_tra[i])*0.001*(hand_tra[i].Tmax/(hand_tra[i].Tmax+hand_tra[i].synctime));

			hand_tra[i].current_t+=0.001*(hand_tra[i].Tmax/(hand_tra[i].Tmax+hand_tra[i].synctime));///////
			if(hand_tra[i].current_t>=hand_tra[i].Tmax)
			{
				memset(&hand_tra[i],0,sizeof(hand_tra[i]));
			}
		}
	}
//	hsrlog.Print(_Debug,"current set :%4f  %4f   %4f  %4f\n",ref_value[0],ref_value[1],ref_value[2],ref_value[3]);
	if (flag==15)
	{
//		hsrlog.Print(_Debug,"tra is empty,need new target to tra\n");
		last_tra_end=1;
		if(!this->hsr_hand_routine.empty())
		{
			hsrlog.Print(_Debug,"target rountine is ok ,so tra\n");
			hand_routine_info temp_routine;
			temp_routine=this->hsr_hand_routine.front();
			this->hsr_hand_routine.pop_front();
			double w_b[4]={0,0,0,0};double w_e[4]={0,0,0,0};
//			hsrlog.Print(_Debug,"current st:%4f  %4f  %4f  %4f\n",cur_info[0],cur_info[1],cur_info[2],cur_info[3]);
		
			flag=Trajectory_From_Target(cur_info,temp_routine.st,w_b,w_e);
//		cout<<"traject result"<<res<<endl;
			int ttflag=0;
			double begin[4];hand_trajectory_info temp_tra[4];
			for(int i=0;i<4;i++)
			{
				begin[i]=cur_info[i];
				temp_tra[i]=hand_tra[i];
			}
			while(ttflag!=15)
			{
				for(int i=0;i<4;i++)
				{
					if(temp_tra[i].Tmax==0)
					{
						begin[i]+=0;
						ttflag|=(1<<i);
					}
					else
					{
						begin[i]+=Get_Hand_Trajectory(temp_tra[i])*0.001;
						temp_tra[i].current_t+=0.001;
						if(temp_tra[i].current_t>=temp_tra[i].Tmax)
						{
							memset(&temp_tra[i],0,sizeof(temp_tra[i]));
						}
					}
				}
				if(Check_Collision(begin)==1)
				{
				}
			}
		}
	}
//			hsrlog.Print(_Debug,"ref %4f  %4f  %4f  %4f\n",ref_value[0],ref_value[1],ref_value[2],ref_value[3]);
//		ref_value[0]=ref_value[1]=ref_value[2]=0;
//			ref_value[3]=0;
//		ref_value[0]=ref_value[1]=ref_value[2]=ref_value[3]=sin(double(timete)/5000*2*3.1415926)*0.00015;
//		timete++;
//		if(timete==5000)timete=0;
//	return;
	
}
int P2PTrajectoryPlan::Check_Collision(double current_st[4])//Œì²éµ±Ç°¹ìŒ£ÊÇ·ñ»ážÉÉæ»úÆ÷ÈË×ÔÉí
{
/*	const double L1=0.3361;const double L2=0.1675;const double aw=0.095;const double origin_1Z=0.0795;
	//ÏÈÅÐ¶ÏÖâ¹ØœÚµÄÎ»ÖÃ
	double position_hand[3];
	position_hand[0]=L1*cos(current_st[0])*cos(current_st[1]);//x
	position_hand[1]=L1*sin(current_st[0])*cos(current_st[1]);//y
	position_hand[2]=origin_1Z-L1*sin(current_st[1]);//z
	//ÅÐ¶ÏÊÇ·ñžÉÉæ
	if(position_hand[2]<-0.1)//100mm×ó²àÎªÉíÌå
		return 1;
	//ÇóÈ¡Íó¹ØœÚÎ»ÖÃ
	double position_wrist[3];
	position_wrist[0]=L1*cos(current_st[0])*cos(current_st[1])-L2*cos(current_st[0])*sin(current_st[1]+current_st[2]);
	position_wrist[1]=L1*sin(current_st[0])*cos(current_st[1])-L2*sin(current_st[0])*sin(current_st[1]+current_st[2]);
	position_wrist[2]=origin_1Z-L1*sin(current_st[1])-L2*cos(current_st[1]+current_st[2]);
	//ÅÐ¶ÏÊÇ·ñžÉÉæ
	if(position_wrist[2]<-0.1)
		return 1;
	//ÇóÈ¡Ä©¶ËÎ»ÖÃ
	double Target[4][4];
	Kine(Target,current_st[0],current_st[1],current_st[2],current_st[3]);
	//ÅÐ¶ÏÊÇ·ñžÉÉæ
	if(Target[2][3]<-0.1)
		return 1;

	return 0;*/
	return 0;
}
double P2PTrajectoryPlan::Get_Hand_Trajectory(hand_trajectory_info &sul)//»ñÈ¡¹ìŒ£¹æ»®µÄµã
{
	double value=0;
	double t=sul.current_t;
	if(sul.Tmax==0)
	{
		return 0;
	}
	value=DELTAST*sul.flag;
	return value;
}

int P2PTrajectoryPlan::Trajectory_From_Target(double st_b[4], double st_e[4],double w_b[4],double w_e[4])
{
	int i=0;
	int res=0;
	for(i=0;i<4;i++)
	{
		hsrlog.Print(_Debug,"begin trajectory of joint %d  ,the input is current: %4f  %4f   %4f  %4f  target:   %4f  %4f   %4f  %4f\n"
		,i+1,st_b[0],st_b[1],st_b[2],st_b[3],st_e[0],st_e[1],st_e[2],st_e[3]);
		this->hand_tra[i].Tmax=fabs(st_e[i]-st_b[i])/DELTAST;
		if(st_e[i]>=st_b[i])
			this->hand_tra[i].flag=1;
			else
				this->hand_tra[i].flag=-1;
		res=0;
//		res=this->Cal_Trajectory(st_b[i],st_e[i],w_b[i],w_e[i],this->hand_tra[i]);
		if(res!=0)
		{
			hsrlog.Print(_Debug,"the trajectory of joint %d is failed\n",i+1);
			memset(hand_tra,0,sizeof(hand_tra));
			return -1;
		}
	}
	double tmax=this->hand_tra[0].Tmax;
	for(i=1;i<4;i++){
		if(tmax<=this->hand_tra[i].Tmax){
			tmax=this->hand_tra[i].Tmax;
		}
	}
	for(i=0;i<4;i++){
		this->hand_tra[i].synctime=tmax-this->hand_tra[i].Tmax;
	}
		hsrlog.Print(_Debug,"trajectory is completed:Tmax=%4f \n",tmax);
	last_tra_end=0;
//	cout<<"trajectory suceed"<<endl;
	return 0;
}

int P2PTrajectoryPlan::Cal_Trajectory(double st_b,double st_e,double v0,double vf,hand_trajectory_info &sul)
{
	if (fabs(v0)<0.001&&fabs(vf)<0.001&&fabs(st_b-st_e)<0.005)
	{
		memset(&sul,0,sizeof(sul));
		return 0;
	}
	double HAND_A_MAX=app_setting.get_setting_param().rigidhand_tra_max_acc;
	double HAND_V_MAX=app_setting.get_setting_param().rigidhand_tra_max_vel;

	memset(&sul,0,sizeof(sul));
	double a0,af,am,vm,k,th0,thf,dth;
	a0 = 0; af = 0;
	am = HAND_A_MAX;
	vm = HAND_V_MAX;
	k = 1.5;
	th0 = st_b;
	thf =st_e;
	dth = thf - th0;
	//////////////////////////////////////////////////////////////////////////
	double tvb1 = am/k;
	double tvb2 =  (vf - v0)/am ;
	double tinf2 = tvb1 + tvb2;
	double tinf3 = tvb1 - tvb2;
	double tinf = tinf2>=tinf3?tinf2:tinf3;

	double a2t = sqrt(k*(-v0 + vm));
	double a6t = -sqrt(k*(-vf + vm));
	double a2=a2t<am?a2t:am;
	double a6=a6t>-am?a6t:-am;
	double t4 = 0;

	double tinc2 = (vf - v0)/am;
	double tinc3 = am/k;
	double aainc = am*k*tinc2;

	double tup1 = (a2 + pow(a2t,2)/a2 - a6 - pow(a6t,2)/a6)/k;
	double tup2 = fabs(tinc2) + 4*tinc3;
	double deltas = tinc3 - fabs(tinc2);
	//////////////////////////////////////////////////////////////////////////
	double tup3,tup4;
	if (deltas < 0)
	{
		tup3 = 0; tup4 = 0;
	}
	else
	{
		tup3=2*tinc3 + 2*sqrt(deltas*tinc3);
		tup4=2*tinc3 - 2*sqrt(deltas*tinc3);
	}

	if(tup2 > tup1)tup2 = tup1;
	if(tup3 > tup2)tup3 = tup2;
	if(tup3 <= tinf)tup3 = tinf;
	if(tup4 >= tup3)tup4 = tup3;
	if(tup4 <= tinf)tup4 = tinf;
	//////////////////////////////////////////////////////////////////////////
	double tempk[4],tempt[7],tempp[6];
	double k1,k2,k3,k4;double t1,t2,t3,t5,t6,t7;
	MATP(1,v0,vf,tinf,k,am,vm,tempk,tempt,tempp);
	double th1=GetDTheta(tempk,tempt,v0);

	MATP(1,v0,vf,tup4,k,am,vm,tempk,tempt,tempp);
	double th2=GetDTheta(tempk,tempt,v0);

	MATP(1,v0,vf,tup3,k,am,vm,tempk,tempt,tempp);
	double th3=GetDTheta(tempk,tempt,v0);

	MATP(1,v0,vf,tup2,k,am,vm,tempk,tempt,tempp);
	double th4=GetDTheta(tempk,tempt,v0);

	MATP(1,v0,vf,tup1,k,am,vm,tempk,tempt,tempp);
	double th5=GetDTheta(tempk,tempt,v0);
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	double tmin;
	if (dth>=th1)
	{
		if (dth >= th5)
		{
			tmin=tup1 + (dth - th5)/vm;
		}
		else
		{
			if (dth >= th4)
			{
				double coE3=am/4;
				double coE4=am*(3*am+k*(tinc2-4*tinc3))/2/k+v0;
				double coE5=-dth - (104*pow(am,3) +  24*pow(am,2)*k*(tinc2 - 13*tinc3) - 128*pow(k,3)*pow(tinc3,3) + 3*am*pow(k,2)*(pow(tinc2,2) - 8*tinc2*tinc3 + 112*pow(tinc3,2)))/(12.*pow(k,2));
				double m=coE4*coE4-4*coE3*coE5;
				if(m<=0)m=0;
				double tmine[2];
				double eps=1e-15;
				tmine[0]=2*(-coE4-sqrt(m))/am;
				tmine[1]=2*(-coE4+sqrt(m))/am;
				for(int i=0;i<2;i++)
				{
					if(tmine[i]>=tup2-eps&&tmine[i]<=tup1+eps)
					{
						tmin=tmine[i];
						break;
					}
				}
			}
			else
			{
				//ÇóÈ¡Tmin
				Cal_Positive_Tmin(dth,th2,th3,tinc2,am,k,v0,vf,a6,a2,tmin,tup3,tup4,aainc);

			}

		}
	}
	MATP(1,v0,vf,tmin,k,am,vm,tempk,tempt,tempp);
	k1=tempk[0];
	k2=tempk[1];
	k3=tempk[2];
	k4=tempk[3];
	t1=tempt[0];t2=tempt[1];t3=tempt[2];t4=tempt[3];t5=tempt[4];t6=tempt[5];t7=tempt[6];
	a2=tempp[3];
	a6=tempp[4];



	/*ž÷žö±äÁ¿ÇóµÃÍê±Ï*/

	/*ÒÔÉÏÎªÕýÏò¹æ»®£¡*/
	double th10,th20,th30,th40,th50;
	//////////////////////////////////////////////////////////////////////////
	if (dth <=th1)
	{
		/*tinf,tup4,tup3,tup2,tup1ÒÀŽÎ±íÊŸž÷±ßœçµÄÊ±Œä*/
		tvb1 = am/k;
		tvb2 =  (vf - v0)/am ;
		tinf2 = tvb1 - tvb2;
		tinf3 = tvb1 + tvb2;
		tinf = tinf2>=tinf3?tinf2:tinf3;

		a2t = sqrt(k*(vf + vm));
		a6t = -sqrt(k*(v0 + vm));
		a2=a2t<am?a2t:am;
		a6=a6t>-am?a6t:-am;
		t4 = 0;
		/*ÖÐŒä±äÁ¿*/
		tinc2 = (vf - v0)/am;
		tinc3 = am/k;
		aainc = am*k*tinc2;
		tup1 = (a2 + pow(a2t,2)/a2 - a6 - pow(a6t,2)/a6)/k;
		tup2 = fabs(tinc2)+ 4*tinc3;
		deltas = tinc3 - fabs(tinc2);

		if (deltas < 0)
		{
			tup3 = 0; tup4 = 0;
		}
		else
		{
			tup3 = 2*tinc3 + 2*sqrt(deltas*tinc3);
			tup4 = 2*tinc3 - 2*sqrt(tinc3*deltas);
		}
		if(tup2 > tup1)tup2 = tup1;
		if(tup3 > tup2)tup3 = tup2;
		if(tup3 <= tinf)tup3 = tinf;
		if(tup4 >= tup3)tup4 = tup3;
		if(tup4 <= tinf)tup4 = tinf;
		MATP(-1,-vf,-v0,tinf,k,am,vm,tempk,tempt,tempp);
		th10=-GetDTheta(tempk,tempt,v0);

		MATP(-1,-vf,-v0,tup4,k,am,vm,tempk,tempt,tempp);
		th20=-GetDTheta(tempk,tempt,v0);

		MATP(-1,-vf,-v0,tup3,k,am,vm,tempk,tempt,tempp);
		th30=-GetDTheta(tempk,tempt,v0);

		MATP(-1,-vf,-v0,tup2,k,am,vm,tempk,tempt,tempp);
		th40=-GetDTheta(tempk,tempt,v0);

		MATP(-1,-vf,-v0,tup1,k,am,vm,tempk,tempt,tempp);
		th50=-GetDTheta(tempk,tempt,v0);

		dth = -dth;
		/*Çó×ÜÊ±Œä*/
		if (dth >= th50)
		{
			tmin = tup1 + (dth - th50)/vm;
		}
		else
		{
			if (dth > th40)
			{
				double coE3=am/4;
				double coE4=am*(3*am+k*(tinc2-4*tinc3))/2/k-vf;
				double coE5=-dth - (104*pow(am,3) +  24*pow(am,2)*k*(tinc2 - 13*tinc3) - 128*pow(k,3)*pow(tinc3,3) + 3*am*pow(k,2)*(pow(tinc2,2) - 8*tinc2*tinc3 + 112*pow(tinc3,2)))/(12.*pow(k,2));
				double m=coE4*coE4-4*coE3*coE5;
				if(m<=0)m=0;
				double tmine[2];
				double eps=1e-15;
				tmine[0]=2*(-coE4-sqrt(m))/am;
				tmine[1]=2*(-coE4+sqrt(m))/am;
				for(int i=0;i<2;i++)
				{
					if(tmine[i]>=tup2-eps&&tmine[i]<=tup1+eps)
					{
						tmin=tmine[i];
						break;
					}
				}

			}
			else
			{
				//ÇóÈ¡Tmin
				//				Cal_Positive_Tmin(dth,th2,th3,tinc2,am,k,-vf,-v0,a6,a2,tmin,tup3,tup4,aainc);
				Cal_Negative_Tmin(dth,th2,th3,tinc2,am,k,v0,vf,a6,a2,tmin,tup3,tup4,aainc);
			}
		}
		MATP(-1,-vf,-v0,tmin,k,am,vm,tempk,tempt,tempp);
		k1=tempk[0];k2=tempk[1];k3=tempk[2];k4=tempk[3];
		t1=tempt[0];t2=tempt[1];t3=tempt[2];t4=tempt[3];t5=tempt[4];t6=tempt[5];t7=tempt[6];
		a2=tempp[3];
		a6=tempp[4];

		dth = -dth;
	}
	if (dth >= -th10 && dth <= th1)
	{
		if (dth==0&&v0==vf)
		{
			memset(&sul,0,sizeof(sul));
			return 0;
		}

//		cout<<"ÎÞ·š¹æ»®£¡£¡£¡"<<endl;
		return -1;
	}
	else
	{

		sul.w_b=v0;
		sul.l_v[0] = ((a0 + a2)*t1)/2. + v0;
		sul.l_v[1]= sul.l_v[0] + t2*a2;
		sul.l_v[2]=a2*t3 + (k2*pow(t3,2))/2. +sul.l_v[1];
		sul.l_v[3]= sul.l_v[2];
		sul.l_v[4]= a6*t5 - (k3*pow(t5,2))/2. + sul.l_v[3];
		sul.l_v[5]= sul.l_v[4] + t6*a6;
		sul.l_v[6]= a6*t7 + (k4*pow(t7,2))/2. + sul.l_v[5];
		sul.l_a2=a2;
		sul.l_a6=a6;
		sul.l_k[0]=k1;
		sul.l_k[1]=k2;
		sul.l_k[2]=k3;
		sul.l_k[3]=k4;

		sul.l_t[0]=t1;
		sul.l_t[1]=t2;
		sul.l_t[2]=t3;
		sul.l_t[3]=t4;
		sul.l_t[4]=t5;
		sul.l_t[5]=t6;
		sul.l_t[6]=t7;
		sul.Tmax=t1+t2+t3+t4+t5+t6+t7;
		return 0;
	}
}
void P2PTrajectoryPlan::MATP(int type,double v0, double vf,  double tf, double k, double am, double vm,double ke[4],double t[7], double param[6])
{
	double a2t = sqrt(k*(-v0 + vm));
	double a6t = -sqrt(k*(-vf + vm));
	double a2=a2t<am?a2t:am;
	double a6=a6t>-am?a6t:-am;
	double t4 = 0;

	double tinc2 = (vf - v0)/am;
	double tinc3 = am/k;
	double aainc = am*k*tinc2;

	double tup1 = (a2 + pow(a2t,2)/a2 - a6 - pow(a6t,2)/a6)/k;
	double tup2 ;
	tup2=fabs(tinc2) +4.0f*tinc3;
	double deltas ;
	deltas=tinc3- fabs(tinc2);
	//////////////////////////////////////////////////////////////////////////
	double tup3,tup4;
	if (deltas < 0)
	{
		tup3 = 0; tup4 = 0;
	}
	else
	{
		tup3=2*tinc3 + 2*sqrt(deltas*tinc3);
		tup4=2*tinc3 - 2*sqrt(deltas*tinc3);
	}
	if (tf>=tup1)
	{
		t4=tf-tup1;
		tf=tup1;
	}
	double t2 = 0; double t6 = 0; a2 = am; a6 = -am;
	double ainc;
	if (tf>=tup2)
	{
		t2 = (tf + tinc2 - 4*tinc3)/2;
		t6 = (tf - tinc2 - 4*tinc3)/2;

	}
	else
	{
		if (tf >= tup3 || tf <= tup4)
		{
			if (tinc2>0)
			{
				a6=am-sqrt(am*k*(tf-tinc2));
				t2=tf-2*tinc3+2*a6/k;
			}
			else
			{
				a2=-am+sqrt(am*k*(tf+tinc2));
				t6=tf-2*tinc3-2*a2/k;
			}
		}
		else
		{
			ainc = k*(tf)/2;
			aainc = am*k*tinc2;
			a2 = (aainc/ainc + ainc)/2;
			a6 = (aainc/ainc - ainc)/2;
		}
	}
	double t1,t7,t3,t5;
	t1=a2/k;
	t7=-a6/k;
	if (t4>0)
	{
		t3=a2/k;
		t5=-a6/k;
	}
	else
	{
		t3=(a2-a6)/2/k;
		t5=t3;
	}
	if (type>0)
	{
		ke[0]=k;ke[1]=-k;ke[2]=-k;ke[3]=k;
		t[0]=t1;t[1]=t2;t[2]=t3;t[3]=t4;t[4]=t5;t[5]=t6;t[6]=t7;
		param[0]=0;param[1]=v0;param[2]=0;param[3]=a2;param[4]=a6;param[5]=0;
	}
	else
	{
		ke[0]=-k;ke[1]=k;ke[2]=k;ke[3]=-k;
		t[0]=t7;t[1]=t6;t[2]=t5;t[3]=t4;t[4]=t3;t[5]=t2;t[6]=t1;
		param[0]=0;param[1]=-vf;param[2]=0;param[3]=a6;param[4]=a2;param[5]=0;
	}
}
double P2PTrajectoryPlan::GetDTheta(double k[4],double t[7],double v0)
{
	double t10=t[0];
	double t20=t[1]+t10;
	double t30=t[2]+t20;
	double t40=t[3]+t30;
	double t50=t[4]+t40;
	double t60=t[5]+t50;
	double tf0=t[6]+t60;
	return (k[0]*pow(t10,3) + k[1]*(-pow(t20,3) + pow(t30,3)) + k[2]*(-pow(t40,3) + pow(t50,3)) - k[3]*pow(t60,3))/6. +
		((k[0]*t10 + k[1]*(-t20 + t30) + k[2]*(-t40 + t50) - k[3]*t60)*pow(tf0,2))/2. + (k[3]*pow(tf0,3))/6. + (tf0*(-(k[0]*pow(t10,2)) +
		k[1]*(pow(t20,2) - pow(t30,2)) + k[2]*(pow(t40,2) - pow(t50,2)) + k[3]*pow(t60,2) + 2*v0))/2.;
}
void P2PTrajectoryPlan::SolveN(double a,double b,double c,double d,double e,complex<double> x[4])
{
	complex<double>B(c*c- 3*b*d + 12*a*e);
	complex<double>A;
	complex<double>temp(-4*pow(pow(c,2) - 3*b*d + 12*a*e,3) + pow(2*pow(c,3) - 9*b*c*d + 27*a*pow(d,2) + 27*pow(b,2)*e - 72*a*c*e,2));
	A=pow(complex<double>(2*pow(c,3) - 9*b*c*d + 27*a*pow(d,2) + 27*pow(b,2)*e - 72*a*c*e )+ sqrt(temp),0.3333333333333333);
	complex<double>part1(-b/4/a);


	complex<double>part2;
	part2=0.5*sqrt(complex<double>(b*b/4/a/a-2*c/3/a)+complex<double>(pow(2,0.333333)/3/a)*B/A+A/complex<double>(3*pow(2,0.333333)*a));


	complex<double>temp2;
	temp2=complex<double>(b*b/2/a/a-4*c/3/a)-complex<double>(pow(2,0.333333)/3/a)*B/A-A/complex<double>(3*pow(2,0.333333)*a);
	complex<double>temp3;
	temp3=complex<double>((-b*b*b/a/a/a+4*b*c/a/a-8*d/a)/8.0f)/part2;
	complex<double>part3=0.5*sqrt(temp2-temp3);
	complex<double>part4=0.5*sqrt(temp2+temp3);
	x[0]=part1-part2-part3;
	x[1]=part1-part2+part3;
	x[2]=part1+part2-part4;
	x[3]=part1+part2+part4;

}


void P2PTrajectoryPlan::Cal_Positive_Tmin(double &dth,double &th2,double &th3,double &tinc2,double &am,double &k,double &v0,
	double &vf,double &a6,double &a2,double &tmin,double &tup3,double &tup4,double &aainc)
{
	if (dth >= th3 || dth <= th2)
	{
		if (tinc2>0)
		{
			complex<double> a60[4];
			double coE1 = 1/(2*am*k*k);
			double coE2 = -(1/k/k);
			double coE3 = (am*am + 2*k*vf)/(2*am*k*k);
			double coE4 = (-2*vf)/k;
			double coE5 = 1/(24*am*k*k)*(12*k*(v0 + vf)*(am*am - k*v0 + k*vf)) - dth;
			SolveN(coE1,coE2,coE3,coE4,coE5,a60);
			for (int i=0;i<4;i++)
			{
				if(a60[i].imag()==0&&a60[i].real()>=-am&&a60[i].real()<=0)
				{
					a6=a60[i].real();
					break;
				}
			}
			tmin=pow(-a6 + am,2)/(am*k) + tinc2;
			//////////////////////////////////////////////////////////////////////////
		}
		else
		{
			complex<double> a20[4];
			double coE1 = 1/(2*am*k*k);
			double coE2 = (1/k/k);
			double coE3 = (am*am + 2*k*v0)/(2*am*k*k);
			double coE4 = (2*v0)/k;
			double coE5 = 1/(24*am*k*k)*(12*k*(v0 + vf)*(am*am +k*v0 -k*vf)) - dth;
			SolveN(coE1,coE2,coE3,coE4,coE5,a20);
			//////////////////////////////////////////////////////////////////////////
			for (int i=0;i<4;i++)
			{
				if (a20[i].imag()==0&&a20[i].real()>=0&&a20[i].real()<=am)
				{
					a2=a20[i].real();
					break;
				}
			}
			tmin=pow(a2 + am,2)/(am*k) - tinc2;
		}
	}
	else
	{
		complex<double> tmin0[4];

		double coE1 = k/32;
		double coE2 = 0;
		double coE3 =  aainc/(2 *k) + v0;
		double coE4 =-dth;
		double coE5 =  -(aainc*aainc/(2*k*k*k));
		SolveN(coE1,coE2,coE3,coE4,coE5,tmin0);
		double eps=1e-15;
		for (int i=0;i<4;i++)
		{
			if (tmin0[i].imag()==0&&tmin0[i].real()>=tup4-eps&&tmin0[i].real()<=tup3+eps)
			{
				tmin=tmin0[i].real();
				break;
			}
		}
	}

}

void P2PTrajectoryPlan::Cal_Negative_Tmin(double &dth,double &th20,double &th30,double &tinc2,double &am,double &k,double &v0,
	double &vf,double &a6,double &a2,double &tmin,double &tup3,double &tup4,double &aainc)
{
	if (dth > th30 || dth < th20)
	{
		if (tinc2 > 0)
		{
			complex<double> a60[4];
			double coE1 = 1/(2*am*k*k);
			double coE2 = -(1/k/k);
			double coE3 = (am*am + 2*k*v0)/(2*am*k*k);
			double coE4 = (2*v0)/k;
			double coE5 = 1/(24*am*k*k)*(-12*k*(v0 + vf)*(am*am - k*v0 + k*vf)) - dth;
			//////////////////////////////////////////////////////////////////////////
			SolveN(coE1,coE2,coE3,coE4,coE5,a60);
			for (int i=0;i<4;i++)
			{
				if (a60[i].imag()==0&&a60[i].real()>=-am&&a60[i].real()<=0)
				{
					a6=a60[i].real();
					break;
				}
			}
			tmin=pow(-a6 + am,2)/(am*k) + tinc2;
		}
		else
		{
			complex<double> a20[4];
			double coE1 = 1/(2*am*k*k);
			double coE2 = (1/k/k);
			double coE3 = (am*am - 2*k*vf)/(2*am*k*k);
			double coE4 = (-2*vf)/k;
			double coE5 = 1/(24*am*k*k)*(-12*k*(v0 + vf)*(am*am +k*v0 -k*vf)) - dth;
			SolveN(coE1,coE2,coE3,coE4,coE5,a20);
			for (int i=0;i<4;i++)
			{
				if (a20[i].imag()==0&&a20[i].real()>=0&&a20[i].real()<=am)
				{
					a2=a20[i].real();
					break;
				}
			}
			tmin=pow(a2 + am,2)/(am*k) - tinc2;
		}

	}
	else
	{
		complex<double> tmin0[4];
		double coE1 = k/32;
		double coE2 = 0;
		double coE3 =  aainc/(2 *k) - vf;
		double coE4 =-dth;
		double coE5 =  -(aainc*aainc/(2*k*k*k));
		SolveN(coE1,coE2,coE3,coE4,coE5,tmin0);
		double eps=1e-15;
		for (int i=0;i<4;i++)
		{
			if (tmin0[i].imag()==0&&tmin0[i].real()>=tup4-eps&&tmin0[i].real()<=tup3+eps)
			{
				//				tmin=tmin0[i].real();
				tmin=tmin0[i].real();
				break;
			}
		}
	}
}
