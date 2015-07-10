/*
 * EMGTrajectoryPlan.cpp
 *
 *  Created on: 2014-11-21
 *      Author: alvin
 */

#include "EMGTrajectoryPlan.h"

EMGTrajectoryPlan::EMGTrajectoryPlan() {
	// TODO Auto-generated constructor stub
	sonar_dis[0]=sonar_dis[1]=sonar_dis[2]=2.0f;
	acc_wheel[0]=acc_wheel[1]=0;
	vel_wheel[0]=vel_wheel[1]=0;
	moving_flag=ZERO;
}

EMGTrajectoryPlan::~EMGTrajectoryPlan() {
	// TODO Auto-generated destructor stub

}
void EMGTrajectoryPlan::AddTask(int taskid,double tra_info[]){//tra_info :0-sonar1,1-sonar2,2-sonar8,3-wheel0,4-wheel1
	for(int i=0;i<3;i++){
		this->sonar_dis[i]=tra_info[i];
	}
	hsrlog.Print(_Debug,"sonar distance:%4f  %4f  %4f\n",this->sonar_dis[0],this->sonar_dis[1],this->sonar_dis[2]);
	if(taskid==0){//首次进入应急模式
		this->vel_wheel[0]=tra_info[3];
		this->vel_wheel[1]=tra_info[4];
	}
	//计算应急控制方向
	double cur_vel=this->vel_wheel[0]+this->vel_wheel[1];
	if(cur_vel<0.01&& cur_vel>-0.01)
	{
		this->moving_flag=ZERO;
		this->acc_wheel[0]=this->acc_wheel[1]=0.0002;
		hsrlog.Print(_Debug,"Direction Slow Down\n");
	}else if(this->sonar_dis[0]>SONAR_DIS&&this->sonar_dis[1]>SONAR_DIS&&this->sonar_dis[2]>SONAR_DIS){
		//无警报情况
		this->moving_flag=ZERO;
		this->acc_wheel[0]=this->acc_wheel[1]=0.0005;
		hsrlog.Print(_Debug,"Direction Slow Down\n");
	}else if(this->sonar_dis[2]>SONAR_DIS&&this->sonar_dis[1]<SONAR_DIS){
		//向左
		this->moving_flag=TURNLEFT;
		this->acc_wheel[0]=this->acc_wheel[1]=0.0005;
		hsrlog.Print(_Debug,"Direction Turn Left\n");
	}else if(this->sonar_dis[2]<SONAR_DIS&&this->sonar_dis[1]>SONAR_DIS){
		//向右
		this->moving_flag=TURNRIGHT;
		this->acc_wheel[0]=this->acc_wheel[1]=0.0005;
		hsrlog.Print(_Debug,"Direction Turn Right\n");
	}else{
		//急停
		this->moving_flag=GOBACK;
		this->acc_wheel[0]=this->acc_wheel[1]=0.0008;
		hsrlog.Print(_Debug,"Direction Quick Stop \n");
	}
}
void EMGTrajectoryPlan::CleanTask(){
	sonar_dis[0]=sonar_dis[1]=sonar_dis[2]=2.0f;
	acc_wheel[0]=acc_wheel[1]=0;
	vel_wheel[0]=vel_wheel[1]=0;
}
void EMGTrajectoryPlan::GetPeriodRef(int & flag,double ref_value[],double cur_info[]){
	double MAX_VELOCITY=app_setting.get_setting_param().car_max_vel;
	switch(this->moving_flag){
	case ZERO:
	{
		for(int i=0;i<2;i++){
			if((this->vel_wheel[i]-this->acc_wheel[i])>0){
				this->vel_wheel[i]-=this->acc_wheel[i];
			}else{
				if((this->vel_wheel[i]+this->acc_wheel[i])<0){
					this->vel_wheel[i]+=this->acc_wheel[i];
				}else{
					this->vel_wheel[i]=0;
				}
			}
		}
	}
	break;
	case TURNLEFT:
	{
		this->vel_wheel[0]-=this->acc_wheel[0];
		this->vel_wheel[1]+=this->acc_wheel[1];
	}
	break;
	case TURNRIGHT:
	{
		this->vel_wheel[0]+=this->acc_wheel[0];
		this->vel_wheel[1]-=this->acc_wheel[1];
	}
	break;
	case GOBACK:
	{
		this->vel_wheel[0]-=this->acc_wheel[0];
		this->vel_wheel[1]-=this->acc_wheel[1];
	}
	}
	for(int i=0;i<2;i++){
		if(this->vel_wheel[i]>=MAX_VELOCITY){
			this->vel_wheel[i]=MAX_VELOCITY;
			}else if(this->vel_wheel[i]<=-MAX_VELOCITY){
					this->vel_wheel[i]=-MAX_VELOCITY;
			}
		ref_value[i]=this->vel_wheel[i];
	}
}
