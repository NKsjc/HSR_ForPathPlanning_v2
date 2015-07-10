/*
 * KeyTrajectoryPlan.cpp
 *
 *  Created on: 2014-9-7
 *      Author: alvin
 */

#include "KeyTrajectoryPlan.h"
#include <string.h>
KeyTrajectoryPlan::KeyTrajectoryPlan(ModuleId type) {
	// TODO Auto-generated constructor stub
	this->id=type;
	this->count_key=200;
	this->moving_joint=-1;
	this->moving_flag=ZERO;
	this->acc_wheel[0]=this->acc_wheel[1]=app_setting.get_setting_param().car_max_acc/2;
	this->step_joint[0]=this->step_joint[1]=this->step_joint[2]=this->step_joint[3]=app_setting.get_setting_param().rigidhand_key_step;
	for(int i=0;i<4;i++)ref_value[i]=0;
}

KeyTrajectoryPlan::~KeyTrajectoryPlan() {
	// TODO Auto-generated destructor stub
}
void KeyTrajectoryPlan::AddTask(int taskid,double tra_info[]){
//	std::cout<<"task id:"<<taskid<<"  ";

	switch(this->id){
	case CAR:
	{

		if(ZERO==taskid){
			this->moving_flag=ZERO;
		}else if(GOAHEAD==taskid){
			this->moving_flag=GOAHEAD;
		}else if(GOBACK==taskid){
			this->moving_flag=GOBACK;
		}else if(TURNLEFT==taskid){
			this->moving_flag=TURNLEFT;
		}else if(TURNRIGHT==taskid){
			this->moving_flag=TURNRIGHT;
		}
	}
	break;
	case RIGIDHAND:
	case SOFTHAND:
	{
		if(taskid>=0&&taskid<=3){
			this->moving_joint=taskid;
		}else if(4==taskid){
			this->moving_flag=POSITIVE;
		}else if(5==taskid){
			this->moving_flag=NEGTIVE;
		}
	}
	break;
	}
	this->count_key=0;

}
void KeyTrajectoryPlan::CleanTask(){
	memset(this->ref_value,0,sizeof(this->ref_value));
	this->moving_flag=ZERO;
	this->moving_joint=-1;
}
void KeyTrajectoryPlan::GetPeriodRef(int & returnflag,double ref_value[],double cur_info[]){
	double MAX_ACC=app_setting.get_setting_param().car_max_acc;
	double MAX_VELOCITY=app_setting.get_setting_param().car_max_vel;
	this->acc_wheel[0]=this->acc_wheel[1]=app_setting.get_setting_param().car_max_acc/2;
	this->step_joint[0]=this->step_joint[1]=this->step_joint[2]=
	this->step_joint[3]=app_setting.get_setting_param().rigidhand_key_step;
	if(this->count_key>=200){
		this->moving_flag=ZERO;
	}else{
		this->count_key++;
	}
	switch(this->id){
		case CAR:
		{
			switch(this->moving_flag){
			case ZERO:
			{
				for(int i=0;i<2;i++){
//					this->ref_value[i]=cur_info[i];//get current vel
					if((this->ref_value[i]-3*MAX_ACC)>0){
						this->ref_value[i]-=3*MAX_ACC;
					}else{
						if((this->ref_value[i]+3*MAX_ACC)<0){
							this->ref_value[i]+=3*MAX_ACC;
						}else{
							this->ref_value[i]=0;
						}
					}
					ref_value[i]=this->ref_value[i];
				}
			}
			break;
			case GOAHEAD:
			{
				this->ref_value[0]+=this->acc_wheel[0];
				this->ref_value[1]+=this->acc_wheel[1];
			}
			break;
			case GOBACK:
			{
				this->ref_value[0]+=-1*this->acc_wheel[0];
				this->ref_value[1]+=-1*this->acc_wheel[1];
			}
			break;
			case TURNLEFT:
			{
				this->ref_value[0]+=-1*this->acc_wheel[0];
				this->ref_value[1]+=this->acc_wheel[1];
			}
			break;
			case TURNRIGHT:
			{
				this->ref_value[0]+=this->acc_wheel[0];
				this->ref_value[1]+=-1*this->acc_wheel[1];
			}
			break;
			}
			for(int i=0;i<2;i++){
				if(this->ref_value[i]>=MAX_VELOCITY){
					this->ref_value[i]=MAX_VELOCITY;
					}else if(this->ref_value[i]<=-MAX_VELOCITY){
							this->ref_value[i]=-MAX_VELOCITY;
					}
				ref_value[i]=this->ref_value[i];
			}
//			hsrlog.Print(_Debug,"%d  %4f  %4f  %4f  %4f\n",\
					this->count_key,this->ref_value[0],this->ref_value[1],cur_info[0],cur_info[1]);
		}
		break;
		case RIGIDHAND:
		case SOFTHAND:
		{
			if(this->moving_joint>=0&&this->moving_joint<=3){
				switch(this->moving_flag){
					case ZERO:
					{
						memset(this->ref_value,0,sizeof(this->ref_value));
					}
					break;
					case POSITIVE:
					{
						this->ref_value[this->moving_joint]=this->step_joint[this->moving_joint];
					}
					break;
					case NEGTIVE:
					{
						this->ref_value[this->moving_joint]=-1*this->step_joint[this->moving_joint];
					}
					break;
					}
			}
			for(int i=0;i<4;i++)ref_value[i]=this->ref_value[i];
		}
		break;
	}
}
