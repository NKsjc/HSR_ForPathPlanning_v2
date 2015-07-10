/*
 * MainProcess.cpp
 *
 *  Created on: 2014-9-6
 *      Author: alvin
 */
#include "MainProcess.h"
#include <string.h>
#define   LOG(format, ...) {message_info temp_message;sprintf (temp_message.printbuff,""format"\n",##__VA_ARGS__);\
this->main_ipc.Push_New_Message(temp_message);}

App_Setting app_setting;
LogFactory hsrlog(app_setting.get_setting_param().log_level,app_setting.get_setting_param().log_path);

double bezierpoint[13]={0,0,0,1,1.49,1.49,2.49,3,1,2.1,3.1,3.11,3};
int beziernumber=5;
MainProcess::MainProcess() {
	// TODO Auto-generated constructor stub

	mlockall(MCL_CURRENT | MCL_FUTURE);
	//creat the realtime task for recv data
	rt_task_create(&Hand_Recv_Task, "Hand_Recv_Task", 0, 80, T_JOINABLE);
	rt_task_create(&Main_Task, "Main_Task", 0, 80,T_JOINABLE);
	rt_task_create(&Car_Recv_Task, "Car_Recv_Task", 0, 80,T_JOINABLE);


	memset(&main_status,0,sizeof(main_status));
	memset(&main_contrl,0,sizeof(main_contrl));
	main_status.current_xyzw[0]=1;
	main_status.current_xyzw[1]=1;
	main_status.current_xyzw[3]=0.0384;
	main_set_in=main_status;
	serve_status=0;
	init_status=0;
	softhand_key_tra=new KeyTrajectoryPlan(SOFTHAND);
	softhand_p2p_tra=new P2PTrajectoryPlan();

	rigidhand_p2p_tra=new P2PTrajectoryPlan();
	rigidhand_key_tra=new KeyTrajectoryPlan(RIGIDHAND);

	car_incricle_tra=new InCircleTrajectoryPlan();
	car_key_tra=new KeyTrajectoryPlan(CAR);
	car_emg_tra=new EMGTrajectoryPlan();
	car_bezier_tra=new BezierPlan();


	car_control=new CarControl(app_setting.get_setting_param().channel_car==string("REAL")?REAL:VIRTUAL);
//	pvu_control=new PVUControl(app_setting.get_setting_param().channel_car==string("REAL")?REAL:VIRTUAL);
	pvu_control=new PVUControl(VIRTUAL);

	rigidhand_control=new RigidHandControl(app_setting.get_setting_param().channel_rigidhand==string("REAL")?REAL:VIRTUAL);
	softhand_control=new SoftHandControl(app_setting.get_setting_param().channel_softhand==string("REAL")?REAL:VIRTUAL);
}

MainProcess::~MainProcess() {
	// TODO Auto-generated destructor stub
	delete softhand_key_tra;
	delete softhand_p2p_tra;
	delete rigidhand_p2p_tra;
	delete rigidhand_key_tra;
	delete car_incricle_tra;
	delete car_key_tra;
	delete car_emg_tra;
	delete car_bezier_tra;


	delete car_control;
	delete rigidhand_control;
	delete softhand_control;
	delete pvu_control;
	this->Signal_Handler(1);
}
void MainProcess::Period_Ctrl_Func(){//周期性控制函数



	this->Period_Update_Info();
	this->Period_Set_Ref();
	this->main_ipc.Pipe_Recv_Info();
	this->main_ipc.Pipe_Send_Info();
	this->Period_Process_Task();
	switch(this->main_contrl.contrl_style){
	case KEYBOARD_CONTRL:
	{
		int getflag=0;
		this->car_key_tra->GetPeriodRef(getflag,this->main_contrl.car_set_ref,this->main_status.velocity);
		this->rigidhand_key_tra->GetPeriodRef(getflag,this->main_contrl.right_hand_set_ref,this->main_status.current_rst);
		this->softhand_key_tra->GetPeriodRef(getflag,this->main_contrl.left_hand_set_ref,this->main_status.current_lst);
	}
	break;
	case TRAJECTORY_CONTRL:
	{
		int getflag=0;
//		std::cout<<this->main_contrl.right_hand_set_ref[0]<<" "<<this->main_contrl.right_hand_set_ref[1]<<endl;
		double current_info[5];
		current_info[0]=this->main_status.current_xyzw[0];
		current_info[1]=this->main_status.current_xyzw[1];
		current_info[2]=this->main_status.current_xyzw[3];
		current_info[3]=this->main_status.velocity[0];
		current_info[4]=this->main_status.velocity[1];

		this->car_incricle_tra->GetPeriodRef(getflag,this->main_contrl.car_set_ref,current_info);
		this->rigidhand_p2p_tra->GetPeriodRef(getflag,this->main_contrl.right_hand_set_ref,this->main_status.current_rst);
		this->softhand_p2p_tra->GetPeriodRef(getflag,this->main_contrl.left_hand_set_ref,this->main_status.current_lst);

//		hsrlog.Print(_Save,"%4f  %4f  %4f  %4f  %4f  %4f  %4f  %4f\n",this->main_contrl.right_hand_set_ref[0],this->main_contrl.right_hand_set_ref[1],this->main_contrl.right_hand_set_ref[2],this->main_contrl.right_hand_set_ref[3],
//				this->main_status.current_rst[0],this->main_status.current_rst[1],this->main_status.current_rst[2],this->main_status.current_rst[3]);
//		hsrlog.Print(_Save,"%4f  %4f   %4f   %4f   %4f\n",this->main_status.current_xyzw[0],this->main_status.current_xyzw[1],
//				this->main_status.current_xyzw[3],this->main_status.velocity[0],this->main_status.velocity[1]);
	}
	break;
	case ETA_TRA_CONTRL:
	{
		int getflag=0;
		double wheel[2];
		this->car_bezier_tra->GetPeriodRef(getflag,wheel,NULL);
		this->main_contrl.car_set_ref[0]=wheel[1];
		this->main_contrl.car_set_ref[1]=wheel[0];

	}
	break;
	case EMERGENCY_CONTRL:
	{
		int getflag=0;
		double current_info[5];
		current_info[0]=this->main_status.current_xyzw[0];
		current_info[1]=this->main_status.current_xyzw[1];
		current_info[2]=this->main_status.current_xyzw[3];
		current_info[3]=this->main_status.velocity[0];
		current_info[4]=this->main_status.velocity[1];
		this->car_emg_tra->GetPeriodRef(getflag,this->main_contrl.car_set_ref,current_info);
	}
	}


	main_set_in.current_xyzw[3]+=PERIOD_CONTRL*0.001*(this->main_contrl.car_set_ref[1]-this->main_contrl.car_set_ref[0])*r/Le/2;//st
	if(main_set_in.current_xyzw[3]>=pi)main_set_in.current_xyzw[3]-=2*pi;
	if(main_set_in.current_xyzw[3]<=-pi)main_set_in.current_xyzw[3]+=2*pi;
	main_set_in.current_xyzw[0]+=PERIOD_CONTRL*0.001*cos(main_set_in.current_xyzw[3])*(this->main_contrl.car_set_ref[0]+this->main_contrl.car_set_ref[1])*r/2;//x
	main_set_in.current_xyzw[1]+=PERIOD_CONTRL*0.001*sin(main_set_in.current_xyzw[3])*(this->main_contrl.car_set_ref[0]+this->main_contrl.car_set_ref[1])*r/2;//y
	main_set_in.current_xyzw[2]=0;//z

}

void MainProcess::Period_Process_Task(){//周期性处理任务函数

	action_info new_action=this->main_ipc.Pop_New_Task();
	if(SONAR_INFO==new_action.action_id)
		return;
	switch(new_action.action_id){
	case HALT:
		this->car_key_tra->CleanTask();
		this->car_bezier_tra->CleanTask();
		this->car_emg_tra->CleanTask();
		this->car_incricle_tra->CleanTask();

		this->rigidhand_key_tra->CleanTask();
		this->softhand_key_tra->CleanTask();

		this->rigidhand_p2p_tra->CleanTask();
		this->softhand_p2p_tra->CleanTask();

		app_setting.Refresh_Param();
	break;
	case ENTER_LINE://如果当前模式是轨迹规划模式则停止，进入待机模式，否则则进入规划模庿
		this->main_contrl.contrl_style=TRAJECTORY_CONTRL;
		this->car_key_tra->CleanTask();
		this->car_bezier_tra->CleanTask();
		this->car_emg_tra->CleanTask();

		this->rigidhand_key_tra->CleanTask();
		this->softhand_key_tra->CleanTask();
	break;
	case ENTER_KEY://如果当前没有控制则进入键盘模式，清除加速度和计时，如果是键盘模式则清除所有值进入待机模庿
		this->main_contrl.contrl_style=KEYBOARD_CONTRL;
		this->car_bezier_tra->CleanTask();
		this->car_incricle_tra->CleanTask();
		this->car_emg_tra->CleanTask();

		this->rigidhand_p2p_tra->CleanTask();
		this->softhand_p2p_tra->CleanTask();
	break;
	case ENTER_ETA://如果当前没有控制则进入ETA模式，清除加速度和计时，如果是键盘模式则清除所有值进入待机模庿
	{
		this->car_incricle_tra->CleanTask();
		this->rigidhand_p2p_tra->CleanTask();
		this->softhand_p2p_tra->CleanTask();
		this->car_key_tra->CleanTask();
		this->car_emg_tra->CleanTask();
		this->rigidhand_key_tra->CleanTask();
		this->softhand_key_tra->CleanTask();
		this->car_bezier_tra->AddTask(beziernumber,bezierpoint);
		hsrlog.Print(_Debug,"hello finish tra");
		this->main_contrl.contrl_style=ETA_TRA_CONTRL;
	}
	break;
	case SONAR_INFO:
	{
		if(new_action.sonar_info[0]>0){//开始预警
			double tra_info[5];
			for(int i=0;i<3;i++){
				tra_info[i]=new_action.sonar_info[i+1];
			}
			tra_info[3]=this->main_status.velocity[0];
			tra_info[4]=this->main_status.velocity[1];

			if(this->main_contrl.contrl_style!=EMERGENCY_CONTRL){
				hsrlog.Print(_Debug,"enter sonar emg contrl\n");
				this->bak_contrlmode=this->main_contrl.contrl_style;
				this->car_key_tra->CleanTask();
				this->car_bezier_tra->CleanTask();
				this->car_incricle_tra->CleanTask();
				this->main_contrl.contrl_style=EMERGENCY_CONTRL;
				this->car_emg_tra->AddTask(0,tra_info);//首次进入应急模式
				
			}
			else{
				this->car_emg_tra->AddTask(1,tra_info);
			}
//			hsrlog.Print(_Debug,"recv sonar info :%4f  %4f  %4f  %4f\n",new_action.sonar_info[0],new_action.sonar_info[1],new_action.sonar_info[2],new_action.sonar_info[3]);

		}
		else{
			if(this->main_contrl.contrl_style==EMERGENCY_CONTRL){
				//退出应急控制,此时等待速度衰减为0 再退出应急模式
				if(fabs(this->main_status.velocity[0])<0.005&&fabs(this->main_status.velocity[1])<0.005){
					this->car_emg_tra->CleanTask();
					this->main_contrl.contrl_style=this->bak_contrlmode;
					hsrlog.Print(_Debug,"remove from sonar emg contrl\n");
				}
				else{
					hsrlog.Print(_Debug,"waiting for remove from sonar emg contrl\n");
					double tra_info[5];
					for(int i=0;i<3;i++){
						tra_info[i]=new_action.sonar_info[i+1];
					}
					tra_info[3]=this->main_status.velocity[0];
					tra_info[4]=this->main_status.velocity[1];
					this->car_emg_tra->AddTask(1,tra_info);//taskid=1代表等待延时
					
				}
			}
		}
	}
	break;
	case SERVE_SWITCH:
		 if(serve_status==0){
			 this->rigidhand_control->InitModule(SERVEON);
			 this->softhand_control->InitModule(SERVEON);
			 serve_status=1;
		 }else {
			 this->rigidhand_control->InitModule(SERVEOFF);
			 this->softhand_control->InitModule(SERVEOFF);
			 serve_status=0;
		 }
	break;
	case CLEAR_ENC:
		/*main_hand.Hand_Clear_Position();*/
//		this->rigidhand_control->InitModule(CLEARENCODER);
			this->softhand_control->InitModule(CLEARENCODER);
	break;

	case HOME:
	{
		this->rigidhand_p2p_tra->AddTask(0,const_cast <double *>(app_setting.get_setting_param().rigidhand_ready_st));
		this->softhand_p2p_tra->AddTask(0,const_cast <double *>(app_setting.get_setting_param().softhand_ready_st));
	}
	break;
	case NEW_DCODE:
		this->Update_Status_By_DCode(new_action);//有新的二维码数据
	break;
	case ENTER_HOME:
		switch(this->init_status){
		case 0://auto home
	//		this->rigidhand_control->InitModule(AUTOHOME);
			this->init_status=1;
			
//		break;
		case 1://go home offset
		{
//			double tempe[4];
//			tempe[0]=0.345;
//			tempe[1]=-0.15;
//			tempe[2]=-0.32;
//			tempe[3]=1.325;
/*			tempe[0]=0.345;
			tempe[1]=-0.15;
			tempe[2]=-0.28;
			tempe[3]=1.485;
			this->rigidhand_p2p_tra->AddTask(0,tempe);
*/
			this->rigidhand_control->InitModule(CLEARENCODER);
			action_info tempaction;
			tempaction.action_id=ENTER_KEY;
			this->main_ipc.Insert_New_Task(tempaction);
			this->init_status=2;
		}
		break;
		case 2://set offset
			this->rigidhand_control->InitModule(HOMEOFFSET);
			this->init_status=3;
		break;
		case 3://reset home offset
		{
			double tempe[4];
			tempe[0]=app_setting.get_setting_param().rigidhand_reset_st[0];
			tempe[1]=app_setting.get_setting_param().rigidhand_reset_st[1];
			tempe[2]=app_setting.get_setting_param().rigidhand_reset_st[2];
			tempe[3]=app_setting.get_setting_param().rigidhand_reset_st[3];
			this->rigidhand_p2p_tra->AddTask(0,tempe);
			
			tempe[0]=app_setting.get_setting_param().softhand_reset_st[0];
			tempe[1]=app_setting.get_setting_param().softhand_reset_st[1];
			tempe[2]=app_setting.get_setting_param().softhand_reset_st[2];
			tempe[3]=app_setting.get_setting_param().softhand_reset_st[3];
			this->softhand_p2p_tra->AddTask(0,tempe);
			
			
			action_info tempaction;
			tempaction.action_id=ENTER_LINE;
			this->main_ipc.Insert_New_Task(tempaction);
		}
		}
	break;
	case SET_POINT:
	{
		double xyw[3];
		xyw[0]=new_action.target_car[0];xyw[1]=new_action.target_car[1];xyw[2]=new_action.target_car[3];
		this->car_incricle_tra->AddTask(0,xyw);
	}
	break;
	case NEW_TARGET:
		if(2==new_action.hand_num){
			this->rigidhand_p2p_tra->AddTask(0,new_action.target_hand_st);
		}
		if(1==new_action.hand_num){
			hsrlog.Print(_Debug,"softhand recv target\n");
			this->softhand_p2p_tra->AddTask(0,new_action.target_hand_st);
		}
	break;
	case LEFT:
		if(this->main_contrl.contrl_style==KEYBOARD_CONTRL)
		{
			this->car_key_tra->AddTask(TURNLEFT,NULL);
		}
	break;
	case BACK:
		if(this->main_contrl.contrl_style==KEYBOARD_CONTRL)
		{
			this->car_key_tra->AddTask(GOBACK,NULL);
		}
	break;
	case RIGHT:
		if(this->main_contrl.contrl_style==KEYBOARD_CONTRL)
		{
			this->car_key_tra->AddTask(TURNRIGHT,NULL);
		}
	break;
	case GO:
		if(this->main_contrl.contrl_style==KEYBOARD_CONTRL)
		{
			this->car_key_tra->AddTask(GOAHEAD,NULL);
		}
	break;
	case STOP:
		if(this->main_contrl.contrl_style==KEYBOARD_CONTRL)
		{
			this->car_key_tra->AddTask(ZERO,NULL);
		}
	break;
	case CHANGE_JOINT:
		if(this->main_contrl.contrl_style==KEYBOARD_CONTRL)
		{
			if(new_action.hand_num==2)
			{
				this->rigidhand_key_tra->AddTask(new_action.joint_num-1,NULL);
				this->softhand_key_tra->CleanTask();
			}
			if(new_action.hand_num==1)
			{
				this->softhand_key_tra->AddTask(new_action.joint_num-1,NULL);
				this->rigidhand_key_tra->CleanTask();
			}
		}
	break;
	case JOINT_POSITIVE_MOVE:
		if(this->main_contrl.contrl_style==KEYBOARD_CONTRL)
		{
			this->rigidhand_key_tra->AddTask(POSITIVE+3,NULL);//3 is offset of cmd
			this->softhand_key_tra->AddTask(POSITIVE+3,NULL);
		}
	break;
	case JOINT_NEGTIVE_MOVE:
		if(this->main_contrl.contrl_style==KEYBOARD_CONTRL)
		{
			this->rigidhand_key_tra->AddTask(NEGTIVE+3,NULL);
			this->softhand_key_tra->AddTask(NEGTIVE+3,NULL);
		}
	break;
#if 0
	case CHANGE_LEVEL:
		app_setting.Set_Vel_Level(new_action.level);
	break;
#endif
	}

	if(READY!=new_action.action_id){
		this->Period_Process_Task();
	}



}
void MainProcess::Period_Set_Ref(){
	this->car_control->SetReference(0,main_contrl.car_set_ref);
//	std::cout<<main_contrl.car_set_ref[0]<<"  "<<main_contrl.car_set_ref[1]<<endl;

	if(this->init_status>=2&&this->serve_status==1){
		this->rigidhand_control->SetReference(0,this->main_contrl.right_hand_set_ref);
	}else{
		this->rigidhand_control->SetReference(0,NULL);
	}
	if(this->serve_status==1){
		this->softhand_control->SetReference(0,this->main_contrl.left_hand_set_ref);
	}else{
		this->softhand_control->SetReference(0,NULL);
	}
//	this->softhand_control->SetReference(0,this->main_contrl.left_hand_set_ref);
//	hsrlog.Print(_Debug,"current set :%4f  %4f   %4f  %4f\n",this->main_contrl.left_hand_set_ref[0],this->main_contrl.left_hand_set_ref[1],
//	this->main_contrl.left_hand_set_ref[2],this->main_contrl.left_hand_set_ref[3]);
}

current_status & MainProcess::Get_Current_Status()//向外界传递当前状徿
{
	return this->main_status;
}
void MainProcess::Period_Update_Info()//周期性刷新系统状徿
{
	int feedbackflag=0;
	double pvu[4];
	this->pvu_control->GetCurrent(feedbackflag,pvu);
	this->car_control->GetCurrent(feedbackflag,this->main_status.velocity);

	main_status.current_xyzw[3]+=PERIOD_CONTRL*0.001*(main_status.velocity[1]-main_status.velocity[0])*r/Le/2;//st
	if(main_status.current_xyzw[3]>=pi)main_status.current_xyzw[3]-=2*pi;
	if(main_status.current_xyzw[3]<=-pi)main_status.current_xyzw[3]+=2*pi;
	main_status.current_xyzw[0]+=PERIOD_CONTRL*0.001*cos(main_status.current_xyzw[3])*(main_status.velocity[0]+main_status.velocity[1])*r/2;//x
	main_status.current_xyzw[1]+=PERIOD_CONTRL*0.001*sin(main_status.current_xyzw[3])*(main_status.velocity[0]+main_status.velocity[1])*r/2;//y
	main_status.current_xyzw[2]=0;//z
	if(dcode_list.size()>=500){
		dcode_list.pop_front();
	}
	log_temp_status new_ve;
	new_ve.v_left=main_status.velocity[0];
	new_ve.v_right=main_status.velocity[1];
	dcode_list.push_back(new_ve);

	this->rigidhand_control->GetCurrent(feedbackflag,this->main_status.current_rst);
	this->softhand_control->GetCurrent(feedbackflag,this->main_status.current_lst);


//	hsrlog.Print(_Save,"%4f  %4f  %4f  %4f  %4f  %4f\n",
//			main_set_in.current_xyzw[0],main_set_in.current_xyzw[1],main_set_in.current_xyzw[2],
//			main_status.current_xyzw[0],main_status.current_xyzw[1],main_status.current_xyzw[2]);

	LOG("%4f   %4f   %4f  %4f  %4f  %4f   %4f	%4f   %4f    %4f   %4f   %4f    %4f\n",\
	this->main_status.current_xyzw[0],this->main_status.current_xyzw[1],this->main_status.current_xyzw[3],this->main_status.velocity[0],this->main_status.velocity[1],\
	this->main_status.current_rst[0],this->main_status.current_rst[1],this->main_status.current_rst[2],this->main_status.current_rst[3],\
	this->main_status.current_lst[0],this->main_status.current_lst[1],this->main_status.current_lst[2],this->main_status.current_lst[3]\	
	);

//	LOG("%4f  %4f   %4f   %4f   %4f  %4f  %4f   %4f   %4f\n",this->main_status.current_xyzw[0],this->main_status.current_xyzw[1],this->main_status.current_xyzw[3],this->main_status.velocity[0],this->main_status.velocity[1],
//	pvu[0],pvu[1],pvu[2],pvu[3]);

	if(this->main_contrl.contrl_style==ETA_TRA_CONTRL)
		hsrlog.Print(_Save,"%4f  %4f   %4f   %4f   %4f\n",this->main_status.current_xyzw[0],this->main_status.current_xyzw[1],this->main_status.current_xyzw[3],this->main_status.velocity[0],this->main_status.velocity[1]);
}
void MainProcess::Update_Status_By_DCode(action_info &new_action)
{
	current_status temp_status;
	unsigned int delta_t=new_action.dead_time;
	if(delta_t>=dcode_list.size()||dcode_list.size()==0) return;
	//读取二维码给出的位姿
	temp_status.current_xyzw[3]=new_action.target_car[3];//st
	temp_status.current_xyzw[0]=new_action.target_car[0];//x
	temp_status.current_xyzw[1]=new_action.target_car[1];//y
	list<log_temp_status>::iterator pointer=dcode_list.end();
	//向前倒退二维码时刻的速度便
	for(int i=0;i<new_action.dead_time-10&&pointer!=dcode_list.begin();i++)
	{
		pointer--;
	}

	//累加速度的积分得到实际当前时刻的位姿
	while(pointer!=dcode_list.end())
	{
		temp_status.velocity[0]=(*pointer).v_left;
		temp_status.velocity[1]=(*pointer).v_right;
		temp_status.current_xyzw[3]+=PERIOD_CONTRL*0.001*(temp_status.velocity[1]-temp_status.velocity[0])*r/Le/2;//st
		if(temp_status.current_xyzw[3]>=pi)temp_status.current_xyzw[3]-=2*pi;
		if(temp_status.current_xyzw[3]<=-pi)temp_status.current_xyzw[3]+=2*pi;
		temp_status.current_xyzw[0]+=PERIOD_CONTRL*0.001*cos(temp_status.current_xyzw[3])*(temp_status.velocity[0]+temp_status.velocity[1])*r/2;//x
		temp_status.current_xyzw[1]+=PERIOD_CONTRL*0.001*sin(temp_status.current_xyzw[3])*(temp_status.velocity[0]+temp_status.velocity[1])*r/2;//y
		pointer++;
	}
	main_status=temp_status;
}
void signal_exit(int signal)
{
	exit(0);
}
int main()//1--0主函数
{
	
	MainProcess HSR_Project;
	signal(SIGINT, signal_exit); 
	HSR_Project.Start_Loop();
	return 0;
}
void MainProcess::Car_Recv_Task_CallBack(void *param){
	MainProcess* this_obj = static_cast<MainProcess*>(param);
	if(VIRTUAL==this_obj->car_control->GetVirtualFlag()&&VIRTUAL==this_obj->pvu_control->GetVirtualFlag())
	{
		return ;
	}
	while(1)
	{
		if(REAL==this_obj->car_control->GetVirtualFlag()){
			this_obj->car_control->RecvFeedback();
		}
		if(REAL==this_obj->pvu_control->GetVirtualFlag()){
			this_obj->pvu_control->RecvFeedback();
		}
		SLEEP(PERIOD_FEEDBACK*0.2);
	}
}
void MainProcess::Main_Task_Callback(void *param)
{

	MainProcess* this_obj = static_cast<MainProcess*>(param);
	this_obj->car_control->InitModule(INITMOTOR);
	this_obj->rigidhand_control->InitModule(INITMOTOR);
	this_obj->softhand_control->InitModule(INITMOTOR);

	rt_task_set_periodic(NULL, TM_NOW,PERIOD_CONTRL*1000*1000);//ns ,ms
	int count_period=0;
	while(1)
	{
		rt_task_wait_period(NULL);
		count_period++;
		for(int i=0;i<5;i++)
		{
			if(VIRTUAL==this_obj->car_control->GetVirtualFlag()){
				this_obj->car_control->RecvFeedback();
			}
			if(VIRTUAL==this_obj->rigidhand_control->GetVirtualFlag()){
				this_obj->rigidhand_control->RecvFeedback();
			}
			if(VIRTUAL==this_obj->softhand_control->GetVirtualFlag()){
				this_obj->softhand_control->RecvFeedback();
			}
		}
		this_obj->Period_Ctrl_Func();
		//每隔50ms上传实时状态
		if(50/PERIOD_CONTRL==count_period)
		{
			count_period=0;
			this_obj->main_ipc.Push_New_Status(this_obj->Get_Current_Status());
		}

	}
}
void MainProcess::Hand_Recv_Task_CallBack(void *param)
{

	MainProcess* this_obj = static_cast<MainProcess*>(param);
	VirtualFlag rigid,soft;
	rigid=this_obj->rigidhand_control->GetVirtualFlag();
	soft=this_obj->softhand_control->GetVirtualFlag();
	if(VIRTUAL==rigid&&VIRTUAL==soft){
		return;
	}
	while(1)
	{
		if(REAL==rigid){
			this_obj->rigidhand_control->RecvFeedback();
		}
		if(REAL==soft){
			this_obj->softhand_control->RecvFeedback();
		}
		SLEEP(PERIOD_FEEDBACK*0.1);
	}
}

void MainProcess::Start_Loop()//程序运行
{
	rt_task_start(&Main_Task, Main_Task_Callback, this);

	rt_task_start(&Hand_Recv_Task, Hand_Recv_Task_CallBack, this);

	rt_task_start(&Car_Recv_Task, Car_Recv_Task_CallBack, this);

	rt_task_join(&Main_Task);
	rt_task_join(&Hand_Recv_Task);
	rt_task_join(&Car_Recv_Task);

}
void MainProcess::Signal_Handler(int status)//2--9信号处理函数
{
	this->rigidhand_control->InitModule(SERVEOFF);
	this->softhand_control->InitModule(SERVEOFF);
	rt_task_delete(&Main_Task);
	rt_task_delete(&Hand_Recv_Task);
	rt_task_delete(&Car_Recv_Task);
	exit(0);
}
