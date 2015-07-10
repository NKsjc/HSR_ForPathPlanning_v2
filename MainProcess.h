/*
 * MainProcess.h
 *
 *  Created on: 2014-9-6
 *      Author: alvin
 */

#ifndef MAINPROCESS_H_
#define MAINPROCESS_H_
#include <signal.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>

#include "CarControl.h"
#include "RigidHandControl.h"
#include "SoftHandControl.h"
#include "PVUControl.h"

#include "KeyTrajectoryPlan.h"
#include "InCircleTrajectoryPlan.h"
#include "P2PTrajectoryPlan.h"
#include "EMGTrajectoryPlan.h"
#include "BezierTrajectory.h"

#include "IPC.h"
typedef list<log_temp_status>queue_dcode;//二维码迭代队列

class MainProcess {
private:
	current_status main_set_in;//当前的给定输入
	current_status main_status;//当前状态
	contrl_param main_contrl;//控制逻辑的参数
	queue_dcode dcode_list;
	int bak_contrlmode;
public:

	//进程分配的相关变量
	RT_TASK Hand_Recv_Task;
	RT_TASK Car_Recv_Task;
	RT_TASK Main_Task;

	int init_status;//define the status of home 0=no home, 1=auto home,2=offset home;
	int serve_status;//define the status of hand serve

	IPC main_ipc;
	TrajectoryPlan * softhand_key_tra;
	TrajectoryPlan * softhand_p2p_tra;

	TrajectoryPlan * rigidhand_p2p_tra;
	TrajectoryPlan * rigidhand_key_tra;

	TrajectoryPlan * car_incricle_tra;
	TrajectoryPlan * car_key_tra;
	TrajectoryPlan * car_emg_tra;
	TrajectoryPlan * car_bezier_tra;


	ModuleControl  * car_control;
	ModuleControl  * rigidhand_control;
	ModuleControl  * softhand_control;
	ModuleControl  * pvu_control;
public:
	MainProcess();
	virtual ~MainProcess();

	void Signal_Handler(int status);//2--9信号处理函数
	void Start_Loop();//程序运行
	static void Hand_Recv_Task_CallBack(void *);//1--1手臂接收线程的回调函数
	static void Car_Recv_Task_CallBack(void *);//1--2小车接收线程的回调函数
	static void Main_Task_Callback(void *);//1--4主控线程，用于周期控制

	current_status &Get_Current_Status();//向外界传递当前状态
	void Period_Update_Info();//周期性刷新系统状态
	void Period_Set_Ref();
	void Period_Process_Task();//周期性处理任务函数
	void Period_Ctrl_Func();//周期性控制函数
	void Update_Status_By_DCode(action_info &new_action);//2--5根据二维码更新小车的状态
};

#endif /* MAINPROCESS_H_ */
