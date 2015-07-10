/*
 * DataProtocol.h
 *
 *  Created on: 2014-9-6
 *      Author: alvin
 */

#ifndef DATAPROTOCOL_H_
#define DATAPROTOCOL_H_
//机器人本体内部协议
/*-------------------------------------------------------------------------------------------------------------------------------------------------*/
typedef struct _action_info//1---动作级指令，用于内部RT-NRT-PC2交互
{
	//动作编号，供程序逻辑使用
	int action_id;
	//手臂的目标状态
//	double target_hand[4][4];
	//手臂目标关节角
	double target_hand_st[4];
	//小车需要实现的目标状态,矩阵
	double target_car[4];
	//执行任务手臂编号
	int hand_num;
	//执行任务手臂关节编号
	int joint_num;
	//动作完成时间
	int dead_time;//毫秒,或者附加参数
	//附加信息，对速度要求,分别是，min，max，normal
//	double speed_cmd[3];
	double sonar_info[4];
	int level;
}action_info;

typedef struct _log_temp_status
{
	double v_left,v_right;
}log_temp_status;
typedef struct _message_info//2---message结构体用于RT-NRT传递消息，其中包括打印消息，以及控制命令消息
{
	//打印字符串，供写入日志系统
	char printbuff[300];
	//实时进程可能传递的状态信息，例如实时进程已经崩溃或结束
	int status;
	//本体当前的所有状态信息
}message_info;


typedef struct _contrl_param//contrl_param结构体用于程序逻辑控制
{

	//程序逻辑控制
	int key_end;//控制运行逻辑的变量,各个线程循环结束的标志
	int release_contrl;//控制等待程序完全关闭下位机使能后才能退出主程序

	double car_set_ref[2];//最终控制小车和手臂发数的数据
	double right_hand_set_ref[4];//最终控制小车和手臂发数的数据
	double left_hand_set_ref[4];//最终控制小车和手臂发数的数据
	int contrl_style;//标记当前的规划结果来源于键盘控制还是轨迹规划

}contrl_param;

typedef struct _current_status//3---current_status结构体用于RT-NRT传递当前本体的状态
{
	//小车的当前状态单位为m，弧度
	double current_xyzw[4];//直接来自下位机发送的数据
	//左右轮子的速度
	double velocity[2];
	//手臂的当前状态,弧度,根据从驱动器获取的码盘值转换到程序值
	double current_rst[4];
	double current_right_handP[4][4];//矩阵值
	double current_lst[4];
}current_status;

#endif /* DATAPROTOCOL_H_ */
