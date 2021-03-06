/*
 * ProjectCommon.h
 *
 *  Created on: 2014-9-3
 *      Author: alvin
 */

#ifndef PROJECTCOMMON_H_
#define PROJECTCOMMON_H_

#ifndef _SOCKETBIND_
#include "LogFactory.h"
#include "App_Setting.h"
extern LogFactory hsrlog;
extern App_Setting app_setting;
#endif

enum InitPhase{
	INITMOTOR,
	SERVEON,
	AUTOHOME,
	CLEARENCODER,
	MANUALHOME,
	HOMEOFFSET,
	SERVEOFF
};
enum MovingDirection{
	ZERO,
	POSITIVE,
	NEGTIVE,
	GOAHEAD,
	GOBACK,
	TURNLEFT,
	TURNRIGHT
};
enum ModuleId{
	CAR,
	RIGIDHAND,
	SOFTHAND
};
//return code used by all
enum ReturnCode{
	SUCCEED,
	FAILED
};
//virtual flag used by can card
enum VirtualFlag{
	REAL,
	VIRTUAL
};
//channel id used by can card 1,2,3,4
enum ChannelId{
	CHANNEL_CAR=1,
	CHANNEL_PVU,
	CHANNEL_NORMAL_HAND,
	CHANNEL_SOFT_HAND
};
//robot action id enum ,used by ipc
enum  ActionId{
	ENTER_KEY,//keycontrol mode
	ENTER_LINE,//trajectory mode
	ENTER_HOME,//
	ENTER_ETA,
	HALT,//stop
	READY,
	HOME,//进入准备位置
	LEFT,
	BACK,
	GO,
	RIGHT,
	STOP,//小车停止运动
	SET_POINT,//给出新的路径点
	NEW_TARGET,
	NEW_DCODE,//二维码给出的，更新位姿
	SERVE_SWITCH,
	CLEAR_ENC,
	JOINT_POSITIVE_MOVE,
	JOINT_NEGTIVE_MOVE,
	CHANGE_JOINT,
	SONAR_INFO,
	CHANGE_LEVEL
};
//robot work mode ,used by main_process module
enum ContorlMode{
	NOTHING_CONTRL=0,//待机状态
	KEYBOARD_CONTRL,//标记当前轨迹来源于键盘控制还是轨迹规划的结果
	TRAJECTORY_CONTRL,
	SEARCH_CONTRL,
	ETA_TRA_CONTRL,
	EMERGENCY_CONTRL
};
//working period
#define PERIOD_CONTRL 1
#define PERIOD_FEEDBACK 1
#define PIPE_MINOR 0
#define PIPE_MINOR1 1
/*-----------physical param------------------*/
#define K_PIDAI  (1.38461538f)
#define LEFT_OFFSET (1.0000f)
#define RIGHT_OFFSET (1.0000f)
#define LINE__TRA		0
#define CIRCLE_TRA	1
#define GO_AHEAD    0
#define TURN_LEFT    1
#define TURN_RIGHT 2
#define GO_BACK   3

//小车参数
//#define Le (0.23985f*1.0016)
#define Le (0.23985f)
#define r 0.195472f
#define T 0.5f
#define SLEEP(X) (rt_task_sleep(X*1000000))
#define pi 3.14159265358979f
#define SONAR_DIS 0.8f
#endif /* PROJECTCOMMON_H_ */
