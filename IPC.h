/*
 * IPC.h
 *
 *  Created on: 2014-9-6
 *      Author: alvin
 */

#ifndef IPC_H_
#define IPC_H_
#include "ProjectCommon.h"
#include "DataProtocol.h"
#include <native/pipe.h>
#include <native/mutex.h>
#include <list>
using namespace std;
typedef list<action_info>queue_task;
typedef list<message_info>queue_message;
typedef list<current_status>queue_status;
class IPC {
private:
	//进程间通信的临时缓冲区
	queue_task down_task;
	queue_message up_message;
	queue_status up_status;
	//与非实时进程通信的管道文件
	RT_PIPE pipe_desc;
	RT_PIPE pipe_upload;
public:
	IPC();
	virtual ~IPC();
	void Pipe_Send_Info();//4--1使用pipe上传状态信息
	void Pipe_Recv_Info();//下载指令数据到队列中
	void Insert_New_Task(action_info & temp);
	action_info Pop_New_Task();//取出任务队列的头处理
	void Push_New_Message(message_info &temp);//由各个线程调用者向队列中加入消息
	void Push_New_Status(current_status& temp);//由各个线程调用者向队列中加入状态
};

#endif /* IPC_H_ */
