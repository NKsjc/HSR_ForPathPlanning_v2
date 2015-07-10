/*
 * IPC.cpp
 *
 *  Created on: 2014-9-6
 *      Author: alvin
 */

#include "IPC.h"
#include <iostream>
using namespace std;
IPC::IPC() {
	// TODO Auto-generated constructor stub
	int err5, err7;
	err5 = rt_pipe_create(&pipe_desc, NULL, PIPE_MINOR, 0);
	if (err5) {
	//	cout << "MSG_pipe: Failed to create pipe, code" << errno << endl;
	}
	err7 = rt_pipe_create(&pipe_upload, NULL, PIPE_MINOR1, 0);
	if (err7) {
	//	cout << "MSG_pipe: Failed to create pipe, code" << errno << endl;
	}
}

IPC::~IPC() {
	// TODO Auto-generated destructor stub
	down_task.clear();
	up_message.clear();
	up_status.clear();
	rt_pipe_delete(&pipe_desc);
	rt_pipe_delete(&pipe_upload);
}
void IPC::Push_New_Message(message_info &temp)//由各个线程调用者向队列中加入消息
{
	up_message.push_back(temp);
}
void IPC::Push_New_Status(current_status& temp)//由各个线程调用者向队列中加入状态
{
	up_status.push_back(temp);
}
action_info IPC::Pop_New_Task()//取出任务队列的头处理
{
	action_info temp;
	if (!down_task.empty()) {
		temp = this->down_task.front();
		this->down_task.pop_front();
	} else {
		temp.action_id = READY;
	}
	return temp;
}
void IPC::Pipe_Send_Info()//4--1使用pipe上传状态信息
{
	int err;
	if (!up_message.empty())//上传消息队列非空，则上传一次
	{
		message_info new_message = up_message.front();
		up_message.pop_front();
		err = rt_pipe_write(&pipe_desc, (void*) &new_message,
				sizeof(new_message), P_NORMAL);
		//		cout<<"send message"<<endl;
	}
	if (!up_status.empty())//上传状态队列非空，则上传一次
	{
		current_status new_status = up_status.front();
		up_status.pop_front();
		err = rt_pipe_write(&pipe_upload, (void*) &new_status,
				sizeof(new_status), P_NORMAL);
		//		cout<<"send status"<<endl;
	}

}
void IPC::Insert_New_Task(action_info & temp){
	down_task.push_front(temp);
}

void IPC::Pipe_Recv_Info()//下载指令数据到队列中
{
	//接收Pipe数据，转换成指令
	action_info new_action;
	while (rt_pipe_read(&pipe_desc, (void *) &new_action, sizeof(new_action),TM_NONBLOCK) > 0){
		//		 LOG("action_name==%s\n,ID==%d\nLevel=%d\n",new_action.action_name,new_action.action_id,new_action.action_level);
		//接收指令，处理后加入到待执行指令队列中
		down_task.push_back(new_action);
	}
}
