/*
 * PeakCan.cpp
 *
 *  Created on: 2014-9-4
 *      Author: alvin
 */

#include "PeakCan.h"
#include <string.h>
PeakCan::PeakCan() {
	// TODO Auto-generated constructor stub

}
PeakCan::PeakCan(ChannelId channel,DWORD MSGType,DWORD bardrate,VirtualFlag flag){//construct func
	can_init.ucCANMsgType=MSGType;
	can_init.ucListenOnly=0;
	can_init.wBTR0BTR1=bardrate;
	can_status.nLastError=0;
	can_status.wErrorFlag=0x0000;
	handle_channel=NULL;
	channel_ID=channel;
	this->flag=flag;
	if(REAL==flag){
		if(SUCCEED==Open_CAN()){
		}
	}else if(VIRTUAL==flag){
		virtual_msg.clear();
	}
}

PeakCan::~PeakCan() {
	// TODO Auto-generated destructor stub
	if(REAL==flag){
		Exit_CAN();
	}
}
void PeakCan::Exit_CAN(){
	if (NULL!=this->handle_channel){
		CAN_Close(this->handle_channel);
		this->handle_channel=NULL;
	}
}
ReturnCode PeakCan::Open_CAN(){
	if (NULL==this->handle_channel){
		this->handle_channel=CAN_Open(HW_PCI, channel_ID);
		if(NULL==this->handle_channel)
			return FAILED;
		CAN_Init(this->handle_channel,this->can_init.wBTR0BTR1,this->can_init.ucCANMsgType);
	}
	return SUCCEED;
}
ReturnCode PeakCan::Send_Frame(DWORD ID,BYTE MSGTYPE,BYTE LEN,BYTE  DATA[8]){
	memset(&this->can_msg_send,0,sizeof(this->can_msg_send));
	this->can_msg_send.ID=ID;
	this->can_msg_send.LEN=LEN;
	this->can_msg_send.MSGTYPE=MSGTYPE;
	for (int i=0;i<LEN;i++){
		this->can_msg_send.DATA[i]=DATA[i];
	}

	if(REAL==this->flag){//real channel send
		CAN_Write(this->handle_channel,&(this->can_msg_send));
	}else if(VIRTUAL==this->flag){// virtual channel send
		switch(this->channel_ID)//do virtual send identify by channel id
		{
			case CHANNEL_CAR://car channel
			{
				if(0x400==ID/16*16){//velocity set ref
					TPCANRdMsg temp;
					temp.Msg.ID=0x280+ID-0x400;
					temp.Msg.LEN=LEN;
					for (int i=0;i<LEN;i++)temp.Msg.DATA[i]=DATA[i];
					virtual_msg.push_back(temp);
				}
			}
			break;
			case CHANNEL_NORMAL_HAND://手臂通道处理
			{
				if(0x60==ID/16*16){//clear encoder
					TPCANRdMsg temp;
					memset(&temp,0,sizeof(temp));
					temp.Msg.LEN=4;
					temp.Msg.ID=0x741;virtual_msg.push_back(temp);
					temp.Msg.ID=0x742;virtual_msg.push_back(temp);
					temp.Msg.ID=0x743;virtual_msg.push_back(temp);
					temp.Msg.ID=0x744;virtual_msg.push_back(temp);
				}else if(0x300==ID/16*16){//set position ref
					TPCANRdMsg temp;
					temp.Msg.ID=0x740+ID-0x300;
					temp.Msg.LEN=LEN;
					for (int i=0;i<LEN;i++)temp.Msg.DATA[i]=this->can_msg_send.DATA[i];
					virtual_msg.push_back(temp);
				}
			}
			break;
			case CHANNEL_SOFT_HAND:
			{
				if(0x200==ID/16*16){//set velocity ref
					TPCANRdMsg temp;
					temp.Msg.ID=0x280+ID-0x200;
					temp.Msg.LEN=LEN;
					for (int i=0;i<LEN;i++)temp.Msg.DATA[i]=DATA[i];
					virtual_msg.push_back(temp);
				}
			}
			break;
		}
	}
	return SUCCEED;
}
ReturnCode PeakCan::Recv_Frame(DWORD & ID,BYTE &LEN,BYTE  DATA[8])//recv frame
{
	int i=0;
	memset(&this->can_msg_recv,0,sizeof(this->can_msg_recv));
	if(REAL==flag){//real channel read
		if(LINUX_CAN_Read(this->handle_channel,&(this->can_msg_recv))!=0)return FAILED;
	}else if(VIRTUAL==flag){//virtual channel read
		if(!virtual_msg.empty()){
			this->can_msg_recv=virtual_msg.front();
			virtual_msg.pop_front();
		}else{
			return FAILED;
		}
	}
	ID=this->can_msg_recv.Msg.ID;
	LEN=this->can_msg_recv.Msg.LEN;
	for (i=0;i<LEN;i++)DATA[i]=this->can_msg_recv.Msg.DATA[i];
	return SUCCEED;

}
