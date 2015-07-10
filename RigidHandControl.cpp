/*
 * RigidHandControl.cpp
 *
 *  Created on: 2014-9-5
 *      Author: alvin
 */

#include "RigidHandControl.h"
double HAND_POSITIVE_LIMIT[4]={0.5,0.5,0.5,3.14159};
double HAND_NEGTIVE_LIMIT[4]={-0.5,-0.5,-0.5,-3.14159};
RigidHandControl::RigidHandControl() {
	// TODO Auto-generated constructor stub

}

RigidHandControl::~RigidHandControl() {
	// TODO Auto-generated destructor stub
}
RigidHandControl::RigidHandControl(VirtualFlag channelflag)
:channel_hand(CHANNEL_NORMAL_HAND,CAN_INIT_TYPE_ST,CAN_BAUD_1M,channelflag){
	// TODO Auto-generated constructor stub
	for(int i=0;i<4;++i){
		cur_value[i]=0;
		ref_value[i]=0;
		enc_value[i]=0;
		home_offset[i]=0;
		home_flag[i]=1;
	}
	this->channelflag=channelflag;
}
VirtualFlag RigidHandControl::GetVirtualFlag(){
	return this->channelflag;
}
ReturnCode RigidHandControl::HandServeOn(){//serve on
	return  channel_hand.Send_Frame(0x80,MSGTYPE_RTR,0,NULL);
}
ReturnCode RigidHandControl::HandSyncMove(){//sync move
	if(SUCCEED!=channel_hand.Send_Frame(0x120,MSGTYPE_RTR,0,NULL)){
		return FAILED;
	}
	return  channel_hand.Send_Frame(0xA0,MSGTYPE_RTR,0,NULL);
}
ReturnCode RigidHandControl::HandServeOff(){//serve off
	return  channel_hand.Send_Frame(0x100,MSGTYPE_RTR,0,NULL);
}
void RigidHandControl::HandSetOffset(){//set home offset
	home_offset[0]=0;home_offset[1]=0;home_offset[2]=-pi/2;home_offset[3]=0;
	home_flag[0]=1;home_flag[1]=-1;home_flag[2]=-1;home_flag[3]=1;
	HAND_POSITIVE_LIMIT[0]=1.8;HAND_POSITIVE_LIMIT[1]=0.3;HAND_POSITIVE_LIMIT[2]=-0.3;HAND_POSITIVE_LIMIT[3]=3.14159;
	HAND_NEGTIVE_LIMIT[0]=-1.0;HAND_NEGTIVE_LIMIT[1]=-1.0;HAND_NEGTIVE_LIMIT[2]=-3.0;HAND_NEGTIVE_LIMIT[3]=-3.1415926;
	for(int k=0;k<4;k++)ref_value[k]=home_offset[k];
}
ReturnCode RigidHandControl::HandClearPosition(){//clear encoder
	return channel_hand.Send_Frame(0x60,MSGTYPE_RTR,0,NULL);
}
ReturnCode RigidHandControl::HandHome(){//auto home
	return channel_hand.Send_Frame(0x20,MSGTYPE_RTR,0,NULL);
}

ReturnCode RigidHandControl::InitModule(InitPhase phase){//interface inherit from super
	switch(phase){
	case INITMOTOR:
	break;
	case SERVEON:
		return this->HandServeOn();
	break;
	case AUTOHOME:
		return this->HandHome();
	break;
	case CLEARENCODER:
		return this->HandClearPosition();
	break;
	case MANUALHOME:
	break;
	case HOMEOFFSET:
		if(SUCCEED!=this->HandClearPosition()){
			return FAILED;
		}
		this->HandSetOffset();
		return SUCCEED;
	break;
	case SERVEOFF:
		return this->HandServeOff();
	break;
	}
	return SUCCEED;
}
void RigidHandControl::RecvFeedback(){//interface inherit from super
	DWORD ID;
	BYTE LEN;
	BYTE DATA[8];
	if(SUCCEED==channel_hand.Recv_Frame(ID,LEN,DATA)){
		switch((ID/16*16))
		{
			case 0x740:
			{
				if(ID<0x74||ID>0x744){
					return;
				}
			   long temp=0;temp=DATA[3];
			   temp=temp*256;temp+=DATA[2];
			   temp=temp*256;temp+=DATA[1];
			   temp=temp*256;temp+=DATA[0];
			   if (DATA[3]&0x80){// the negative
				   temp-=0xFFFFFFFF;
				}
			   enc_value[ID-0x741]=temp;
			   temp*=home_flag[ID-0x741];// change feedback from motor sapce -> joint space
			   if(ID==0x741){
					cur_value[0]=temp/(180.0f/pi/360*100*4000)+home_offset[0];
				}
			   if(ID==0x742){
					cur_value[1]=temp/(1.6*180.0f/pi/360*100*4000)+home_offset[1];
				}
				if(ID==0x743){
					cur_value[2]=temp/(1.6*180.0f/pi/360*100*2048)+home_offset[2];
				}
				if(ID==0x744){
					cur_value[3]=temp/(180.0f/pi/360*100*2048)+home_offset[3];
				}
//				std::cout<<cur_value[0]<<" "<<cur_value[1]<<" "<<cur_value[2]<<" "<<cur_value[3]<<endl;
			}
			break;
			default:;
		}
	}
}
void RigidHandControl::SetReference(int flag,double ref_value[]){//interface inherit from super
	double target=0;
	int value=0;
	BYTE DATA[8];
	channel_hand.Send_Frame(0x120,MSGTYPE_RTR,0,NULL);//get period feedback;
	if(NULL==ref_value){
		return;
	}
	for(int i=0;i<4;i++){
		this->ref_value[i]+=ref_value[i];
		if(this->ref_value[i]>=HAND_POSITIVE_LIMIT[i]){
			this->ref_value[i]=HAND_POSITIVE_LIMIT[i];
			//print warning
		}
		if(this->ref_value[i]<=HAND_NEGTIVE_LIMIT[i]){
			this->ref_value[i]=HAND_NEGTIVE_LIMIT[i];
			//print waning
		}
		target=(this->ref_value[i]-home_offset[i])*home_flag[i]; // change set ref from joint space -> motor space
		if(i==0){
			value=target*180/pi/360*100*4000;
		}
		if(i==1){
			value=target*1.6*180/pi/360*100*4000;
		}
		if(i==2){
			value=target*1.6*180/pi/360*100*2048;
		}
		if(i==3){
			value=target*180/pi/360*100*2048;
		}
		DATA[0]=value&0xFF;
		DATA[1]=(value>>8)&0xFF;
		DATA[2]=(value>>16)&0xFF;
		DATA[3]=(value>>24)&0xFF;
		if(SUCCEED!=channel_hand.Send_Frame(0x301+i,MSGTYPE_STANDARD,4,DATA)){
			return;
		}
	}
	this->HandSyncMove();
}
void RigidHandControl::GetCurrent(int & flag,double cur_value[]){//interface inherit from super
	for(int i=0;i<4;++i)cur_value[i]=this->cur_value[i];
}
