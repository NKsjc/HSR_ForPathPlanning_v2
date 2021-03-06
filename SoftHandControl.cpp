/*
 * SoftHandControl.cpp
 *
 *  Created on: 2014-9-5
 *      Author: alvin
 */

#include "SoftHandControl.h"
const char ReMap_RPDO1[][9]=// receive target position
{
    {8,0x23,0x00,0x14,0x01,0x00,0x00,0x00,0x80},//disable
    {5,0x2f,0x00,0x16,0x00,0x00},//count=0
    {8,0x23,0x00,0x16,0x01,0x20,0x01,0xC1,0x60},//new map
    {5,0x2f,0x00,0x16,0x00,0x01},//new count
    {5,0x2f,0x00,0x14,0x02,0x01},//sync type
    {8,0x23,0x00,0x14,0x01,0x00,0x02,0x00,0x00}//enable
};
const char ReMap_TPDO1[][9]=// send current position
{
    {8,0x23,0x01,0x18,0x01,0x00,0x00,0x00,0x80},
    {5,0x2f,0x01,0x1A,0x00,0x00},
    {8,0x23,0x01,0x1A,0x01,0x20,0x00,0x63,0x60},
    {5,0x2f,0x01,0x1A,0x00,0x01},
    {5,0x2f,0x01,0x18,0x02,0x01},//1sync
    {8,0x23,0x01,0x18,0x01,0x80,0x02,0x00,0x00}
};

const char IP_Mode[9]={5,0x2f,0x60,0x60,0x00,0x07};//set mode=ip
const char Line_IP[][9]=
{
    {6,0x2b,0xc0,0x60,0x00,0x00,0x00},//mode=line const
    {5,0x2f,0xc2,0x60,0x01,0x01}//time=1ms
};
SoftHandControl::SoftHandControl() {
	// TODO Auto-generated constructor stub

}

SoftHandControl::~SoftHandControl() {
	// TODO Auto-generated destructor stub
}
SoftHandControl::SoftHandControl(VirtualFlag channelflag)
:channel_hand(CHANNEL_SOFT_HAND,CAN_INIT_TYPE_ST,CAN_BAUD_1M,channelflag) {
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
VirtualFlag SoftHandControl::GetVirtualFlag(){
	return this->channelflag;
}
ReturnCode SoftHandControl::HandInitCtrl(){//Init serve
	DWORD ID;
	BYTE LEN;
	BYTE DATA[8];
	for(int i=1;i<=4;i++){
		ID=0x600+i;
		//remap rpdo 1
		for(int j=0;j<6;j++){
			LEN=ReMap_RPDO1[j][0];
			for(int k=1;k<=LEN;k++)DATA[k-1]=ReMap_RPDO1[j][k];
			if(j==5)DATA[4]+=i;
			if(SUCCEED!=channel_hand.Send_Frame(ID,MSGTYPE_STANDARD,LEN,DATA)){
				return FAILED;
			}
		}
		//remap tpdo1
		for(int j=0;j<6;j++){
			LEN=ReMap_TPDO1[j][0];
			for(int k=1;k<=LEN;k++)DATA[k-1]=ReMap_TPDO1[j][k];
			if(j==5)DATA[4]+=i;
			if(SUCCEED!=channel_hand.Send_Frame(ID,MSGTYPE_STANDARD,LEN,DATA)){
				return FAILED;
			}
		}
		//set mode =ip move
		LEN=IP_Mode[0];
		for(int j=1;j<=LEN;j++)DATA[j-1]=IP_Mode[j];
		if(SUCCEED!=channel_hand.Send_Frame(ID,MSGTYPE_STANDARD,LEN,DATA)){
			return FAILED;
		}
		// set ip method =line const time =1ms
		for(int j=0;j<2;j++){
			LEN=Line_IP[j][0];
			for(int k=1;k<=LEN;k++)DATA[k-1]=Line_IP[j][k];
			if(SUCCEED!=channel_hand.Send_Frame(ID,MSGTYPE_STANDARD,LEN,DATA)){
				return FAILED;
			}
		}
	}

	return this->HandPDOOn();
}
ReturnCode SoftHandControl::HandSyncMove(){//sync move
	return  channel_hand.Send_Frame(0x80,MSGTYPE_STANDARD,0,NULL);
}
ReturnCode SoftHandControl::HandServeOn(){//enable serve
	BYTE DATA[8];
	for(int i=1;i<=4;i++){
		 DATA[0]=0x2B;DATA[1]=0x40;DATA[2]=0x60;DATA[3]=0x00;
		 DATA[4]=0x0F;DATA[5]=0x00;
		 if(SUCCEED!=channel_hand.Send_Frame(0x600+i,MSGTYPE_STANDARD,6,DATA)){
			 return FAILED;
		 }
		 DATA[0]=0x2B;DATA[1]=0x40;DATA[2]=0x60;DATA[3]=0x00;
		 DATA[4]=0x3F;DATA[5]=0x00;
		 if(SUCCEED!=channel_hand.Send_Frame(0x600+i,MSGTYPE_STANDARD,6,DATA)){
			 return FAILED;
		 }
	}
	return SUCCEED;
}
ReturnCode SoftHandControl::HandServeOff(){//disable serve
	BYTE DATA[6];
	for(int i=1;i<=4;i++){
	DATA[0]=0x2B;DATA[1]=0x40;DATA[2]=0x60;DATA[3]=0x00;
	DATA[4]=0x00;DATA[5]=0x00;
	if(SUCCEED!=channel_hand.Send_Frame(0x600+i,MSGTYPE_STANDARD,6,DATA)){
			return FAILED;
		}
	}
	return SUCCEED;
}

ReturnCode SoftHandControl::HandPDOOn(){
	DWORD ID;
	BYTE LEN;
	BYTE DATA[8];
	ID=0x000;
	LEN=2;
	DATA[0]=0x01;DATA[1]=0x00;
	return channel_hand.Send_Frame(ID,MSGTYPE_STANDARD,LEN,DATA);
}

ReturnCode SoftHandControl::HandHome(){
	return this->HandClearPosition();
}
ReturnCode SoftHandControl::HandClearPosition(){
	home_offset[0]=-enc_value[0]/(180.0f/pi/360*100*2048);
	home_offset[1]=-enc_value[1]/(180.0f/pi/360*100*2048);
	home_offset[2]=-enc_value[2]/(180.0f/pi/360*100*2048);
	home_offset[3]=-enc_value[3]/(180.0f/pi/360*100*2048);
	for(int i=0;i<4;i++)ref_value[i]=-home_offset[i];
	return SUCCEED;
}

ReturnCode SoftHandControl::InitModule(InitPhase phase){//interface inherit from super
	switch(phase){
		case INITMOTOR:
			return this->HandInitCtrl();
		break;
		case SERVEON:
			if(SUCCEED!=this->HandHome()){
				return FAILED;
			}
			return this->HandServeOn();
		break;
		case AUTOHOME:
		break;
		break;
		case MANUALHOME:
		break;
		case HOMEOFFSET:
		break;
		case SERVEOFF:
			return this->HandServeOff();
		break;
		case CLEARENCODER:
			return this->HandClearPosition();
		break;
	}
	return SUCCEED;

}

void SoftHandControl::RecvFeedback(){//interface inherit from super
	DWORD ID;
	BYTE LEN;
	BYTE DATA[8];
	if(SUCCEED==channel_hand.Recv_Frame(ID,LEN,DATA))
	{
		switch((ID/16*16))
		{
			case 0x280:
			{
			   long temp=0;temp=DATA[3];
			   temp=temp*256;temp+=DATA[2];
			   temp=temp*256;temp+=DATA[1];
			   temp=temp*256;temp+=DATA[0];
			   if (DATA[3]&0x80)temp-=0xFFFFFFFF;
			   enc_value[ID-0x281]=temp;
				if(ID==0x281){
					cur_value[0]=temp/(180.0f/pi/360*100*2048);
				}
				if(ID==0x282){
					cur_value[1]=temp/(180.0f/pi/360*100*2048);
				}
				if(ID==0x283){
					cur_value[2]=temp/(180.0f/pi/360*100*2048);
				}
				if(ID==0x284){
					cur_value[3]=temp/(180.0f/pi/360*100*2048);
				}
				cur_value[ID-0x281]+=home_offset[ID-0x281];
			}
			break;
			default:;
		}
//		std::cout<<cur_value[0]<<" "<<cur_value[1]<<" "<<cur_value[2]<<" "<<cur_value[3]<<" "<<endl;
	}
}

void SoftHandControl::SetReference(int flag,double ref_value[]){//interface inherit from super
	int value=0;
	if(ref_value==NULL){
		this->HandSyncMove();
		return;
	}
	BYTE DATA[8];
	for(int jointnum=1;jointnum<=4;jointnum++){	
		this->ref_value[jointnum-1]+=ref_value[jointnum-1];
		
		
		if(jointnum==1){
			value=this->ref_value[jointnum-1]*180/pi/360*100*2048;
		}
		if(jointnum==2){
			value=this->ref_value[jointnum-1]*180/pi/360*100*2048;
		}
		if(jointnum==3){
			value=this->ref_value[jointnum-1]*180/pi/360*100*2048;
		}
		if(jointnum==4){
			value=this->ref_value[jointnum-1]*180/pi/360*100*2048;
		}
		
		DATA[0]=value&0xFF;
		DATA[1]=(value>>8)&0xFF;
		DATA[2]=(value>>16)&0xFF;
		DATA[3]=(value>>24)&0xFF;
		if(SUCCEED!=channel_hand.Send_Frame(0x200+jointnum,MSGTYPE_STANDARD,4,DATA)){
			hsrlog.Print(_Debug,"send error\n");
			return;
		}
	}
//	hsrlog.Print(_Debug,"current set :%4f  %4f   %4f  %4f current ref :%4f  %4f   %4f  %4f current val:%4f  %4f   %4f  %4f\n",ref_value[0],ref_value[1],ref_value[2],ref_value[3],
//	this->ref_value[0],this->ref_value[1],this->ref_value[2],this->ref_value[3],
//	cur_value[0],cur_value[1],cur_value[2],cur_value[3]);
	this->HandSyncMove();

}
void SoftHandControl::GetCurrent(int & flag,double cur_value[]){//interface inherit from super
	for(int i=0;i<4;i++){
		cur_value[i]=this->cur_value[i];
	}
}
