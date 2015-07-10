/*
 * CarControl.cpp
 *
 *  Created on: 2014-9-5
 *      Author: alvin
 */

#include "PVUControl.h"
#include <iostream>
using namespace std;
PVUControl::PVUControl() {
	// TODO Auto-generated constructor stub

}

PVUControl::~PVUControl() {
	// TODO Auto-generated destructor stub
}
PVUControl::PVUControl(VirtualFlag channelflag)
:channel_pvu(CHANNEL_PVU,CAN_INIT_TYPE_ST,CAN_BAUD_500K,channelflag){//init channel card with default value and virtual flag
	encoder1=0;
	encoder2=0;
	gyro=0;
	dencoder2=0;
	this->channelflag=channelflag;
	flag=0;
}
VirtualFlag PVUControl::GetVirtualFlag(){
	return this->channelflag;
}
ReturnCode PVUControl::InitModule(InitPhase phase){
	return SUCCEED;
}
double localencoder2=0;
void PVUControl::RecvFeedback(){
	DWORD ID;
	BYTE LEN;
	BYTE DATA[8];
	if(SUCCEED==channel_pvu.Recv_Frame(ID,LEN,DATA)){
		//check the data received
		float temp_data;
		switch(ID)
		{
			case 0x40:
				if(LEN==8){
					memcpy(&temp_data,DATA,4);
					encoder1=temp_data;
					memcpy(&temp_data,DATA+4,4);
					gyro=temp_data;
				}
			break;
			case 0x41:
			// this package is velocity feedback
				if(LEN==8){
					memcpy(&temp_data,DATA,4);
					encoder2=temp_data;
					memcpy(&temp_data,DATA+4,4);
					dencoder2=temp_data;
					if(dencoder2<0)dencoder2*=1.0442;
					if(dencoder2>=0)dencoder2*=1.0261;//buchang
					localencoder2+=dencoder2;
					hsrlog.Print(_Debug,"%4f   %4f   %4f  %4f\n",encoder2,localencoder2,dencoder2,gyro);
					hsrlog.Print(_Save,"%4f   %4f   %4f  %4f\n",encoder2,localencoder2,dencoder2,gyro);
					dencoder2=0;
				}

			break;
			default:;
		}
	}
}
void PVUControl::SetReference(int flag,double ref_value[]){
		BYTE  LEN=8;             // count of data bytes (0..8)
		BYTE  DATA[8];         // data bytes, up to 8
		//clear
		for(int j=0;j<7;j++){
			//create frame
			DATA[j]=j;
		}
		if(SUCCEED!=channel_pvu.Send_Frame(0x50,MSGTYPE_STANDARD,LEN,DATA)){
			return ;
		}
}
void PVUControl::GetCurrent(int & flag,double cur_value[]){

	cur_value[0]=encoder1;
	cur_value[1]=encoder2;
	cur_value[2]=dencoder2;
	cur_value[3]=gyro;
}
