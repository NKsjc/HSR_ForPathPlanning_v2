/*
 * CarControl.cpp
 *
 *  Created on: 2014-9-5
 *      Author: alvin
 */

#include "CarControl.h"
const char Clean_PDO[][9]=//clear the used pdo
{
	{8,0x23,0x00,0x18,0x01,0x00,0x00,0x00,0x80},// disable 1800-1
	{8,0x23,0x02,0x18,0x01,0x00,0x00,0x00,0x80},// disable 1802-1
	{8,0x23,0x04,0x18,0x01,0x00,0x00,0x00,0x80}// disable 1804-1
};
const char RPDO1_Map[][9]=//working mode
{
	{8,0x23,0x00,0x14,0x01,0x00,0x00,0x00,0x80},// disable 1400-1
	{5,0x2f,0x00,0x16,0x00,0x00},//clear the 1600-0 -> 0
	{8,0x23,0x00,0x16,0x01,0x08,0x00,0x60,0x60},//remap 1600-1 ->dict
	{5,0x2f,0x00,0x16,0x00,0x01},// count the 1600-0 ->new count
	{5,0x2f,0x00,0x14,0x02,0x00},// set 1400-2 para
	{8,0x23,0x00,0x14,0x01,0x00,0x02,0x00,0x00},// enable 1400-1//00---02+ID---00
};
const char RPDO2_Map[][9]=//enable set
{
	{8,0x23,0x01,0x14,0x01,0x00,0x00,0x00,0x80},//// disable 1401-1
	{5,0x2f,0x01,0x16,0x00,0x00},//clear the 1601-0 -> 0
	{8,0x23,0x01,0x16,0x01,0x10,0x00,0x00,0x23},//remap 1601-1 ->dict
	{5,0x2f,0x01,0x16,0x00,0x01},// count the 1601-0 ->new count
	{5,0x2f,0x01,0x14,0x02,0x00},// set 1401-2 para
	{8,0x23,0x01,0x14,0x01,0x00,0x03,0x00,0x00}// enable 1401-1,,//01-----00+ID-----03
};
const char RPDO3_Map[][9]=//velocity set
{
	{8,0x23,0x02,0x14,0x01,0x00,0x00,0x00,0x80},// disable 1402-1
	{5,0x2f,0x02,0x16,0x00,0x00},//clear the 1602-0 -> 0
	{8,0x23,0x02,0x16,0x01,0x20,0x00,0x41,0x23},//remap 1602-1 ->dict
	{5,0x2f,0x02,0x16,0x00,0x01},// count the 1602-0 ->new count
	{5,0x2f,0x02,0x14,0x02,0x01},// set 1402-2 para
	{8,0x23,0x02,0x14,0x01,0x00,0x04,0x00,0x00}// enable 1402-1//01-----00+ID-----04
};
const char TPDO2_Map[][9]=//velocity feedback set
{
	{8,0x23,0x01,0x18,0x01,0x00,0x00,0x00,0x80},// disable 1801-1
	{5,0x2f,0x01,0x1a,0x00,0x00},//clear the 1A01-0 -> 0
	{8,0x23,0x01,0x1a,0x01,0x20,0x00,0x6c,0x60},//remap 1A01-1 ->dict
	{5,0x2f,0x01,0x1a,0x00,0x01},// count the 1A01-0 ->new count
	{5,0x2f,0x01,0x18,0x02,0x01},// set 1801-2 para.,,,此处00代表同步方式
	{8,0x23,0x01,0x18,0x01,0x80,0x02,0x00,0x00}// enable 1801-1  //01-----80+ID-----02
};

CarControl::CarControl() {
	// TODO Auto-generated constructor stub

}

CarControl::~CarControl() {
	// TODO Auto-generated destructor stub
}
CarControl::CarControl(VirtualFlag channelflag)
:channel_car(CHANNEL_CAR,CAN_INIT_TYPE_EX,CAN_BAUD_1M,channelflag){//init channel card with default value and virtual flag
	for(int i=0;i<2;++i){
		this->cur_value[i]=0;
		this->ref_value[i]=0;
	}
	this->channelflag=channelflag;
}
VirtualFlag CarControl::GetVirtualFlag(){
	return this->channelflag;
}
ReturnCode CarControl::CarInitCtrl(){//Init Car drive module
	for(int i=1;i<=2;++i){
		if(FAILED==this->CarInitMotor(i)){
			return FAILED;
		}
	}
	return this->CarPDOOn();
}
ReturnCode CarControl::CarInitMotor(int num){//init motor num
	if(num<1||num>2){
		return FAILED;
	}
	int j=0,k=0;
	BYTE  LEN;             // count of data bytes (0..8)
	BYTE  DATA[8];         // data bytes, up to 8
	//clear
	for(j=0;j<3;j++){
		//create frame
		LEN=Clean_PDO[j][0];
		for(k=1;k<=LEN;k++)DATA[k-1]=Clean_PDO[j][k];
		if(SUCCEED!=channel_car.Send_Frame(0x600+num,MSGTYPE_STANDARD,LEN,DATA)){
			return FAILED;
		}
	}
	//map set mode
	for(j=0;j<6;j++){
		LEN=RPDO1_Map[j][0];
		for(k=1;k<=LEN;k++)DATA[k-1]=RPDO1_Map[j][k];
		if(j==5)DATA[4]=RPDO1_Map[j][5]+num;//the 5th is 0x0+ID
		if(SUCCEED!=channel_car.Send_Frame(0x600+num,MSGTYPE_STANDARD,LEN,DATA)){
			return FAILED;
		}
	}
	//map enable
	for(j=0;j<6;j++){
		LEN=RPDO2_Map[j][0];
		for(k=1;k<=LEN;k++)DATA[k-1]=RPDO2_Map[j][k];
		if(j==5)DATA[4]=RPDO2_Map[j][5]+num;//the 5th is 0x0+ID
		if(SUCCEED!=channel_car.Send_Frame(0x600+num,MSGTYPE_STANDARD,LEN,DATA)){
			return FAILED;
		}
	}
	//map velocity set
	for(j=0;j<6;j++){

		LEN=RPDO3_Map[j][0];
		for(k=1;k<=LEN;k++)DATA[k-1]=RPDO3_Map[j][k];
		if(j==5)DATA[4]=RPDO3_Map[j][5]+num;//the 5th is 0x0+ID
		if(SUCCEED!=channel_car.Send_Frame(0x600+num,MSGTYPE_STANDARD,LEN,DATA)){
			return FAILED;
		}
	}
	for(j=0;j<6;j++){
		LEN=TPDO2_Map[j][0];
		for(k=1;k<=LEN;k++)DATA[k-1]=TPDO2_Map[j][k];
		if(j==5)DATA[4]=TPDO2_Map[j][5]+num;//the 5th is0x80+ID
		if(SUCCEED!=channel_car.Send_Frame(0x600+num,MSGTYPE_STANDARD,LEN,DATA)){
			return FAILED;
		}
	}
	return SUCCEED;
}

ReturnCode CarControl::CarPDOOn(){//broadcast to open pdo
	BYTE  LEN;             // count of data bytes (0..8)
	BYTE  DATA[8];         // data bytes, up to 8
	//open pdo
	LEN=2;
	DATA[0]=0x01;
	DATA[1]=0x00;
	if(SUCCEED!=channel_car.Send_Frame(0x000,MSGTYPE_STANDARD,LEN,DATA)){
		return FAILED;
	}
	//set mode =vel
	for(int i=1;i<=2;i++){
		LEN=1;
		DATA[0]=0x03;
		if(SUCCEED!=channel_car.Send_Frame(0x200+i,MSGTYPE_STANDARD,LEN,DATA)){
			return FAILED;
		}
	}
	return SUCCEED;
}
ReturnCode CarControl::CarSyncMove(){//sync move after send reference

	return  channel_car.Send_Frame(0x80,MSGTYPE_STANDARD,0,NULL);
}
ReturnCode CarControl::InitModule(InitPhase phase){
	if(INITMOTOR==phase){
		return this->CarInitCtrl();
	}
	return SUCCEED;
}
void CarControl::RecvFeedback(){
	DWORD ID;
	BYTE LEN;
	BYTE DATA[8];
	if(SUCCEED==channel_car.Recv_Frame(ID,LEN,DATA)){
		//check the data received
		double temp_data;
		switch((ID/16*16))
		{
			case 0x280:
			// this package is velocity feedback
				temp_data=DATA[3];
				temp_data*=256;
				temp_data+=DATA[2];
				temp_data*=256;
				temp_data+=DATA[1];
				temp_data*=256;
				temp_data+=DATA[0];
				if (DATA[3]&0x80){// the negative
					temp_data-=0xFFFFFFFF;
				}
				temp_data=temp_data*0.1/186/2000*360/K_PIDAI;
				if(0x281==ID)cur_value[0]=-temp_data/LEFT_OFFSET*pi/180;
				if(0x282==ID)cur_value[1]=temp_data/RIGHT_OFFSET*pi/180;
			break;
			default:;
		}
	}
}
void CarControl::SetReference(int flag,double ref_value[]){
	double max=app_setting.get_setting_param().car_max_vel;
	for(int i=0;i<2;i++){
		if(ref_value[i]>=max){
			ref_value[i]=max;
		}else if(ref_value[i]<=-max){
			ref_value[i]=-max;
		}
	}
	int value;
	int velocity0,velocity1,velocity2,velocity3;
	BYTE DATA[8];
	value=int(-1*ref_value[0]*LEFT_OFFSET*180/pi/360*2000*10*186*K_PIDAI);
	velocity0 = value;
	velocity1 = velocity0>>8;
	velocity2 = velocity1>>8;
	velocity3 = velocity2>>8;
	DATA[0]=velocity0 & 0xFF;
	DATA[1]=velocity1 & 0xFF;
	DATA[2]=velocity2 & 0xFF;
	DATA[3]=velocity3 & 0xFF;
	if(SUCCEED!=channel_car.Send_Frame(0x400+1,MSGTYPE_STANDARD,4,DATA)){
		return ;
	}
	value=int(1*ref_value[1]*RIGHT_OFFSET*180/pi/360*2000*10*186*K_PIDAI);
	velocity0 = value;
	velocity1 = velocity0>>8;
	velocity2 = velocity1>>8;
	velocity3 = velocity2>>8;
	DATA[0]=velocity0 & 0xFF;
	DATA[1]=velocity1 & 0xFF;
	DATA[2]=velocity2 & 0xFF;
	DATA[3]=velocity3 & 0xFF;
	if(SUCCEED!=channel_car.Send_Frame(0x400+2,MSGTYPE_STANDARD,4,DATA)){
		return ;
	}
	this->CarSyncMove();
}
void CarControl::GetCurrent(int & flag,double cur_value[]){
	cur_value[0]=this->cur_value[0];
	cur_value[1]=this->cur_value[1];
}
