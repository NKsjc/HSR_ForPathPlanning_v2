/*
 * PeakCan.h
 *
 *  Created on: 2014-9-4
 *      Author: alvin
 */

#ifndef PEAKCAN_H_
#define PEAKCAN_H_
#include"ProjectCommon.h"
#include <libpcan.h>
#include <list>
using namespace std;
typedef list<TPCANRdMsg> msg_queue;
class PeakCan {
private:
	TPSTATUS can_status; //can status
	TPCANInit can_init;//can init param
	TPCANMsg can_msg_send;//send buff
	TPCANRdMsg can_msg_recv;//recv buff
	void * handle_channel; //can channel handle
	int channel_ID; //channel id
	VirtualFlag flag;
	msg_queue virtual_msg;

public:
	PeakCan();
	PeakCan(ChannelId channel,DWORD MSGType,DWORD bardrate,VirtualFlag flag);
	virtual ~PeakCan();
	void Exit_CAN(); //close channel
	ReturnCode Open_CAN();//open channel
	ReturnCode Send_Frame(DWORD ID,BYTE MSGTYPE,BYTE LEN,BYTE  DATA[8]);//send frame
	ReturnCode Recv_Frame(DWORD & ID,BYTE &LEN,BYTE  DATA[8]);//recv frame
};
#endif /* PEAKCAN_H_ */
