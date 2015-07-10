/*
 * QR_Code.c
 *
 *  Created on: 2014��10��17��
 *      Author: AlvinPC
 */


#ifndef WIN32
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

#ifdef WIN32
#include <WinSock2.h>
#pragma comment(lib, "WS2_32")	// ���ӵ�WS2_32.lib
#endif

#include"QR_Recv.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>

//#include "CImg.h"
//#include<sys/time.h>     //C���Ե�ͷ�ļ�
using namespace std;
//using namespace cimg_library;
struct QR_data {
	char strData[MAX_DECODE];
	struct point centers[4];
	double theta;
	double delay;
	unsigned int id;
}QR[QR_MAX_NUM];
point pos[9][7]={{{487,8097},{1490,8098},{2491,8099},{3499,8099},{4488,8086},{5489,8095},{6455,8098}},
				{{486,7101},{1486,7102},{2488,7103},{3486,7102},{4495,7097},{5486,7094},{6455,7092}},
				{{486,6102},{1485,6098},{2484,6103},{3481,6101},{4484,6097},{5483,6095},{6455,6094}},
				{{486,5099},{1486,5102},{2484,5105},{3485,5101},{4484,5098},{5485,5095},{6482,5097}},
				{{486,4101},{1487,4099},{2486,4101},{3486,4101},{4486,4099},{5486,4094},{6484,4094}},
				{{489,3101},{1490,3102},{2490,3105},{3490,3103},{4489,3100},{5488,3099},{6486,3097}},
				{{494,2100},{1490,2090},{2497,2092},{3492,2090},{4496,2093},{5494,2102},{6494,2098}},
				{{500,1100},{1499,1095},{2501,1095},{3497,1100},{4499,1096},{5497,1093},{6497,1096}},
				{{100,500},{100,1500},{100,2500},{8100,3500},{100,4500},{100,5500},{100,6500}}};

int cotopos[63][3]={{3001,0,0},{3002,0,1},{3003,0,2},{3004,0,3},{3005,0,4},{3006,0,5},{3007,0,6},
					{3008,1,0},{3009,1,1},{3010,1,2},{3011,1,3},{3012,1,4},{3013,1,5},{3014,1,6},
					{3015,2,0},{3016,2,1},{3017,2,2},{3018,2,3},{3019,2,4},{3020,2,5},{3021,2,6},
					{3022,3,0},{3023,3,1},{3024,3,2},{3025,3,3},{3026,3,4},{3027,3,5},{3028,3,6},
					{3029,4,0},{3030,4,1},{3031,4,2},{3032,4,3},{3033,4,4},{3034,4,5},{3035,4,6},
					{3036,5,0},{3037,5,1},{3038,5,2},{3039,5,3},{3040,5,4},{3041,5,5},{3042,5,6},
					{3043,6,0},{3044,6,1},{3045,6,2},{3046,6,3},{3047,6,4},{3048,6,5},{3049,6,6},
					{3050,7,0},{3051,7,1},{3052,7,2},{3053,7,3},{3054,7,4},{3055,7,5},{3056,7,6},
					{3057,8,0},{3058,8,1},{3059,8,2},{3060,8,3},{3061,8,4},{3062,8,5},{3063,8,6}};

point off[9]={{-22,22},{0,22},{22,22},{-22,0},{0,0},{22,0},{-22,-22},{0,-22},{22,-22}};



point curpos;
double curang;

void initpos()//����
{
	for(int i=0;i<9;i++)
		for(int j=0;j<7;j++)
		{
			pos[i][j].y=8100-1000*i;
			pos[i][j].x=500+1000*j;
		}
}

double GetAng(struct QR_data *qr)
{
	return qr[0].theta;
}

struct point GetPos(struct QR_data *qr)
{
	int tep;
	double len;
	unsigned int i;
	unsigned int j;
	unsigned int k;
	pointf p,pc,poff;
	point imagec;
	imagec.x = 0;
	imagec.y = 0;
	tep = (qr[0].strData[0]-0x30)*1000 + (qr[0].strData[1]-0x30)*100 + (qr[0].strData[2]-0x30)*10 + (qr[0].strData[3]-0x30);
	//i=(tep/7);
	//j=(tep%7)-1;
	if(tep == 3094)
		tep = 3001;
	for(int m=0; m<63; m++)
	{
		if(tep == cotopos[m][0])
		{
			i=cotopos[m][1];
			j=cotopos[m][2];

			k=qr[0].strData[6]-0x30-1;
			len=(double)((qr[0].centers[0].x-qr[0].centers[1].x)*(qr[0].centers[0].x-qr[0].centers[1].x)+(qr[0].centers[0].y-qr[0].centers[1].y)*(qr[0].centers[0].y-qr[0].centers[1].y));
			len=sqrt(len)/11;
			p.x=pos[i][j].x+off[k].x;
			p.y=pos[i][j].y+off[k].y;
			pc.x=(qr[0].centers[0].x+qr[0].centers[1].x+qr[0].centers[2].x+qr[0].centers[3].x)/4.0;
			pc.y=480-(qr[0].centers[0].y+qr[0].centers[1].y+qr[0].centers[2].y+qr[0].centers[3].y)/4.0;
			poff.x=p.x-(pc.x/len*cos(curang)-(pc.y/len)*sin(curang));
			poff.y=p.y-(pc.x/len*sin(curang)+(pc.y/len)*cos(curang));
			imagec.x=poff.x+376/len*cos(curang)-(480-240)/len*sin(curang);
			imagec.y=poff.y+376/len*sin(curang)+(480-240)/len*cos(curang);

		}
	}
	return imagec;
}


int GetSerial(struct QR_data *qr)
{
	int tep;
	tep = (qr[0].strData[0]-0x30)*1000 + (qr[0].strData[1]-0x30)*100+ (qr[0].strData[2]-0x30)*10 + (qr[0].strData[3]-0x30);
	return tep;
}



unsigned int recv_data(int ser, char *datap, unsigned int len)
{
	unsigned int data_len;
	int n;

	if(datap == NULL){
		return -1;
	}
	data_len = len;
	while(data_len > 0){
		n = recvfrom(ser, datap, data_len, MSG_DONTWAIT,NULL, NULL);
//		n = recvfrom(ser, datap, data_len, 0,NULL, NULL);
		if(n <= 0){
//			printf("recv count %d\n",n);
			return -1;
		}
		data_len -= n;
		datap += n;
	}
	return len;
}

void get_QR_code(int socket,float &p_x,float &p_y,float&p_st,int & serial,int & QR_flag,double & delay)
{
	p_x = 0;
	p_y = 0;
	p_st = 0;
	serial = 0;
	QR_flag=0;
	char* data;
	int red = recv_data(socket, (char *)QR, sizeof(QR_data));
	if(red<0)
	{
//		printf("error data!\n");
		return;
	}
//printf("%s,%f\n",QR[0].strData,QR[0].theta);
//	printf("sizeof %d    %s  %d  %d  %d  %d  %d  %d  %d  %d  %f  %f		%u\n",sizeof(QR_data),QR[0].strData,\
			QR[0].centers[0].x,QR[0].centers[0].y,\
			QR[0].centers[1].x,QR[0].centers[1].y,\
			QR[0].centers[2].x,QR[0].centers[2].y,\
			QR[0].centers[3].x,QR[0].centers[3].y,QR[0].theta,QR[0].delay,QR[0].id);

	if (QR[0].strData[0])
	{
		curang = -1*GetAng(QR);
		curpos = GetPos(QR);
		QR_flag = 1;
	}
	p_x=curpos.x*1.0f/1000;
	p_y=curpos.y*1.0f/1000;
	p_st=curang+3.1415926/2;
	serial= GetSerial(QR);
	delay=QR[0].delay;
}
