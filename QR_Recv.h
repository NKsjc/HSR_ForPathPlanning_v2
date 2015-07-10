/*
 * QR_Code.h
 *
 *  Created on: 2014��10��17��
 *      Author: AlvinPC
 */

#ifndef QR_CODE_H_
#define QR_CODE_H_

//#define DLL_EXPORT extern "C" __declspec(dllexport)
#define		PRO_HEADER	0xaa

#define		PRO_TYPE1		0x1
#define		PRO_TYPE2		0x2
#define		PRO_TYPE3		0x3
#define		PRO_TYPE4		0x4
#define		PRO_TYPE255		0xff
extern  const double un_map[752*480][2];
struct pro_format{
	char	header;
	char	type;
	char	align1;
	char	align2;
	unsigned int len;
	char	*p;
};

#define MAX_DECODE 16	//һ������������ַ���
#define QR_MAX_NUM 1		//һ�ν⼸����

struct point {
	int x;
	int y;
};
struct pointf {
	double x;
	double y;
};
unsigned int recv_data(int ser, char *datap, unsigned int len);
void get_QR_code(int socket,float &p_x,float &p_y,float&p_st,int & serial,int & QR_flag,double & delay);
void initpos();//����
struct point GetPos(struct QR_data *qr);//��ȡλ��
double GetAng(struct QR_data *qr);//��ȡ�Ƕ�
int GetSerial(struct QR_data *qr);//��ȡ���к�

#endif /* QR_CODE_H_ */
