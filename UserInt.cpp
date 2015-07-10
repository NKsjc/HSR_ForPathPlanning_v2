/*
 * UsrInt.cpp
 *
 *  Created on: 2014-11-1
 *      Author: alvin
 */

#include<stdio.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<sys/time.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "DataProtocol.h"
#define _SOCKETBIND_//添加标志，以表明此文件要使用unix socket bind 不能和boost bind 冲突

#include "ProjectCommon.h"
#include <termios.h>
#include "QR_Recv.h"
#include<iostream>
using namespace std;
#include "HICDataHelper.h"
#include <math.h>






//////////////////////////////////////////////////////////////////////////
//函数申明
int Init_Pipe_Message();//初始化管道通信的变量
void cleanup(int signal);//程序退出时的清理工作
void Add_Keyboard_Task(char keytask);//根据键盘指令生成task信息，并发送到pipe队列
void * Process_Log_Callback(void *);//处理记录log信息的回调函数
void * Process_Keyboard_Callback(void *);//处理键盘指令的回调函数
void * Process_Socket_Callback(void *);//处理来自外接网络的命令，socket连接的指令
void * Process_QR_Callback(void *);//处理二维码的回调函数
void * Process_Sonar_Callback(void *);//处理超声波的回调函数
void * Process_RemoteContrl_Callback(void *);

int GetPacket(void * buff,int size);//从socket数据解析指令
int Init_UART();//初始化uart串口
int RecvSensorData(int fd,float length[8]);//recv data
//////////////////////////////////////////////////////////////////////////
//全局变量
#define PIPE_MINOR 0
#define PIPE_MINOR1 1
int pipe_fd;
int pipe_upload_fd;
static struct termios backup;
//socket监听句柄
int listenfd;
int udp_socketfd;
#define SENDCMD(A) write(pipe_fd, (void*)&A, sizeof(A))

//向外界传递消息，例如上传状态信息
#define SERVER_IP "192.168.3.6"
#define SERVER_PORT 9001
#define UDP_SERVER_PORT 9002
#define UDP_REMOTE_CONTRL_PORT 9000

#define Delta_x 0.015f
#define Delta_y 0
//控制程序运行逻辑

contrl_param hsr_contrl;
//////////////////////////////////////////////////////////////////////////
//多线程结构体
pthread_t socket_thread;
pthread_t keyboard_thread;
pthread_t log_thread;
pthread_t QR_thread;
pthread_t Sonar_thread;
pthread_t Remote_thread;

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void * Process_QR_Callback(void *)//处理二维码的回调函数
{
	int sock;
	struct sockaddr_in servaddr;
	int QR_flag;
	float p_x,p_y,p_st;
	int serial;
	timeval start_time,stop_time;
 	struct timeval tv;
 	double delay=0;


	if((sock=socket(AF_INET,SOCK_DGRAM,0))<0){
			return 0;
	}
	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr =htonl(INADDR_ANY);
	servaddr.sin_port = htons(7777);

	if (bind(sock, (struct sockaddr*)&servaddr, sizeof(struct sockaddr_in))<0){
		close(sock);
		return 0;
	}
	while(!hsr_contrl.key_end)
	{
//		usleep(60000);
		gettimeofday(&start_time,NULL);
		delay=0;
		get_QR_code(sock,p_x,p_y,p_st,serial,QR_flag,delay);
		gettimeofday(&stop_time,NULL);
		if(QR_flag==1)
		{
//			cout<<p_x<<"  "<<p_y<<"  "<<p_st<<"   "<<serial<<"  "<<delay<<"  "<<\
								(stop_time.tv_sec-start_time.tv_sec)*1000+(stop_time.tv_usec-start_time.tv_usec)/1000.0f<<endl;
			action_info tempaction={0};
			tempaction.action_id=NEW_DCODE;
			tempaction.target_car[0]=p_x-(cos(p_st)*Delta_x-sin(p_st)*Delta_y);//x
			tempaction.target_car[1]=p_y-(sin(p_st)*Delta_x+cos(p_st)*Delta_y);//y
			tempaction.target_car[3]=p_st;//st
			tempaction.target_car[2]=serial;//序列号
			tempaction.dead_time=(stop_time.tv_sec-start_time.tv_sec)*1000+(stop_time.tv_usec-start_time.tv_usec)/1000.0f+delay;
			SENDCMD(tempaction);
			QR_flag=0;
		}
	}
}
int Init_UART(){
		//open uart
		int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
		    B38400, B19200, B9600, B4800, B2400, B1200, B300, };
		int fd;
		char *dev ="/dev/ttyS0";
		fd = open( dev, O_RDWR| O_NOCTTY | O_NDELAY );//O_NONBLOCK );
		if (-1 == fd)
		{ /*设置数据位数*/
			perror("Can't Open Serial Port");
			return -1;
		}
		fcntl(fd,F_SETFL,0);
		struct termios options;
		/* get the current options */
		if(tcgetattr(fd,&options)!=0)
		{
			return -1;
		}
		cfsetispeed(&options,speed_arr[4]);
		cfsetospeed(&options,speed_arr[4]);
		options.c_cflag |= CLOCAL;
		options.c_cflag |= CREAD;
		options.c_cflag &= ~CRTSCTS;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;
		options.c_cflag &= ~PARENB;
		options.c_iflag &= ~INPCK;
		options.c_cflag &= ~CSTOPB;
		options.c_oflag &= ~OPOST;
		options.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
		options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
		options.c_lflag        = 0;
		options.c_oflag        = 0;
		options.c_iflag        = 0;
		options.c_cc[VTIME]=5;
		options.c_cc[VMIN]=10;
		tcflush(fd, TCIFLUSH);
		if(tcsetattr(fd,TCSANOW,&options)!=0)
		{
			return -1;
		}
		return fd;
}
int RecvSensorData(int fd,float length[8])
{
	char sensor_cmd[3]={'1','3','2'};
	unsigned char recvbuff[10];
	int nread;
	tcflush(fd,TCIOFLUSH);
	nread=write(fd,&sensor_cmd[0],1);//begin measure;
//	printf("begin measure %d\n",nread);
	usleep(80000);
	nread=write(fd,&sensor_cmd[1],1);//begin recv channel 1;
	nread = read(fd,recvbuff,10);
	if(nread<10)
	{
		printf("1 du \n");
		tcflush(fd,TCIOFLUSH);
		return -1;
	}
//	printf("recv channel one %d    ",nread);
	if(nread>0){
		for(int i=0;i<4;i++)
		{
			length[i]=((recvbuff[i*2+1]-0x00)*256+(recvbuff[i*2+2]-0x00))/58;
			length[i]/=100;
//			printf("%4f ",length[i]);
		}
//		printf("        ");
		memset(recvbuff,0,sizeof(recvbuff));
	}
	nread=write(fd,&sensor_cmd[2],1);//begin recv channel 2;
	nread = read(fd,recvbuff,10);
	if(nread<10)
	{
		printf("2 du \n");
		tcflush(fd,TCIOFLUSH);
		return-1;
	}
//	printf("recv channel two %d     ",nread);
	if(nread>0){
		for(int i=0;i<4;i++)
		{
			length[i+4]=((recvbuff[i*2+1]-0x00)*256+(recvbuff[i*2+2]-0x00))/58;
			length[i+4]/=100;
//			printf("%4f ",length[i+4]);
		}
//		printf("\n");
		memset(recvbuff,0,sizeof(recvbuff));
	}
	return 0;
}
void * Process_Sonar_Callback(void *){//处理超声波的回调函数

	int uartfd=Init_UART();
	if(uartfd<0)
		return NULL;
	float sonardata[8];
//	int islastsend=0;
	while(1){
		tcflush(uartfd,TCIOFLUSH);
		if(RecvSensorData(uartfd,sonardata)==0){
//			cout<<"sonar data:"<<sonardata[0]<<"  "<<sonardata[2]<<"  "<<sonardata[7]<<endl;
			if(sonardata[0]<SONAR_DIS||sonardata[2]<SONAR_DIS||sonardata[7]<SONAR_DIS){
				//需要处理的超声信息
//				islastsend=1;//上次发送标志
				action_info tempaction={0};
				tempaction.action_id=SONAR_INFO;
				tempaction.sonar_info[0]=1;//开始预警
				tempaction.sonar_info[1]=sonardata[0];
				tempaction.sonar_info[2]=sonardata[2];
				tempaction.sonar_info[3]=sonardata[7];
				SENDCMD(tempaction);
			}
			else{
//				if(islastsend==1){
					//上次发送的还是预警，所以这次停止预警
					action_info tempaction={0};
					tempaction.action_id=SONAR_INFO;
					tempaction.sonar_info[0]=-1;//取消预警
					tempaction.sonar_info[1]=sonardata[0];
					tempaction.sonar_info[2]=sonardata[2];
					tempaction.sonar_info[3]=sonardata[7];
					SENDCMD(tempaction);
//					islastsend=0;
//				}
			}
		}
	}
	close(uartfd);
}

int GetPacket(void * buff,int size)//从socket数据解析指令
{
	if(size<7)return -1;
	char msg_type;
	memcpy((void *)&msg_type,buff+2,sizeof(char));//提取长度
	cout<<"msg_type"<<(int)msg_type<<endl;
//	cout<<"buff";
	for(int i=0;i<size;i++)
	{
		char dataaa;
		memcpy((void *)&dataaa,buff+i,sizeof(char));//提取长度
		cout<<(int)dataaa<<"  ";
	}
	cout<<endl;
	switch(msg_type)
	{
		case 1:
		//机器人路径规划
		{
			int msg_len;
			action_info tempaction={0};
			memcpy((void *)&msg_len,buff+3,sizeof(int));//提取长度
			//路径点个数匹配
			float temp;
			for (int i=0;i<msg_len/4;i+=6)
			{
				tempaction.action_id=SET_POINT;
				memcpy((void *)&temp,buff+7+i*4,4);
				tempaction.target_car[0]=temp;
				memcpy((void *)&temp,buff+7+4+i*4,4);
				tempaction.target_car[1]=temp;
				memcpy((void *)&temp,buff+7+4+4+i*4,4);
				tempaction.target_car[2]=temp;
				memcpy((void *)&temp,buff+7+4+4+4+i*4,4);
				tempaction.target_car[3]=temp;
				SENDCMD(tempaction);
//				cout<<"new setpoint:"<<tempaction.target_car[0]<<"    "<<tempaction.target_car[1]<<endl;
			}
		}
		break;
		case 2:
			//机器人前后左右控制
		{
			int move_type;
			memcpy((void *)&move_type,buff+3,sizeof(int));//提取运动方向0-停，3-前，4-后，1-左，2-右
			//提取socket的数据，转换成指令
			action_info tempaction={0};
			//此处包含了家庭信息中心关于指令的协议解析,提取指令
			switch(move_type)
			{
			case 1:
				tempaction.action_id=LEFT;SENDCMD(tempaction);
				break;
			case 2:
				tempaction.action_id=RIGHT;SENDCMD(tempaction);
				break;
			case 4:
				tempaction.action_id=BACK;SENDCMD(tempaction);
				break;
			case 3:
				tempaction.action_id=GO;SENDCMD(tempaction);
				break;
			case 0:
				tempaction.action_id=STOP;SENDCMD(tempaction);
				break;
			default :;
			}
//			cout<<"car move"<<move_type<<endl;
		}
		break;
		case 6:
		{
			//change level
			action_info tempaction={0};
			int level;
			memcpy((void *)&level,buff+3,sizeof(int));
			if(level>=1&&level<=3){
				tempaction.action_id=CHANGE_LEVEL;
				tempaction.level=level;
				SENDCMD(tempaction);
			}
		}
		break;

		case 3:
		{
			action_info tempaction={0};
			int hand_num;
			memcpy((void *)&hand_num,buff+3,sizeof(int));//提取运动手臂编号1-左，2-右
			int joint_num;
			memcpy((void *)&joint_num,buff+7,sizeof(int));//提取运动关节1-4
			int move_type;
			memcpy((void *)&move_type,buff+11,sizeof(int));//提取运动方向1--正，2-负
			cout<<"move hand"<<hand_num<<"  move joint"<<joint_num<<"  move type"<<move_type<<endl;
			//发送指令
			if(joint_num>=1&&joint_num<=4&&hand_num>=1&&hand_num<=2)
			{
				tempaction.action_id=CHANGE_JOINT;
				tempaction.hand_num=hand_num;
				tempaction.joint_num=joint_num;
				SENDCMD(tempaction);
			}
			if(move_type==1){tempaction.action_id=JOINT_NEGTIVE_MOVE;SENDCMD(tempaction);}
			if(move_type==2){tempaction.action_id=JOINT_POSITIVE_MOVE;SENDCMD(tempaction);}
		}
		break;
		case 5:
		{
			//手臂规划控制
			action_info tempaction={0};
			int hand_num;
			memcpy((void *)&hand_num,buff+3,sizeof(int));//提取运动手臂编号1-左 2-右
			float setref[4];
			memcpy(setref,buff+7,sizeof(setref));//提取角度
			cout<<"new target"<<setref[0]<<"  "<<setref[1]<<"  "<<setref[2]<<"  "<<setref[3]<<endl;
			tempaction.hand_num=hand_num;
			for(int i=0;i<4;i++)tempaction.target_hand_st[i]=setref[i];
			tempaction.action_id=NEW_TARGET;
			SENDCMD(tempaction);
		}
		break;
	}


}
void * Process_RemoteContrl_Callback(void *)
{
	int sock;
	struct sockaddr_in servaddr;
	if((sock=socket(AF_INET,SOCK_DGRAM,0))<0){
			return 0;
	}
	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr =htonl(INADDR_ANY);
	servaddr.sin_port = htons(UDP_REMOTE_CONTRL_PORT);

	if (bind(sock, (struct sockaddr*)&servaddr, sizeof(struct sockaddr_in))<0){
		close(sock);
		return 0;
	}
	char pbuff[1024];
	while(!hsr_contrl.key_end)
	{
		int red =recvfrom(sock, pbuff, 1024, MSG_DONTWAIT,NULL, NULL);//read data max=1024
		GetPacket(pbuff,red);//从socket数据解析指令
		usleep(1000);//delay 1ms
	}
	close(sock);
}

void *Process_Socket_Callback(void *)//处理来自外接网络的命令，socket连接的指令
{
#if 1
	struct sockaddr_in servaddr;
	struct timeval tv;
	tv.tv_sec=1;
	tv.tv_usec=0;//1s延迟处理
	//初始化监听socket
	if( (listenfd = socket(AF_INET, SOCK_STREAM, 0)) == -1 )
	{
		printf("create socket error: %s(errno: %d)\n",strerror(errno),errno);
		return NULL;
	}

	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = inet_addr(SERVER_IP);
	servaddr.sin_port = htons(SERVER_PORT);

	if(connect(listenfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1)
	{
		printf(" Failed connect() \n");
		return NULL;
	}

	float fArray[] = {10.0f, 12.0f, -29.0f};
	//创建换一个数据包
	HICDataPack *pPack = HICCreatePack();
	//3.设置当前发送数据设备的类型
	HICSetDeviceType(pPack, 1);
	//4. 设置当前发送数据设备的设备名
	HICSetDeviceName(pPack, 4);
	//5.设置当前发送的数据类型
	HICSetDataType(pPack, 3);
	//6.将需要发送的数据添加到数据包结构体之中
	HICAddFloatArray(pPack, fArray, 3);
	//7.将数据包结构体转换为符合家庭信息中心标准的字节流
	int nDataLength = 0;
	char *pBuf = (char *)HICPackToByte(pPack, &nDataLength);
	int size;
	//8.发送数据
	size=send(listenfd, pBuf, nDataLength, 0);
	HICReleasePack(pPack);
	if (size<=0)
	{
		return NULL;
	}

	fd_set fd_all;
	FD_ZERO(&fd_all);	//FD_ZERO将指定的文件描述符集清空，在对文件描述符集合进行设置前，必须对其进行初始化
	FD_SET(listenfd, &fd_all);
#if 1
	int ret=-1;
	int result;
	char pBuff[1024];
	//提取socket的数据，转换成指令
	action_info tempaction={0};
	while(!hsr_contrl.key_end)
	{
		fd_set fdread = fd_all;
		ret = select(listenfd+1, &fdread, NULL, NULL, &tv);	//select函数
		if (ret >0)
		{
			ret=-1;
			//此时连接发生变化，可能是数据，也可能是断开连接

			memset(pBuff,0,sizeof(pBuff));
			ret=recv(listenfd,pBuff,1024,0);
			cout<<"recv data"<<endl;
			GetPacket(pBuff,ret);//从socket数据解析指令
			//此处必须延时10ms的一段时间，因为子线程还来不急处理select变化，父线程就进行下次循环了
			usleep(10000);
		}

	}//end while

	//线程结束，开始销毁socket
	close(listenfd);
	printf("Socket Thread is stop now !!,release contrl==%d\n",hsr_contrl.release_contrl);
	hsr_contrl.release_contrl++;

#endif

#endif
}
int kbhit(void) //检查键盘是否有输入
{

	fd_set rfds;
	struct timeval tv;
	int retval;
	/* Watch stdin (fd 0) to see when it has input. */
	FD_ZERO(&rfds);
	FD_SET(0, &rfds);
	/* Wait up to five seconds. */
	tv.tv_sec  = 0;
	tv.tv_usec = 0;
	retval = select(1, &rfds, NULL, NULL, &tv);
	/* Don't rely on the value of tv now! */

	if (retval == -1)
	{
		perror("select()");
		return 0;
	}
	else
		if (retval)
			return 1;
	/* FD_ISSET(0, &rfds) will be true. */
		else
			return 0;
	return 0;
}
int main(int argc, char *argv[])
{

	int ret=Init_Pipe_Message();
	if (ret<0)
	{
		return -1;
	}
	memset(&hsr_contrl,0,sizeof(hsr_contrl));
	//生成多线程，分开处理各种输入
	signal(SIGINT, cleanup); 
	//线程设置的参数
	pthread_attr_t thattr;
	pthread_attr_init(&thattr);
	pthread_attr_setdetachstate(&thattr, PTHREAD_CREATE_JOINABLE);
	ret = pthread_create(&socket_thread, &thattr, Process_Socket_Callback, NULL);
	if (ret)
	{
		errno = ret;
		perror("pthread_create failed");
		return 1;
	}
	ret = pthread_create(&Sonar_thread, &thattr, Process_Sonar_Callback, NULL);
	if (ret)
	{
		errno = ret;
		perror("pthread_create failed");
		return 1;
	}
	ret = pthread_create(&QR_thread, &thattr, Process_QR_Callback, NULL);
	if (ret)
	{
		errno = ret;
		perror("pthread_create failed");
		return 1;
	}

	//生成接收线程
	ret = pthread_create(&keyboard_thread, &thattr, Process_Keyboard_Callback, NULL);
	if (ret)
	{
		errno = ret;
		perror("pthread_create failed");
		return 1;
	}
	//生成交互线程
	ret = pthread_create(&log_thread, &thattr, Process_Log_Callback, NULL);
	if (ret)
	{
		errno = ret;
		perror("pthread_create failed");
		return 1;
	}
	ret = pthread_create(&Remote_thread, &thattr, Process_RemoteContrl_Callback, NULL);
	if (ret)
	{
		errno = ret;
		perror("pthread_create failed");
		return 1;
	}
	pthread_join(Remote_thread, NULL);
	pthread_join(Sonar_thread, NULL);
	pthread_join(socket_thread, NULL);
	pthread_join(keyboard_thread, NULL);
	pthread_join(log_thread, NULL);
	pthread_join(QR_thread, NULL);

	//程序结束，关闭文件和管道
	cleanup(1);
}
void cleanup(int signal)
{
	close(pipe_fd);
	close(pipe_upload_fd);
	exit(0);
}


int Init_Pipe_Message()//初始化管道通信的变量
{
	char devname[32];
	char keycmd;
	int err;
	//打开pipe管道
	sprintf(devname, "/dev/rtp%d", PIPE_MINOR);
	pipe_fd = open(devname, O_RDWR);
	if (pipe_fd < 0)
	{
		printf("打开管道Pipe文件失败\n");
		return -1;
	}
	sprintf(devname,"/dev/rtp%d",PIPE_MINOR1);
	pipe_upload_fd=open(devname, O_RDWR);
	if (pipe_upload_fd < 0)
	{
		printf("打开管道Pipe文件失败\n");
		return -1;
	}
	return 0;
}
void *Process_Log_Callback(void *)//处理记录log信息的回调函数
{
	message_info new_message={0};
	current_status new_uploadInfo={0};
	fd_set fd_all;
	FD_ZERO(&fd_all);	//FD_ZERO将指定的文件描述符集清空，在对文件描述符集合进行设置前，必须对其进行初始化
	FD_SET(pipe_fd, &fd_all);
	FD_SET(pipe_upload_fd, &fd_all);

	struct sockaddr_in servaddr;
	//初始化udp_socket
	if( (udp_socketfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1 )
	{
		printf("create socket error: %s(errno: %d)\n",strerror(errno),errno);
		return NULL;
	}

	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = inet_addr(SERVER_IP);
	servaddr.sin_port = htons(UDP_SERVER_PORT);

	int ret=-1;
	struct timeval tv;
	tv.tv_sec=0;
	tv.tv_usec=0;
	int max_fd=pipe_fd>pipe_upload_fd?pipe_fd:pipe_upload_fd;
	int nDataLength = 0;
	float fArray[3];
	char *pBuf ;
	FILE * logfile=fopen("log.txt","w");
	int count;
	while (!(hsr_contrl.key_end==1&&hsr_contrl.release_contrl==1))
	{
//处理打印信息
		fd_set fdread = fd_all;
		ret = select(max_fd+1, &fdread, NULL, NULL,&tv);	//select函数
		if(ret>0)
		{
//			cout<<"aaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n"<<endl;
			// 有新数据
			if(FD_ISSET(pipe_fd,&fdread))
			{
	//			cout<<"bbbbbbbbbbbbbbbbbbbbb\n"<<endl;
				//pipe_fd 可读新数据流
				if(read(pipe_fd, (void *)&new_message, sizeof(new_message))>0)
				{
					//处理消息
//					fprintf(logfile,"%s",new_message.printbuff);
	//				cout<<"vcount"<<count<<endl;
					count++;
					if(count==50)
					{
					system("clear");
					printf("%s\n",new_message.printbuff);
					count=0;
                    			}
				}
			}
			//处理完双向的pipe管道后，开始处理单向上传数据的管道
			if(FD_ISSET(pipe_upload_fd,&fdread))
			{
//				cout<<"222222222222222222222222222222222222222"<<endl;
				if(read(pipe_upload_fd, (void *)&new_uploadInfo, sizeof(new_uploadInfo))>0)
				{
					HICDataPack *pPack = HICCreatePack();
					pPack->nHeader = 1;
					pPack->nDeviceName = 4;
					pPack->nDeviceType = 1;
					pPack->nDataType = 1;

					//自定义数据
					HICAddFloat(pPack, (float)new_uploadInfo.current_xyzw[0]);//x
					HICAddFloat(pPack, (float)new_uploadInfo.current_xyzw[1]);//y
					HICAddFloat(pPack, (float)new_uploadInfo.current_xyzw[3]);//st
					int nDataLength = 0;
					char *pBuf = (char *)HICPackToByte(pPack, &nDataLength);

					// TODO: 在此添加控件通知处理程序代码
					int sizeofsend=sendto(udp_socketfd, pBuf, nDataLength, 0,(struct sockaddr*)&servaddr,sizeof(servaddr));
//					cout<<"sizeof:"<<sizeofsend<<endl;
//					cout<<"socketfd"<<udp_socketfd<<endl;
					HICReleasePack(pPack);


					//////////////////////////////////////////////////////////////////////////
					//发送手臂的数据
					pPack = HICCreatePack();
					pPack->nHeader = 1;
					pPack->nDeviceName = 4;//机器人编号
					pPack->nDeviceType = 1;//设备类型
					pPack->nDataType = 4;//数据类型是4右臂，左臂是3。

					//自定义数据 4个角度
					for(int i = 0; i<4; i++)
					HICAddFloat(pPack, (float)new_uploadInfo.current_rst[i]);

					nDataLength = 0;
					pBuf = (char *)HICPackToByte(pPack, &nDataLength);

					// TODO: 在此添加控件通知处理程序代码
					sendto(udp_socketfd, pBuf, nDataLength, 0,(struct sockaddr*)&servaddr,sizeof(servaddr));
					HICReleasePack(pPack);
				}
			}
		}
	}
	fclose(logfile);
	//程序结束，关闭文件和管道
	printf("Log Thread is stop now !!release contrl==%d\n",hsr_contrl.release_contrl);
}
#define NUM 5
#if 1

float setpoint[][2]=
{
	{4.5f,7.1f},
	{1.5f,7.1f},
//	{2.0f,8.1f},
	{1.5f,5.1f},
//	{1.5f,4.5f},
//	{2.5f,5.1f},
//	{3.5f,5.1f},
//	{4.5f,5.1f},
	{5.5f,5.1f},
	{5.5f,7.1f},
};
#endif
void Add_Keyboard_Task(char keytask)//根据键盘指令生成task信息，并发送到pipe队列
{
	action_info tempaction={0};
	//生成新的task
#if 1
	switch(keytask)
	{
		//程序退出
	case 'q':
		tempaction.action_id=HALT;SENDCMD(tempaction);
		break;
	case 'e':
		tempaction.action_id=ENTER_ETA;SENDCMD(tempaction);
		break;
	case 'l':
		tempaction.action_id=ENTER_LINE;SENDCMD(tempaction);
		break;
	case 'k':
		tempaction.action_id=ENTER_KEY;SENDCMD(tempaction);
		break;
	case 'a':

		tempaction.action_id=LEFT;SENDCMD(tempaction);
		break;
	case 's':
//		key_end=0;
		tempaction.action_id=BACK;SENDCMD(tempaction);
		break;
	case 'd':
//		key_end=0;
		tempaction.action_id=RIGHT;SENDCMD(tempaction);
		break;
	case 'w':
//		key_end=0;
		tempaction.action_id=GO;SENDCMD(tempaction);
		break;
	case ' ':
//		key_end=0;
		tempaction.action_id=STOP;SENDCMD(tempaction);
		break;
	case 'm':
			tempaction.action_id=NEW_TARGET;
			tempaction.hand_num=1;
			tempaction.target_hand_st[0]=-0.3;
			tempaction.target_hand_st[1]=-0.3;
			tempaction.target_hand_st[2]=-0.3;
			tempaction.target_hand_st[3]=3.14;
		//	tempaction.target_hand[0][0]=-0.3024;tempaction.target_hand[0][1]=0.7339;tempaction.target_hand[0][2]=0.6082;tempaction.target_hand[0][3]=0.3229;
		//	tempaction.target_hand[1][0]=-0.77314;tempaction.target_hand[1][1]=-0.56206;tempaction.target_hand[1][2]=0.2938;tempaction.target_hand[1][3]=0.0964;
		//	tempaction.target_hand[2][0]=0.55748;tempaction.target_hand[2][1]=-0.3814;tempaction.target_hand[2][2]=0.7374;tempaction.target_hand[2][3]=0.44575;
		//	tempaction.target_hand[3][0]=0;tempaction.target_hand[3][1]=0;tempaction.target_hand[3][2]=0;tempaction.target_hand[3][3]=1;
			SENDCMD(tempaction);
		break;
	case 'h':

		tempaction.action_id=HOME;SENDCMD(tempaction);
		break;
	case 'x':

		tempaction.action_id=ENTER_HOME;SENDCMD(tempaction);
		break;
	case 'o':
		tempaction.action_id=SERVE_SWITCH;SENDCMD(tempaction);
		break;
	case 'c':
		tempaction.action_id=CLEAR_ENC;SENDCMD(tempaction);
		break;
	case '1':
		tempaction.action_id=CHANGE_JOINT;
		tempaction.hand_num=1;
		tempaction.joint_num=1;
		SENDCMD(tempaction);
		break;
		case '2':
		tempaction.action_id=CHANGE_JOINT;
		tempaction.hand_num=1;
		tempaction.joint_num=2;
		SENDCMD(tempaction);
		break;
		case '3':
		tempaction.action_id=CHANGE_JOINT;
		tempaction.hand_num=1;
		tempaction.joint_num=3;
		SENDCMD(tempaction);
		break;
	  case '4':
		tempaction.action_id=CHANGE_JOINT;
		tempaction.hand_num=1;
		tempaction.joint_num=4;
		SENDCMD(tempaction);
		break;
	case '5':
		tempaction.action_id=CHANGE_JOINT;
		tempaction.hand_num=2;
		tempaction.joint_num=1;
		SENDCMD(tempaction);
		break;
	case '6':
		tempaction.action_id=CHANGE_JOINT;
		tempaction.hand_num=2;
		tempaction.joint_num=2;
		SENDCMD(tempaction);
		break;
	case '7':
		tempaction.action_id=CHANGE_JOINT;
		tempaction.hand_num=2;
		tempaction.joint_num=3;
		SENDCMD(tempaction);
		break;
	  case '8':
		tempaction.action_id=CHANGE_JOINT;
		tempaction.hand_num=2;
		tempaction.joint_num=4;
		SENDCMD(tempaction);
		break;
	  case '-':
		tempaction.action_id=JOINT_NEGTIVE_MOVE;SENDCMD(tempaction);
		break;
	  case '=':
		tempaction.action_id=JOINT_POSITIVE_MOVE;SENDCMD(tempaction);
		break;
	}
#endif
}

void reset_terminal()
{
	tcsetattr(0,TCSANOW,&backup);
}
void disable_return()
{
	struct termios new_config;
	tcgetattr(0,&backup);
	new_config=backup;
	new_config.c_lflag &=~(ICANON|ECHO);
	tcsetattr(0,TCSANOW,&new_config);
	atexit(reset_terminal);
}
static struct termios ori_attr;
static struct termios cur_attr;
static __inline
int tty_reset(void)
{
        if (tcsetattr(STDIN_FILENO, TCSANOW, &ori_attr) != 0)
                return -1;

        return 0;
}
static __inline
int tty_set(void)
{

        if ( tcgetattr(STDIN_FILENO, &ori_attr) )
                return -1;

        memcpy(&cur_attr, &ori_attr, sizeof(cur_attr) );
        cur_attr.c_lflag &= ~ICANON;
//        cur_attr.c_lflag |= ECHO;
        cur_attr.c_lflag &= ~ECHO;
        cur_attr.c_cc[VMIN] = 1;
        cur_attr.c_cc[VTIME] = 0;

        if (tcsetattr(STDIN_FILENO, TCSANOW, &cur_attr) != 0)
                return -1;

        return 0;
}

static __inline
int keybhit(void)
{

        fd_set rfds;
        struct timeval tv;
        int retval;

        /* Watch stdin (fd 0) to see when it has input. */
        FD_ZERO(&rfds);
        FD_SET(0, &rfds);
        /* Wait up to five seconds. */
        tv.tv_sec  = 0;
        tv.tv_usec = 0;

        retval = select(1, &rfds, NULL, NULL, &tv);
        /* Don't rely on the value of tv now! */

        if (retval == -1) {
                perror("select()");
                return 0;
        } else if (retval)
                return 1;
        /* FD_ISSET(0, &rfds) will be true. */
        else
                return 0;
        return 0;
}
void *Process_Keyboard_Callback(void *)//处理键盘指令的回调函数
{
	//循环处理程序I/O
	char keycmd='`';
	int count=0;
//	disable_return();
//	int tty_set_flag;
//	tty_set_flag = tty_set();


	while(!hsr_contrl.key_end)
	{
		//读取键盘指令，并发送到pipe管道
		int tty_set_flag;
		tty_set_flag=tty_set();
		if(keybhit())
		{
			keycmd = getchar();
		}
		else
			keycmd='`';
		if(tty_set_flag == 0)
			tty_reset();

		//根据键盘指令处理任务
		if (keycmd!='`')
		{
//			printf("key==%c\n",keycmd);
			Add_Keyboard_Task(keycmd);
		}
//		sleep(10);
	}
	//线程结束;
	printf("Keyboard Thread is stop now !!,release contrl==%d\n",hsr_contrl.release_contrl);
	hsr_contrl.release_contrl++;

}

