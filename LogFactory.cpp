/*
 * LogFactory.cpp
 *
 *  Created on: 2014-9-17
 *      Author: alvin
 */

#include "LogFactory.h"
#include<stdio.h>
#include <stdarg.h>
#include <native/timer.h>
RTIME now;
RTIME begin;
LogFactory::LogFactory(string level,string path) {
	begin=rt_timer_read();
	// TODO Auto-generated constructor stub
	if(string("_Debug")==level){
		this->loglevel=_Debug;
	}else if(string("_Info")==level){
		this->loglevel=_Info;
	}
	else if(string("_Warning")==level){
		this->loglevel=_Warning;
	}
	else if(string("_Error")==level){
		this->loglevel=_Error;
	}else{
		throw "Invalid log level";
	}
	this->logpath=path;
	this->InitLogFile();
}

LogFactory::~LogFactory() {
	// TODO Auto-generated destructor stub
	fclose(this->logfd);
}
void LogFactory::Print(LOGLEVEL level,const char * format,...){
	now=rt_timer_read();
	if(this->loglevel>level){
		return;
	}
	va_list argptr;
	int cnt;
	va_start(argptr, format);
	if(_Save==level){
		fprintf(this->logfd,"%u.%-6u   ",(unsigned)((now-begin)/1000000),(unsigned)((now-begin)%1000000));
		vfprintf(this->logfd,format,argptr);
	}else{
		printf("[[[  Log At Time:%u.%-6u ms  ]]]   ",(unsigned)((now-begin)/1000000),(unsigned)((now-begin)%1000000));
		cnt = vprintf(format, argptr);
	}
	va_end(argptr);
	return;
}

void LogFactory::InitLogFile()
{
	char tm_buf[100] = {0};
	time_t t = time(NULL);
	strftime(tm_buf, 99, ".\log\LogFile_%Y_%m_%d_%H%M%S.txt", localtime(&t));
	logfd=fopen(tm_buf,"w");
}


