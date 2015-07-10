/*
 * LogFactory.h
 *
 *  Created on: 2014-9-17
 *      Author: alvin
 */

#ifndef LOGFACTORY_H_
#define LOGFACTORY_H_
#include<stdio.h>
#include <string>

using namespace std;
enum LOGLEVEL{
	_Debug,
	_Info,
	_Warning,
	_Error,
	_Save
};
class LogFactory {
private:
	LOGLEVEL loglevel;
	string logpath;
	FILE * logfd;

public:
	LogFactory(string level,string path);
	virtual ~LogFactory();
	void Print(LOGLEVEL level,const char * format,...);
	void InitLogFile();
};

#endif /* LOGFACTORY_H_ */
