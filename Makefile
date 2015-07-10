#****************************************************************************
# Copyright (C) 2001-2006  PEAK System-Technik GmbH
#
# linux@peak-system.cppom
# www.peak-system.cppom
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#
# Maintainer(s): Klaus Hitschler (klaus.hitschler@gmx.de)
#****************************************************************************

#****************************************************************************
#
# Makefile - Makefile for receivetest and transmittest programs
#
# $Id: Makefile 518 2007-08-08 07:40:31Z edouard $
#
#****************************************************************************

SRC     =
INC     = -I. -I../lib -I../driver
RT      = XENOMAI
ifeq ($(RT), XENOMAI)
SKIN = xeno
### Xenomai directory, xeno-config and library directory ###########
RT_DIR          ?= /usr/xenomai
RT_CONFIG       ?= $(RT_DIR)/bin/xeno-config
RT_LIB_DIR      ?= $(shell $(RT_CONFIG) --library-dir) -Wl,-rpath $(shell $(RT_CONFIG) --library-dir) -L. -g
### User space application compile options #########################

USERAPP_LIBS      ?= -lnative -lpcan -lm -lpthread
USERAPP_LDFLAGS   ?= $(shell $(RT_CONFIG) --$(SKIN)-ldflags) -L$(RT_LIB_DIR)
USERAPP_CFLAGS    ?= $(shell $(RT_CONFIG) --$(SKIN)-cflags)
endif

ifeq ($(RT), TEST)
SKIN = xeno
### Xenomai directory, xeno-config and library directory ###########
RT_DIR          ?= /usr
RT_CONFIG       ?= $(RT_DIR)/bin/xeno-config
RT_LIB_DIR      ?= $(shell $(RT_CONFIG) --library-dir) -Wl,-rpath $(shell $(RT_CONFIG) --library-dir)
### User space application compile options #########################
USERAPP_LIBS      ?= -lnative -lpcan -lm -lpthread
USERAPP_LDFLAGS   ?= $(shell $(RT_CONFIG) --$(SKIN)-ldflags) -L$(RT_LIB_DIR)
USERAPP_CFLAGS    ?= $(shell $(RT_CONFIG) --$(SKIN)-cflags)
endif


ifeq ($(RT), NO_RT)
  USERAPP_LIBS = -lpcan
endif

ifeq ($(HOSTTYPE),x86_64)
  LDLIBS  = -L../lib -L/lib64 -L/usr/lib64 -L/usr/local/lib64
else
  LDLIBS  = -L../lib -L/lib -L/usr/lib -L/usr/local/lib
endif

ifneq ($(RT), NO_RT)
DBGFLAGS   = -g
else
DBGFLAGS   = -g
endif

ifeq ($(DBG), DEBUG)
CFLAGS  = $(DBGFLAGS) $(INC) $(LDLIBS)
else
CFLAGS  = $(INC) $(LDLIBS)
endif


HSR:MainProcess.cpp IPC.cpp ProjectCommon.h CarControl.h IPC.h PeakCan.h  DataProtocol.h BezierTrajectory.o PVUControl.o App_Setting.o LogFactory.cpp LogFactory.h CarControl.o RigidHandControl.o SoftHandControl.o  EMGTrajectoryPlan.o EtaTrajectoryPlan.o InCircleTrajectoryPlan.o KeyTrajectoryPlan.o P2PTrajectoryPlan.o PeakCan.o  User_IO
	g++ MainProcess.cpp IPC.cpp DataProtocol.h  ProjectCommon.h BezierTrajectory.o App_Setting.o PVUControl.o LogFactory.cpp LogFactory.h CarControl.o RigidHandControl.o SoftHandControl.o  EMGTrajectoryPlan.o EtaTrajectoryPlan.o InCircleTrajectoryPlan.o KeyTrajectoryPlan.o P2PTrajectoryPlan.o PeakCan.o -o HSR $(CFLAGS) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -D$(RT) 

BezierTrajectory:BezierTrajectory.cpp BezierTrajectory.h
	g++ BezierTrajectory.cpp BezierTrajectory.h -o BezierTrajectory.o $(CFLAGS) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -D$(RT)

EtaTrajectoryPlan:EtaTrajectoryPlan.cpp EtaTrajectoryPlan.h
	g++ EtaTrajectoryPlan.cpp EtaTrajectoryPlan.h -o EtaTrajectoryPlan.o $(CFLAGS) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -D$(RT)

EtaTrajectoryPlan:EtaTrajectoryPlan.cpp EtaTrajectoryPlan.h
	g++ EtaTrajectoryPlan.cpp EtaTrajectoryPlan.h -o EtaTrajectoryPlan.o $(CFLAGS) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -D$(RT)

InCircleTrajectoryPlan:InCircleTrajectoryPlan.cpp InCircleTrajectoryPlan.h
	g++ InCircleTrajectoryPlan.cpp InCircleTrajectoryPlan.h -o InCircleTrajectoryPlan.o $(CFLAGS) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -D$(RT)

KeyTrajectoryPlan:KeyTrajectoryPlan.cpp KeyTrajectoryPlan.h
	g++ KeyTrajectoryPlan.cpp KeyTrajectoryPlan.h -o KeyTrajectoryPlan.o $(CFLAGS) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -D$(RT)
	
P2PTrajectoryPlan:P2PTrajectoryPlan.cpp P2PTrajectoryPlan.h
	g++ P2PTrajectoryPlan.cpp P2PTrajectoryPlan.h -o P2PTrajectoryPlan.o $(CFLAGS) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -D$(RT)
	
SoftHandControl:SoftHandControl.cpp SoftHandControl.h ProjectCommon.h
	g++ SoftHandControl.cpp SoftHandControl.h -o SoftHandControl.o $(CFLAGS) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -D$(RT)
			
RigidHandControl:RigidHandControl.cpp RigidHandControl.h ProjectCommon.h  PeakCan.h
	g++ RigidHandControl.cpp RigidHandControl.h  -o RigidHandControl.o $(CFLAGS) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -D$(RT)		
	
CarControl:CarControl.cpp CarControl.h ProjectCommon.h PeakCan.h
	g++ CarControl.cpp CarControl.h  -o CarControl.o $(CFLAGS) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -D$(RT)
	
PVUControl:PVUControl.cpp PVUControl.h ProjectCommon.h PeakCan.h
	g++ PVUControl.cpp PVUControl.h  -o PVUControl.o $(CFLAGS) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -D$(RT)

PeakCan:PeakCan.cpp PeakCan.h
	g++ PeakCan.cpp PeakCan.h.h -o PeakCan.o $(CFLAGS) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -D$(RT)

App_Setting:App_Setting.cpp App_Setting.h
	g++ App_Setting.cpp App_Setting.h -o App_Setting.o $(CFLAGS) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -D$(RT)
		

	
User_IO:UserInt.cpp QR_Recv.cpp  ProjectCommon.h  HICDataHelper.cpp HICDataHelper.h 
	g++  UserInt.cpp QR_Recv.h  QR_Recv.cpp HICDataHelper.cpp  ProjectCommon.h -o User_IO -lpthread -lm -lX11 -L. -I. -w	
clean:
	rm *.o
	rm HSR 
	rm User_IO
