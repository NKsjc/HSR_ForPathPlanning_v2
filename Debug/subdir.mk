################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../App_Setting.cpp \
../CarControl.cpp \
../EtaTrajectoryPlan.cpp \
../IPC.cpp \
../InCircleTrajectoryPlan.cpp \
../KeyTrajectoryPlan.cpp \
../LogFactory.cpp \
../MainProcess.cpp \
../P2PTrajectoryPlan.cpp \
../PeakCan.cpp \
../RigidHandControl.cpp \
../SoftHandControl.cpp 

OBJS += \
./App_Setting.o \
./CarControl.o \
./EtaTrajectoryPlan.o \
./IPC.o \
./InCircleTrajectoryPlan.o \
./KeyTrajectoryPlan.o \
./LogFactory.o \
./MainProcess.o \
./P2PTrajectoryPlan.o \
./PeakCan.o \
./RigidHandControl.o \
./SoftHandControl.o 

CPP_DEPS += \
./App_Setting.d \
./CarControl.d \
./EtaTrajectoryPlan.d \
./IPC.d \
./InCircleTrajectoryPlan.d \
./KeyTrajectoryPlan.d \
./LogFactory.d \
./MainProcess.d \
./P2PTrajectoryPlan.d \
./PeakCan.d \
./RigidHandControl.d \
./SoftHandControl.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include/xenomai -I/usr/include/ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


