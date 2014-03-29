################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ArrayList.cpp \
../Ball_detect.cpp \
../Circle.cpp \
../ball_detection.cpp \
../main.cpp 

OBJS += \
./ArrayList.o \
./Ball_detect.o \
./Circle.o \
./ball_detection.o \
./main.o 

CPP_DEPS += \
./ArrayList.d \
./Ball_detect.d \
./Circle.d \
./ball_detection.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/opencv -I/usr/local/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


