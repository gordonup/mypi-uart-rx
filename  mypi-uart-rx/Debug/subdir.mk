################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../main_UART.c \
../uart.c 

OBJS += \
./main_UART.o \
./uart.o 

C_DEPS += \
./main_UART.d \
./uart.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C Compiler'
	arm-unknown-linux-gnueabi-g++ -I"D:\cygwin\opt\cross\x-tools\arm-unknown-linux-gnueabi\arm-unknown-linux-gnueabi\sysroot\usr\include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


