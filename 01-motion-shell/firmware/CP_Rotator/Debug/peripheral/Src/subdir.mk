################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../peripheral/Src/can.cpp \
../peripheral/Src/motors.cpp \
../peripheral/Src/rotator.cpp \
../peripheral/Src/stepper_setting.cpp 

OBJS += \
./peripheral/Src/can.o \
./peripheral/Src/motors.o \
./peripheral/Src/rotator.o \
./peripheral/Src/stepper_setting.o 

CPP_DEPS += \
./peripheral/Src/can.d \
./peripheral/Src/motors.d \
./peripheral/Src/rotator.d \
./peripheral/Src/stepper_setting.d 


# Each subdirectory must supply rules for building sources it contributes
peripheral/Src/%.o peripheral/Src/%.su peripheral/Src/%.cyclo: ../peripheral/Src/%.cpp peripheral/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"/Users/htl/STM32CubeIDE/workspace_1.15.1/CP_Rotator/peripheral/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Oz -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-peripheral-2f-Src

clean-peripheral-2f-Src:
	-$(RM) ./peripheral/Src/can.cyclo ./peripheral/Src/can.d ./peripheral/Src/can.o ./peripheral/Src/can.su ./peripheral/Src/motors.cyclo ./peripheral/Src/motors.d ./peripheral/Src/motors.o ./peripheral/Src/motors.su ./peripheral/Src/rotator.cyclo ./peripheral/Src/rotator.d ./peripheral/Src/rotator.o ./peripheral/Src/rotator.su ./peripheral/Src/stepper_setting.cyclo ./peripheral/Src/stepper_setting.d ./peripheral/Src/stepper_setting.o ./peripheral/Src/stepper_setting.su

.PHONY: clean-peripheral-2f-Src

