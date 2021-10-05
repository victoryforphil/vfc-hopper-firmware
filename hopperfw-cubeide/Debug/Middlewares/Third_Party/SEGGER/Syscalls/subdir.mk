################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.c \
../Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.c \
../Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.c \
../Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.c 

OBJS += \
./Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.o \
./Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.o \
./Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.o \
./Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.o 

C_DEPS += \
./Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.d \
./Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.d \
./Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.d \
./Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.o: ../Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Middlewares/Third_Party/MavLink/common -I../Middlewares/Third_Party/SEGGER -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.o: ../Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Middlewares/Third_Party/MavLink/common -I../Middlewares/Third_Party/SEGGER -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.o: ../Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Middlewares/Third_Party/MavLink/common -I../Middlewares/Third_Party/SEGGER -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.o: ../Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Middlewares/Third_Party/MavLink/common -I../Middlewares/Third_Party/SEGGER -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

