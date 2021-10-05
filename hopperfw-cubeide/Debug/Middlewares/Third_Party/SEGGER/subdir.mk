################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/SEGGER/SEGGER_RTT.c \
../Middlewares/Third_Party/SEGGER/SEGGER_RTT_printf.c \
../Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW.c \
../Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW_Config_FreeRTOS.c \
../Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW_FreeRTOS.c 

S_UPPER_SRCS += \
../Middlewares/Third_Party/SEGGER/SEGGER_RTT_ASM_ARMv7M.S 

OBJS += \
./Middlewares/Third_Party/SEGGER/SEGGER_RTT.o \
./Middlewares/Third_Party/SEGGER/SEGGER_RTT_ASM_ARMv7M.o \
./Middlewares/Third_Party/SEGGER/SEGGER_RTT_printf.o \
./Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW.o \
./Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW_Config_FreeRTOS.o \
./Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW_FreeRTOS.o 

S_UPPER_DEPS += \
./Middlewares/Third_Party/SEGGER/SEGGER_RTT_ASM_ARMv7M.d 

C_DEPS += \
./Middlewares/Third_Party/SEGGER/SEGGER_RTT.d \
./Middlewares/Third_Party/SEGGER/SEGGER_RTT_printf.d \
./Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW.d \
./Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW_Config_FreeRTOS.d \
./Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW_FreeRTOS.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/SEGGER/SEGGER_RTT.o: ../Middlewares/Third_Party/SEGGER/SEGGER_RTT.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Middlewares/Third_Party/MavLink/common -I../Middlewares/Third_Party/SEGGER -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/SEGGER/SEGGER_RTT.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/SEGGER/SEGGER_RTT_ASM_ARMv7M.o: ../Middlewares/Third_Party/SEGGER/SEGGER_RTT_ASM_ARMv7M.S
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Middlewares/Third_Party/SEGGER/SEGGER_RTT_ASM_ARMv7M.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
Middlewares/Third_Party/SEGGER/SEGGER_RTT_printf.o: ../Middlewares/Third_Party/SEGGER/SEGGER_RTT_printf.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Middlewares/Third_Party/MavLink/common -I../Middlewares/Third_Party/SEGGER -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/SEGGER/SEGGER_RTT_printf.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW.o: ../Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Middlewares/Third_Party/MavLink/common -I../Middlewares/Third_Party/SEGGER -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW_Config_FreeRTOS.o: ../Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW_Config_FreeRTOS.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Middlewares/Third_Party/MavLink/common -I../Middlewares/Third_Party/SEGGER -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW_Config_FreeRTOS.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW_FreeRTOS.o: ../Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW_FreeRTOS.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Middlewares/Third_Party/MavLink/common -I../Middlewares/Third_Party/SEGGER -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/SEGGER/SEGGER_SYSVIEW_FreeRTOS.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

