################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/freertos.c \
../src/libjpeg.c \
../src/main.c \
../src/stm32f4xx_hal_msp.c \
../src/stm32f4xx_hal_timebase_TIM.c \
../src/stm32f4xx_it.c \
../src/usb_device.c \
../src/usbd_cdc_if.c \
../src/usbd_conf.c \
../src/usbd_desc.c 

OBJS += \
./src/freertos.o \
./src/libjpeg.o \
./src/main.o \
./src/stm32f4xx_hal_msp.o \
./src/stm32f4xx_hal_timebase_TIM.o \
./src/stm32f4xx_it.o \
./src/usb_device.o \
./src/usbd_cdc_if.o \
./src/usbd_conf.o \
./src/usbd_desc.o 

C_DEPS += \
./src/freertos.d \
./src/libjpeg.d \
./src/main.d \
./src/stm32f4xx_hal_msp.d \
./src/stm32f4xx_hal_timebase_TIM.d \
./src/stm32f4xx_it.d \
./src/usb_device.d \
./src/usbd_cdc_if.d \
./src/usbd_conf.d \
./src/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DTRACE -DSTM32F407xx -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4xx" -I"../system/include/cmsis/device" -I"../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"../Middlewares/Third_Party/LibJPEG/include" -I"../Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"../Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Inc" -I"../Middlewares/Third_Party/FreeRTOS/Source/include" -I"../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


