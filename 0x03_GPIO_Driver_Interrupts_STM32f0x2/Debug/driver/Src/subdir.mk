################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../driver/Src/GPIO_drv.c 

OBJS += \
./driver/Src/GPIO_drv.o 

C_DEPS += \
./driver/Src/GPIO_drv.d 


# Each subdirectory must supply rules for building sources it contributes
driver/Src/%.o driver/Src/%.su driver/Src/%.cyclo: ../driver/Src/%.c driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F0 -DSTM32F072RBTx -DSTM32F072B_DISCO -c -I"/home/mr_learner/Baremetal programming/0x02_GPIO_Driver_STM32f0x2/driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-driver-2f-Src

clean-driver-2f-Src:
	-$(RM) ./driver/Src/GPIO_drv.cyclo ./driver/Src/GPIO_drv.d ./driver/Src/GPIO_drv.o ./driver/Src/GPIO_drv.su

.PHONY: clean-driver-2f-Src

