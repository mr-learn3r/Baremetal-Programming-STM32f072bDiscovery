################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/RGB_Led_Toggle.c 

OBJS += \
./Src/RGB_Led_Toggle.o 

C_DEPS += \
./Src/RGB_Led_Toggle.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F0 -DSTM32F072RBTx -DSTM32F072B_DISCO -c -I"/home/mr_learner/Baremetal programming/0x02_GPIO_Driver_STM32f0x2/driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/RGB_Led_Toggle.cyclo ./Src/RGB_Led_Toggle.d ./Src/RGB_Led_Toggle.o ./Src/RGB_Led_Toggle.su

.PHONY: clean-Src

