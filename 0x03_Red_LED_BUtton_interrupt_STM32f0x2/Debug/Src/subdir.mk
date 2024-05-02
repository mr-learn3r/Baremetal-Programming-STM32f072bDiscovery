################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/Red_LED_Toggle_Button_Interrupt.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/Red_LED_Toggle_Button_Interrupt.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/Red_LED_Toggle_Button_Interrupt.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F0 -DSTM32F072RBTx -DSTM32F072B_DISCO -c -I../Inc -I"/home/mr_learner/Baremetal programming/0x03_Red_LED_BUtton_interrupt_STM32f0x2/driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/Red_LED_Toggle_Button_Interrupt.cyclo ./Src/Red_LED_Toggle_Button_Interrupt.d ./Src/Red_LED_Toggle_Button_Interrupt.o ./Src/Red_LED_Toggle_Button_Interrupt.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

