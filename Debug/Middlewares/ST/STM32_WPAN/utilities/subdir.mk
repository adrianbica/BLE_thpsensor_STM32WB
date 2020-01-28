################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_WPAN/utilities/dbg_trace.c \
../Middlewares/ST/STM32_WPAN/utilities/otp.c \
../Middlewares/ST/STM32_WPAN/utilities/stm_list.c \
../Middlewares/ST/STM32_WPAN/utilities/stm_queue.c 

OBJS += \
./Middlewares/ST/STM32_WPAN/utilities/dbg_trace.o \
./Middlewares/ST/STM32_WPAN/utilities/otp.o \
./Middlewares/ST/STM32_WPAN/utilities/stm_list.o \
./Middlewares/ST/STM32_WPAN/utilities/stm_queue.o 

C_DEPS += \
./Middlewares/ST/STM32_WPAN/utilities/dbg_trace.d \
./Middlewares/ST/STM32_WPAN/utilities/otp.d \
./Middlewares/ST/STM32_WPAN/utilities/stm_list.d \
./Middlewares/ST/STM32_WPAN/utilities/stm_queue.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_WPAN/utilities/dbg_trace.o: ../Middlewares/ST/STM32_WPAN/utilities/dbg_trace.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../tiny_lpm -I../Middlewares/ST/STM32_WPAN/ble/core -I../Utilities/lpm/tiny_lpm -I../STM32_WPAN/App -I../Middlewares/ST/STM32_WPAN/ble -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Core/Inc -I../Utilities/sequencer -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/STM32_WPAN/utilities/dbg_trace.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/STM32_WPAN/utilities/otp.o: ../Middlewares/ST/STM32_WPAN/utilities/otp.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../tiny_lpm -I../Middlewares/ST/STM32_WPAN/ble/core -I../Utilities/lpm/tiny_lpm -I../STM32_WPAN/App -I../Middlewares/ST/STM32_WPAN/ble -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Core/Inc -I../Utilities/sequencer -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/STM32_WPAN/utilities/otp.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/STM32_WPAN/utilities/stm_list.o: ../Middlewares/ST/STM32_WPAN/utilities/stm_list.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../tiny_lpm -I../Middlewares/ST/STM32_WPAN/ble/core -I../Utilities/lpm/tiny_lpm -I../STM32_WPAN/App -I../Middlewares/ST/STM32_WPAN/ble -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Core/Inc -I../Utilities/sequencer -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/STM32_WPAN/utilities/stm_list.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/STM32_WPAN/utilities/stm_queue.o: ../Middlewares/ST/STM32_WPAN/utilities/stm_queue.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../tiny_lpm -I../Middlewares/ST/STM32_WPAN/ble/core -I../Utilities/lpm/tiny_lpm -I../STM32_WPAN/App -I../Middlewares/ST/STM32_WPAN/ble -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Core/Inc -I../Utilities/sequencer -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/STM32_WPAN/utilities/stm_queue.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

