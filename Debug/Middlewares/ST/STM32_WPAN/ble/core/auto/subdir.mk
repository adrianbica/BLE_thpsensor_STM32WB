################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gap_aci.c \
../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gatt_aci.c \
../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_hal_aci.c \
../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_hci_le.c \
../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_l2cap_aci.c 

OBJS += \
./Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gap_aci.o \
./Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gatt_aci.o \
./Middlewares/ST/STM32_WPAN/ble/core/auto/ble_hal_aci.o \
./Middlewares/ST/STM32_WPAN/ble/core/auto/ble_hci_le.o \
./Middlewares/ST/STM32_WPAN/ble/core/auto/ble_l2cap_aci.o 

C_DEPS += \
./Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gap_aci.d \
./Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gatt_aci.d \
./Middlewares/ST/STM32_WPAN/ble/core/auto/ble_hal_aci.d \
./Middlewares/ST/STM32_WPAN/ble/core/auto/ble_hci_le.d \
./Middlewares/ST/STM32_WPAN/ble/core/auto/ble_l2cap_aci.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gap_aci.o: ../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gap_aci.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../tiny_lpm -I../Middlewares/ST/STM32_WPAN/ble/core -I../Utilities/lpm/tiny_lpm -I../STM32_WPAN/App -I../Middlewares/ST/STM32_WPAN/ble -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Core/Inc -I../Utilities/sequencer -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gap_aci.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gatt_aci.o: ../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gatt_aci.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../tiny_lpm -I../Middlewares/ST/STM32_WPAN/ble/core -I../Utilities/lpm/tiny_lpm -I../STM32_WPAN/App -I../Middlewares/ST/STM32_WPAN/ble -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Core/Inc -I../Utilities/sequencer -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gatt_aci.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/STM32_WPAN/ble/core/auto/ble_hal_aci.o: ../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_hal_aci.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../tiny_lpm -I../Middlewares/ST/STM32_WPAN/ble/core -I../Utilities/lpm/tiny_lpm -I../STM32_WPAN/App -I../Middlewares/ST/STM32_WPAN/ble -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Core/Inc -I../Utilities/sequencer -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/STM32_WPAN/ble/core/auto/ble_hal_aci.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/STM32_WPAN/ble/core/auto/ble_hci_le.o: ../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_hci_le.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../tiny_lpm -I../Middlewares/ST/STM32_WPAN/ble/core -I../Utilities/lpm/tiny_lpm -I../STM32_WPAN/App -I../Middlewares/ST/STM32_WPAN/ble -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Core/Inc -I../Utilities/sequencer -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/STM32_WPAN/ble/core/auto/ble_hci_le.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/STM32_WPAN/ble/core/auto/ble_l2cap_aci.o: ../Middlewares/ST/STM32_WPAN/ble/core/auto/ble_l2cap_aci.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx -c -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../tiny_lpm -I../Middlewares/ST/STM32_WPAN/ble/core -I../Utilities/lpm/tiny_lpm -I../STM32_WPAN/App -I../Middlewares/ST/STM32_WPAN/ble -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Core/Inc -I../Utilities/sequencer -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/ST/STM32_WPAN/ble/core/auto/ble_l2cap_aci.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

