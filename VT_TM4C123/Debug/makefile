################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./tm4c123gh6pm_startup_ccs.obj" \
"./FreeRTOS/CLI/FreeRTOS_CLI.obj" \
"./FreeRTOS/Kernal/source/croutine.obj" \
"./FreeRTOS/Kernal/source/event_groups.obj" \
"./FreeRTOS/Kernal/source/list.obj" \
"./FreeRTOS/Kernal/source/queue.obj" \
"./FreeRTOS/Kernal/source/stream_buffer.obj" \
"./FreeRTOS/Kernal/source/tasks.obj" \
"./FreeRTOS/Kernal/source/timers.obj" \
"./FreeRTOS/Kernal/source/portable/CCS/ARC_CM4F/port.obj" \
"./FreeRTOS/Kernal/source/portable/CCS/ARC_CM4F/portasm.obj" \
"./FreeRTOS/Kernal/source/portable/MemMang/heap_4.obj" \
"./src/Core/main.obj" \
"./src/Core/system.obj" \
"./src/Core/system_init_offchip.obj" \
"./src/Core/system_init_onchip.obj" \
"./src/Middleware/OrionBMS.obj" \
"./src/Tasks/blink_task.obj" \
"./src/Tasks/fiddle_task.obj" \
"../tm4c123gh6pm.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \
-l"C:/ti/TivaWare_C_Series-2.2.0.295/driverlib/ccs/Release/driverlib.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include FreeRTOS/CLI/subdir_vars.mk
-include FreeRTOS/Kernal/source/subdir_vars.mk
-include FreeRTOS/Kernal/source/portable/CCS/ARC_CM4F/subdir_vars.mk
-include FreeRTOS/Kernal/source/portable/MemMang/subdir_vars.mk
-include src/Core/subdir_vars.mk
-include src/Middleware/subdir_vars.mk
-include src/Tasks/subdir_vars.mk
-include subdir_rules.mk
-include FreeRTOS/CLI/subdir_rules.mk
-include FreeRTOS/Kernal/source/subdir_rules.mk
-include FreeRTOS/Kernal/source/portable/CCS/ARC_CM4F/subdir_rules.mk
-include FreeRTOS/Kernal/source/portable/MemMang/subdir_rules.mk
-include src/Core/subdir_rules.mk
-include src/Middleware/subdir_rules.mk
-include src/Tasks/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
VT_TM4C123.out 

EXE_OUTPUTS__QUOTED += \
"VT_TM4C123.out" 


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "VT_TM4C123.out"

dependents:
	-cd C:/ti/TivaWare_C_Series-2.2.0.295/driverlib/ccs/Release && $(MAKE) all

# Tool invocations
VT_TM4C123.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS) C:/ti/TivaWare_C_Series-2.2.0.295/driverlib/ccs/Release/driverlib.lib
	@echo 'Building target: "$@"'
	@echo 'Invoking: Arm Linker'
	"C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --define=ccs="ccs" --define=TARGET_IS_TM4C129_RA1 --define=PART_TM4C123GH6PM -g --c11 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi -z -m"VT_TM4C123.map" --heap_size=0 --stack_size=7000 -i"C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/lib" -i"C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="VT_TM4C123_linkInfo.xml" --rom_model -o "VT_TM4C123.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "tm4c123gh6pm_startup_ccs.obj" "FreeRTOS\CLI\FreeRTOS_CLI.obj" "FreeRTOS\Kernal\source\croutine.obj" "FreeRTOS\Kernal\source\event_groups.obj" "FreeRTOS\Kernal\source\list.obj" "FreeRTOS\Kernal\source\queue.obj" "FreeRTOS\Kernal\source\stream_buffer.obj" "FreeRTOS\Kernal\source\tasks.obj" "FreeRTOS\Kernal\source\timers.obj" "FreeRTOS\Kernal\source\portable\CCS\ARC_CM4F\port.obj" "FreeRTOS\Kernal\source\portable\CCS\ARC_CM4F\portasm.obj" "FreeRTOS\Kernal\source\portable\MemMang\heap_4.obj" "src\Core\main.obj" "src\Core\system.obj" "src\Core\system_init_offchip.obj" "src\Core\system_init_onchip.obj" "src\Middleware\OrionBMS.obj" "src\Tasks\blink_task.obj" "src\Tasks\fiddle_task.obj" 
	-$(RM) "tm4c123gh6pm_startup_ccs.d" "FreeRTOS\CLI\FreeRTOS_CLI.d" "FreeRTOS\Kernal\source\croutine.d" "FreeRTOS\Kernal\source\event_groups.d" "FreeRTOS\Kernal\source\list.d" "FreeRTOS\Kernal\source\queue.d" "FreeRTOS\Kernal\source\stream_buffer.d" "FreeRTOS\Kernal\source\tasks.d" "FreeRTOS\Kernal\source\timers.d" "FreeRTOS\Kernal\source\portable\CCS\ARC_CM4F\port.d" "FreeRTOS\Kernal\source\portable\MemMang\heap_4.d" 
	-$(RM) "src\Core\main.d" "src\Core\system.d" "src\Core\system_init_offchip.d" "src\Core\system_init_onchip.d" "src\Middleware\OrionBMS.d" "src\Tasks\blink_task.d" "src\Tasks\fiddle_task.d" 
	-$(RM) "FreeRTOS\Kernal\source\portable\CCS\ARC_CM4F\portasm.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:
C:/ti/TivaWare_C_Series-2.2.0.295/driverlib/ccs/Release/driverlib.lib:

-include ../makefile.targets

