################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
src/Middleware/%.obj: ../src/Middleware/%.cpp $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/Users/FSAE/Desktop/dev/BeeMS/VT_TM4C123" --include_path="C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --include_path="C:/Users/FSAE/Desktop/dev/BeeMS/VT_TM4C123" --include_path="C:/Users/FSAE/Desktop/dev/BeeMS/VT_TM4C123/src" --include_path="C:/Users/FSAE/Desktop/dev/BeeMS/VT_TM4C123/FreeRTOS/Config" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/driverlib/ccs" --include_path="C:/ti/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel" --include_path="C:/ti/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/include" --include_path="C:/ti/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/CCS/ARM_CM4F" --include_path="C:/ti/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Plus-CLI" --define=ccs="ccs" --define=TARGET_IS_TM4C129_RA1 --define=PART_TM4C123GH6PM -g --c11 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="src/Middleware/$(basename $(<F)).d_raw" --obj_directory="src/Middleware" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


