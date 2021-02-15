################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
src_device/%.obj: ../src_device/%.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1011/ccs/tools/compiler/ti-cgt-c2000_20.2.4.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu0 -O2 --fp_mode=relaxed --include_path="D:/FOC_Motor_Controller/motor-controller/Software/TI_ESC_V2/driverlib" --include_path="D:/FOC_Motor_Controller/motor-controller/Software/TI_ESC_V2/headers" --include_path="D:/FOC_Motor_Controller/motor-controller/Software/TI_ESC_V2" --include_path="C:/ti/ccs1011/ccs/tools/compiler/ti-cgt-c2000_20.2.4.LTS/include" --define=_INLINE --define=_FLASH --define=_BOOSTXL_8320RS_REVA_ --define=DRV8320_SPI --define=_DATALOG_EN_ --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=coffabi --preproc_with_compile --preproc_dependency="src_device/$(basename $(<F)).d_raw" --obj_directory="src_device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src_device/%.obj: ../src_device/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1011/ccs/tools/compiler/ti-cgt-c2000_20.2.4.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu0 -O2 --fp_mode=relaxed --include_path="D:/FOC_Motor_Controller/motor-controller/Software/TI_ESC_V2/driverlib" --include_path="D:/FOC_Motor_Controller/motor-controller/Software/TI_ESC_V2/headers" --include_path="D:/FOC_Motor_Controller/motor-controller/Software/TI_ESC_V2" --include_path="C:/ti/ccs1011/ccs/tools/compiler/ti-cgt-c2000_20.2.4.LTS/include" --define=_INLINE --define=_FLASH --define=_BOOSTXL_8320RS_REVA_ --define=DRV8320_SPI --define=_DATALOG_EN_ --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=coffabi --preproc_with_compile --preproc_dependency="src_device/$(basename $(<F)).d_raw" --obj_directory="src_device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


