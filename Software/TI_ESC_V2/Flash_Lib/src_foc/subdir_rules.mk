################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
src_foc/%.obj: ../src_foc/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1011/ccs/tools/compiler/ti-cgt-c2000_20.2.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu0 -O2 --fp_mode=relaxed --include_path="D:/FOC Motor Controller/motor-controller/Software/TI_ESC_V2" --include_path="/libraries/control/ctrl/include" --include_path="/libraries/control/pi/include" --include_path="/libraries/control/vsf/include" --include_path="/libraries/control/fwc/include" --include_path="/libraries/control/mtpa/include" --include_path="/libraries/control/vs_freq/include" --include_path="/libraries/filter/filter_fo/include" --include_path="/libraries/filter/filter_so/include" --include_path="/libraries/filter/offset/include" --include_path="/libraries/observers/est/include" --include_path="/libraries/transforms/clarke/include" --include_path="/libraries/transforms/ipark/include" --include_path="/libraries/transforms/park/include" --include_path="/libraries/transforms/svgen/include" --include_path="/libraries/utilities/angle_gen/include" --include_path="/libraries/utilities/cpu_time/include" --include_path="/libraries/utilities/datalog/include" --include_path="/libraries/utilities/diagnostic/include" --include_path="/libraries/utilities/traj/include" --include_path="/libraries/utilities/types/include" --include_path="/solutions/common/sensorless_foc/include/" --include_path="/solutions/boostxl_drv8320rs/f28004x/drivers/include" --include_path="/c2000ware/driverlib/f28004x/driverlib" --include_path="/c2000ware/device_support/f28004x/common/include/" --include_path="/c2000ware/device_support/f28004x/headers/include/" --include_path="C:/ti/ccs1011/ccs/tools/compiler/ti-cgt-c2000_20.2.1.LTS/include" --define=_INLINE --define=_FLASH --define=_BOOSTXL_8320RS_REVA_ --define=DRV8320_SPI --define=_DATALOG_EN_ --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=coffabi --preproc_with_compile --preproc_dependency="src_foc/$(basename $(<F)).d_raw" --obj_directory="src_foc" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


