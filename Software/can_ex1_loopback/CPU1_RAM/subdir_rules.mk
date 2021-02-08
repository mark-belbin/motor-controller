################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_18.12.4.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu0 -Ooff --include_path="D:/Motor Controller Research/GitHub/motor-controller/Software/can_ex1_loopback" --include_path="D:/Motor Controller Research/GitHub/motor-controller/Software/can_ex1_loopback/device" --include_path="C:/ti/c2000/C2000Ware_3_02_00_00/driverlib/f28004x/driverlib" --include_path="C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_18.12.4.LTS/include" --define=DEBUG --define=CPU1 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


