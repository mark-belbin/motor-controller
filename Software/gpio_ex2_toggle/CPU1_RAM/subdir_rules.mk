################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_18.12.2.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu0 -Ooff --include_path="D:/Motor Controller Research/GitHub/motor-controller/Software/gpio_ex2_toggle" --include_path="D:/Motor Controller Research/GitHub/motor-controller/Software/gpio_ex2_toggle/device" --include_path="C:/ti/c2000/C2000Ware_3_02_00_00/driverlib/f28004x/driverlib" --include_path="C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_18.12.2.LTS/include" --define=DEBUG --define=CPU1 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="D:/Motor Controller Research/GitHub/motor-controller/Software/gpio_ex2_toggle/CPU1_RAM/syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-97286625:
	@$(MAKE) --no-print-directory -Onone -f subdir_rules.mk build-97286625-inproc

build-97286625-inproc: ../gpio_ex2_toggle.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/ccs910/ccs/utils/sysconfig/sysconfig_cli.bat" -d "F28004x" -s "C:/ti/c2000/C2000Ware_3_02_00_00/.metadata/sdk.json" -o "syscfg" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/board.c: build-97286625 ../gpio_ex2_toggle.syscfg
syscfg/board.h: build-97286625
syscfg/pinmux.csv: build-97286625
syscfg/: build-97286625

syscfg/%.obj: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_18.12.2.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu0 -Ooff --include_path="D:/Motor Controller Research/GitHub/motor-controller/Software/gpio_ex2_toggle" --include_path="D:/Motor Controller Research/GitHub/motor-controller/Software/gpio_ex2_toggle/device" --include_path="C:/ti/c2000/C2000Ware_3_02_00_00/driverlib/f28004x/driverlib" --include_path="C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_18.12.2.LTS/include" --define=DEBUG --define=CPU1 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="syscfg/$(basename $(<F)).d_raw" --include_path="D:/Motor Controller Research/GitHub/motor-controller/Software/gpio_ex2_toggle/CPU1_RAM/syscfg" --obj_directory="syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


