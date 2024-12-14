################################################################################
# MRS Version: 1.9.2
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/subdir.mk
-include Startup/subdir.mk
-include Peripheral/src/subdir.mk
-include Core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
Test.hex \

SECONDARY_LIST += \
Test.lst \

SECONDARY_SIZE += \
Test.siz \


# All Target
all: Test.elf secondary-outputs

# Tool invocations
Test.elf: $(OBJS) $(USER_OBJS)
	@	@	riscv-none-embed-gcc -march=rv32ecxw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -T "D:\Projects\LED\Alex\CH32V003\Test\Ld\Link.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"Test.map" --specs=nano.specs --specs=nosys.specs -o "Test.elf" $(OBJS) $(USER_OBJS) $(LIBS) -lprintf
	@	@
Test.hex: Test.elf
	@	riscv-none-embed-objcopy -O ihex "Test.elf"  "Test.hex"
	@	@
Test.lst: Test.elf
	@	riscv-none-embed-objdump --source --all-headers --demangle --disassemble -M xw "Test.elf" > "Test.lst"
	@	@
Test.siz: Test.elf
	@	riscv-none-embed-size --format=berkeley "Test.elf"
	@	@
# Other Targets
clean:
	-$(RM) $(ASM_UPPER_DEPS)$(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS) Test.elf
	-@
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
