###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return

###############################################################################

# external libraries
DRIVERS_DIR = lib/drivers
TURBOTRIG_DIR = lib/turbotrig

# project source files
PROJECT_SRC = src/main.c \
              src/estimator.c \
              src/flash.c \
              src/mavlink.c \
              src/mavlink_param.c \
              src/mavlink_receive.c \
              src/mavlink_stream.c \
              src/mavlink_util.c \
              src/mixer.c \
              src/param.c \
              src/rc.c \
              src/sensors.c \
              src/mux.c \
              src/controller.c \
              src/mode.c \

###############################################################################

# You probably shouldn't modify anything below here!

TARGET		?= rosflight2

# Compile-time options
OPTIONS		?=

# Debugger options, must be empty or GDB
DEBUG ?=

# Serial port/device for flashing
SERIAL_DEVICE	?= /dev/ttyUSB0

# Working directories
ROOT		 = $(dir $(lastword $(MAKEFILE_LIST)))
SRC_DIR		 = $(ROOT)
CMSIS_DIR	 = $(DRIVERS_DIR)/lib/CMSIS
STDPERIPH_DIR	 = $(DRIVERS_DIR)/lib/STM32F30x_StdPeriph_Driver
OBJECT_DIR	 = $(ROOT)/build
BIN_DIR		 = $(ROOT)/build

rosflight2_SRC =\
		   $(DRIVERS_DIR)/gpio_stm32f30x.c \
		   $(DRIVERS_DIR)/drv_i2c.c \
		   $(DRIVERS_DIR)/drv_adc.c \
		   $(DRIVERS_DIR)/drv_spi.c \
		   $(DRIVERS_DIR)/drv_pwm.c \
		   $(DRIVERS_DIR)/drv_system.c \
		   $(DRIVERS_DIR)/drv_serial.c \
		   $(DRIVERS_DIR)/drv_uart.c \
		   $(DRIVERS_DIR)/drv_timer.c \
		   $(DRIVERS_DIR)/drv_mpu6050.c \
		   $(DRIVERS_DIR)/drv_ms4525.c \
		   $(DRIVERS_DIR)/drv_mb1242.c \
		   $(DRIVERS_DIR)/drv_ms5611.c \
		   $(DRIVERS_DIR)/printf.c \
		   $(DRIVERS_DIR)/startup_stm32f30x_md_gcc.S \
		   $(TURBOTRIG_DIR)/turbotrig.c \
		   $(TURBOTRIG_DIR)/turbovec.c \
		   $(PROJECT_SRC) \
		   $(CMSIS_SRC) \
		   $(STDPERIPH_SRC)

VPATH		:= $(SRC_DIR):$(SRC_DIR)/startups

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(CMSIS_DIR)/CM1/CoreSupport:$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM1/CoreSupport/*.c \
			               $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x/*.c))

# Search path and source files for the ST stdperiph library
VPATH		:= $(VPATH):$(STDPERIPH_DIR)/src
STDPERIPH_SRC	 = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

###############################################################################
# Things that might need changing to use different tools
#

# Tool names
CC		 = arm-none-eabi-gcc -std=gnu99
OBJCOPY	 = arm-none-eabi-objcopy

#
# Tool options.
#
INCLUDE_DIRS	 = include \
		   lib \
		   $(DRIVERS_DIR) \
		   $(STDPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM1/CoreSupport \
		   $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x \

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3

ifeq ($(DEBUG),GDB)
OPTIMIZE	 = -Og

else
OPTIMIZE	 = -Os
LTO_FLAGS	 = -flto -fuse-linker-plugin $(OPTIMIZE)
endif

DEBUG_FLAGS	 = -ggdb3

CFLAGS		 = $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(addprefix -D,$(OPTIONS)) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   $(DEBUG_FLAGS) \
		   -Wall -pedantic -Wextra -Wshadow -Wunsafe-loop-optimizations \
		   -ffunction-sections \
		   -fdata-sections \
		   -DSTM32F10X_MD \
		   -DUSE_STDPERIPH_DRIVER \
		   -D$(TARGET)

ASFLAGS		 = $(ARCH_FLAGS) \
		   -x assembler-with-cpp \
		   $(addprefix -I,$(INCLUDE_DIRS))

LD_SCRIPT	 = $(DRIVERS_DIR)/stm32_flash.ld
LDFLAGS		 = -lm \
		   -nostartfiles \
		   --specs=nano.specs \
		   -lc \
		   -lnosys \
		   $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(DEBUG_FLAGS) \
		   -static \
		   -Wl,-gc-sections,-Map,$(TARGET_MAP) \
		   -T$(LD_SCRIPT)

#
# Things we will build
#

TARGET_HEX	 = $(BIN_DIR)/$(TARGET).hex
TARGET_ELF	 = $(BIN_DIR)/$(TARGET).elf
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))
TARGET_MAP   = $(OBJECT_DIR)/$(TARGET).map

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

MKDIR_OBJDIR = @mkdir -p $(dir $@)

# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	$(MKDIR_OBJDIR)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.S
	$(MKDIR_OBJDIR)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<

clean:
	rm -rf $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP) $(BIN_DIR) $(OBJECT_DIR)

flash_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 921600 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 921600 $(SERIAL_DEVICE)

flash: flash_$(TARGET)

unbrick: $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

listen:
	miniterm.py $(SERIAL_DEVICE) 115200
