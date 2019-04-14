

######################################
# target
######################################
TARGET = motorControl


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################

#CPP sources

CPPSRC = \
src/main.cpp \
src/system/system.cpp \
src/platforms/AlexMos32/platform.cpp \
src/platforms/platform_base.cpp \
src/platforms/AlexMos32/spi/spi.cpp


# C sources
C_SOURCES =  \
SPL/src/stm32f30x_adc.c \
SPL/src/stm32f30x_can.c \
SPL/src/stm32f30x_rcc.c \
SPL/src/stm32f30x_gpio.c \
SPL/src/stm32f30x_comp.c \
SPL/src/stm32f30x_tim.c \
SPL/src/stm32f30x_spi.c \
src/system_stm32f30x.c 




# ASM sources
ASM_SOURCES =  \
startup_stm32f30x.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CPP = $(GCC_PATH)/$(PREFIX)g++
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CPP = $(PREFIX)g++
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS = -D USE_STDPERIPH_DRIVER

# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-ICMSIS_5/Include \
-Iinc \
-ISPL/inc \
-Isrc/system \
-Isrc/platforms \
-Isrc/platforms/AlexMos32 \
-Isrc/platforms/AlexMos32/spi




# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif






# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


# compile g++ flags
CPPFLAGS = $(CFLAGS) $(UDEFS) -std=c++11 -flto -fno-exceptions -fno-rtti  


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = stm32f303cb_flash.ld

# libraries
LIBS = -lc -lm -lnosys -static-libstdc++
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections 

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################

# list of objects cpp
OBJECTS := $(addprefix $(BUILD_DIR)/,$(notdir $(CPPSRC:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPPSRC)))
# list of objects C
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))


$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR)
	@echo CPP $(notdir $<)
	@$(CPP) -c $(CPPFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@



$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo C $(notdir $<) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CPP) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  


# *** EOF ***