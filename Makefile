CROSS_COMPILE ?= 
ifneq ($(strip $(CROSS_COMPILE)),)
  CC := $(CROSS_COMPILE)gcc
else
  CC ?= gcc
endif

SRC_DIR     := src
INC_DIR     := include
BUILD_DIR   := build
TARGET      := automotive_control

SRCS        := $(wildcard $(SRC_DIR)/*.c)
OBJS        := $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(SRCS))

CFLAGS      := -Wall -Wextra -O2 -I$(INC_DIR) -pthread
LDFLAGS     := -pthread -lrt -lm

STATIC ?= 0
ifeq ($(STATIC),1)
  LDFLAGS += -static
endif

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

run: $(TARGET)
	./$(TARGET) 10

rpi:
	$(MAKE) clean
	$(MAKE) all CROSS_COMPILE=$(CROSS_COMPILE) STATIC=1

clean:
	rm -rf $(BUILD_DIR) $(TARGET)

.PHONY: all run clean rpi
