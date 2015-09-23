CROSS := arm-linux-
CC := $(CROSS)gcc

SOURCES ?= $(wildcard src/*.c)
OBJS    ?= $(addprefix , $(SOURCES:.c=.o))

CFLAGS := -W -Wall -Wshadow -Wextra
CFLAGS += -g -O3 -fno-omit-frame-pointer
CFLAGS += -fno-strict-aliasing
CFLAGS += -I. -Iinclude/

LDFLAGS := -lpthread -lgpio

TARGET := gpio_test

all: $(TARGET)

$(TARGET): $(OBJS)
	@echo "[BIN] $@"
	@$(CC) $^ $(LDFLAGS) -o $@

$(OBJS): %.o: %.c
	@echo "[CC] $<"
	@$(CC) $(CFLAGS) -o $@ -c $<

clean:
	@echo "RM $(TARGET) $(OBJS)"
	@rm -rf $(TARGET) $(OBJS)

format:
	@astyle --style=linux -s4 -n src/*.c
	@astyle --style=linux -s4 -n include/*.h

.PHONY: clean format
