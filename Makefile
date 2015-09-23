# ------------------------------------------------------------------------
#       libgpio
# ------------------------------------------------------------------------
CROSS :=arm-linux-

CC := $(CROSS)gcc
AR := $(CROSS)ar

#
ALL_C   := $(wildcard *.c)
SOURCES ?= $(filter-out %_test.c,$(ALL_C))
OBJS    ?= $(addprefix , $(SOURCES:.c=.o))

#
# CFLAGS, LDFLAGS and TARGET
#
CFLAGS := -Wall -W -Wstrict-prototypes -Wwrite-strings
CFLAGS += -O3 -fPIC
CFLAGS += -g -ggdb

LDFLAGS :=

LIBNAME := libgpio

GPIO_MAJOR := 1
GPIO_MINOR := 0
# ------------------------------------------------------------------------
#	dynamic library
# ------------------------------------------------------------------------
DYLIBSUFFIX=so
DYLIB_MINOR_NAME=$(LIBNAME).$(DYLIBSUFFIX).$(GPIO_MAJOR).$(GPIO_MINOR)
DYLIB_MAJOR_NAME=$(LIBNAME).$(DYLIBSUFFIX).$(GPIO_MAJOR)
DYLIBNAME=$(LIBNAME).$(DYLIBSUFFIX)

# ------------------------------------------------------------------------
#       static (*.a) library
# ------------------------------------------------------------------------
STLIBSUFFIX=a
STLIBNAME=$(LIBNAME).$(STLIBSUFFIX)

all: static dynamic

dynamic: $(OBJS)
	$(CC) -shared -Wl,-soname,$(DYLIB_MINOR_NAME) -o $(DYLIBNAME) $(LDFLAGS) $^

static: $(OBJS)
	$(AR) rcs $(STLIBNAME) $^

$(OBJS): %.o: %.c
	$(CC) -std=c99 -c $(CFLAGS) $<

clean:
	rm -rf $(OBJS) $(DYLIBNAME) $(STLIBNAME)


.PHONY: clean dynamic static
