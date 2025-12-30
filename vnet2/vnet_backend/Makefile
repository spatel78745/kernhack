CROSS_COMPILE ?= aarch64-linux-gnu-
CC := $(CROSS_COMPILE)gcc

CFLAGS  ?= -O2 -Wall -Wextra
LDFLAGS ?=

# Try fully static first
STATIC ?= 1
ifeq ($(STATIC),1)
  LDFLAGS += -static
endif

all: vnet_backend

vnet_backend: vnet_backend.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f vnet_backend
