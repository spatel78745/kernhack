# Assuming you have KERNEL_SRC pointing to your 6.6 kernel source
KERNEL_SRC ?= /home/spatel/fun/linux-v6.6
ARCH ?= arm64
CROSS_COMPILE ?= aarch64-linux-gnu-

obj-m += vnet.o

all:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL_SRC) M=$(PWD) modules

clean:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL_SRC) M=$(PWD) clean
