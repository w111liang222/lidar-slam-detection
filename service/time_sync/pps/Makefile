obj-m:=pps_generator.o
KERNELDIR:=/lib/modules/`uname -r`/build
PWD :=$(shell pwd)

modules:
	$(MAKE)  -C  $(KERNELDIR)  M=$(PWD)  modules

clean:
	rm -rf *o *.mod.c *.order *.symvers *.cmd .pps* .tmp*