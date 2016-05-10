export ARCH:=arm
export CROSS_COMPILE:=arm-linux-gnueabihf-

obj-m := imx_rpmsg_tty.o
KDIR := ../../linux-toradex/
PWD := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
deploy: default
	scp imx_rpmsg_tty.ko root@${DEST_IP}:~/
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
