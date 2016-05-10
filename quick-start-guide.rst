Instruction of the demo preparation
===================================


[STEP 1] Prepare a SD Card
--------------------------

#. Download SD Card image with rootfs.
#. Copy it to the SD Card (replace ``/dev/sdb`` with relevant SD Card device)::

    dd if=Colibri_iMX7_LinuxImage.rootfs.sdcard of=/dev/sdb

[STEP 2] U-Boot fixing
----------------------

#. Clone U-Boot::

    git clone -b 2015.04-imx7-1.1.0_ga-toradex-next git://git.toradex.com/u-boot-toradex.git

#. Apply u-boot-uart1-fix.patch::

    cd u-boot-toradex/
    patch -p 1 < u-boot-uart1-fix.patch

#. Compile U-Boot::

    export ARCH=arm
    export CROSS_COMPILE=arm-linux-gnueabihf-
    make colibri_imx7_defconfig
    make -j9

#. Update U-Boot on the SD Card (replace ``/dev/sdb`` with relevant SD Card device)::

    sudo dd if=u-boot.imx of=/dev/sdb bs=512 seek=2

[STEP 3] Linux kernel fixing
----------------------------

#. Clone the Linux kernel from Toradex git repository::

    git clone -b toradex_imx_3.14.52_1.1.0_ga-next git://git.toradex.com/linux-toradex.git

#. Put the device tree and defconfig files for TAQ (available in ``imx7-taq-demo`` repository) in:

    * ``colibri_imx7_taq_defconfig`` in ``linux-toradex/arch/arm/configs/``
    * ``imx7d-colibri-taq.dts`` in ``linux-toradex/arch/arm/boot/dts/``

#. Compile kernel and device tree::

    cd linux-toradex/
    export ARCH=arm
    export CROSS_COMPILE=arm-linux-gnueabihf-
    make colibri_imx7_taq_defconfig
    make -j9
    make imx7d-colibri-taq.dtb

#. Place ``zImage`` (from ``arch/arm/boot/``) and ``imx7d-colibri-taq.dtb`` (from ``arch/arm/boot/dts/``) in the ``Boot colibr`` partition. Change the name of the DTB file to that of the existing DTB (probably ``imx7d-colibri.dtb``) OR change the ``fdt_file`` variable in the U-Boot environment.
#. Compile the RPMsg TTY driver (available in ``imx7-taq-demo`` repository)::

    git clone --recursive https://github.com/antmicro/imx7-taq-demo.git
    cd imx7-taq-demo/rpmsg-tty-module/
    make

.. highlights::

    ``make`` assumes that ``linux-toradex`` repository is at the same level as the ``imx7-taq-demo`` (ie. ``../linux-toradex/``).

#. Copy the ``imx_rpmsg_tty.ko`` module to the SD Card (eg. ``/home/root``)

[STEP 4] Compile the firmware for TAQ
----------------------------------------

#. Clone repository::

    git clone --recursive https://github.com/antmicro/imx7-taq-demo.git

#. Prepare environment::

    cd imx7-taq-demo/taq-freertos/build
    cp env.conf.example env.conf

#. Edit ``env.conf`` and set proper paths to the ARM toolchain (preferred version is gcc-arm-none-eabi-4_9-2014q4) and ``FreeRTOS`` (set the absolute path).
#. Compile the application::

    ./build_all.sh

#. Resulting firmware is in ``taq-freertos/build/debug/taq.bin`` or ``taq-freertos/build/release/taq.bin``
#. Put the ``taq.bin`` on the ``Boot colibr`` partition on the SD Card.
#. Adjust the U-Boot environment to automatically start the TAQ application::

    setenv fix_m4 "mw.l 0x3033014C 0x00000014; mw.l 0x303303BC 0x00000073; mw.l 0x30330564 0x00000001; mw.l 0x30330024 0x00000002"
    setenv m4boot "run fix_m4; load mmc 0:1 0x920000 taq.bin; dcache flush; bootaux 0x920000"
    setenv bootcmd "run m4boot; run ubiboot; run sdboot; run nfsboot"
    saveenv
