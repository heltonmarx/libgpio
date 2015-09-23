libgpio
======

Small library to read and write GPIOs of BeagleBone Black

If you are using [buildroot][http://buildroot.uclibc.org/] as toolchain, it's possible copy
the dynamic library to the toolchain directory.

For instance, buildroot was installed in `/opt/toolchain/buldroot`, follow the steps bellow:

```sh
cp -a gpio.h /opt/toolchain/buildroot/output/host/usr/arm-buildroot-linux-gnueabihf/sysroot/usr/include/gpio.h
cp -a libgpio.so /opt/toolchain/buildroot/output/host/usr/arm-buildroot-linux-gnueabihf/sysroot/usr/lib/libgpio.so.1.0
cd /opt/toolchain/buildroot/output/host/usr/arm-buildroot-linux-gnueabihf/sysroot/usr/lib/
ln -sf libgpio.so.1.0 libgpio.so
ln -sf libgpio.so.1 libgpio.so.1.0

```

P.s.: don't forget to copy dynamic library to rootfs when it is created.
Normally, the library directory is : `/usr/lib`.
