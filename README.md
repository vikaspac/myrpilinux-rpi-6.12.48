
Linux kernel
============

This is cloned from https://github.com/raspberrypi/linux on Sun 28 Sep 18:56:46 IST 2025
This is just for my experimental purpose only.


Sync and Build commands
======================

#Build tools
sudo apt install bc bison flex libssl-dev make 

#Sync
git clone --depth=1 --branch rpi-6.12.y https://github.com/raspberrypi/linux 

#Kernel Compilation RPI4
cd linux 
KERNEL=kernel8 
make bcm2711_defconfig 

#Kernel Compilation RPI5
cd linux
KERNEL=kernel_2712
make bcm2712_defconfig

#Local Kernel version edit command
CONFIG_LOCALVERSION="-vikasred-kernel-28092025-rpi-6.12.y" 

#Compile
make -j4 Image.gz modules dtbs 

#Modules install
sudo make -j4 modules_install 

#Update the files for flashing 
sudo cp /boot/firmware/$KERNEL.img /boot/firmware/$KERNEL-backup.img 
sudo cp arch/arm64/boot/Image.gz /boot/firmware/$KERNEL.img 
sudo cp arch/arm64/boot/dts/broadcom/*.dtb /boot/firmware/ 
sudo cp arch/arm64/boot/dts/overlays/*.dtb* /boot/firmware/overlays/ 
sudo cp arch/arm64/boot/dts/overlays/README /boot/firmware/overlays/ 

sudo cp arch/arm/boot/dts/broadcom/*.dtb /boot/firmware/ 

sudo cp arch/arm/boot/dts/overlays/*.dtb* /boot/firmware/overlays/ 
sudo cp arch/arm/boot/dts/overlays/README /boot/firmware/overlays/ 

#Reboot the device
sudo reboot 



My Information
==============

Raspberry Pi 4 Model B Rev 1.5
[    0.000000] Linux version 6.12.48-v8+ (vikasred@rpiwork1) (gcc (Debian 12.2.0-14) 12.2.0, GNU ld (GNU Binutils for Debian) 2.40) #1 SMP PREEMPT Sat Sep 27 09:40:40 BST 2025

vikasred@rpiwork1:~ $ uname -a
Linux rpiwork1 6.12.48-v8+ #1 SMP PREEMPT Sat Sep 27 09:40:40 BST 2025 aarch64 GNU/Linux
vikasred@rpiwork1:~ $ uname -r
6.12.48-v8+



General Commands
================

dmesg
raspinfo
lsusb -t



My VNC ID
=========
1308675394


My github commands
==================
git push -u origin main


Credits
=======

https://www.raspberrypi.com/documentation/computers/linux_kernel.html
cat MAINTAINERS


