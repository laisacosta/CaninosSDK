# CaninosSDK

This repository contains the source code for Caninos Labrador Operating System.
The kernel source code is located at the folder "kernel".
The u-boot source code is located at the folder "u-boot".
The armhf crosscompiler is located at the folder "toolchain".
The rootfs debian image is located at the directory "debian/system".

**Note** Before compiling, please download the Debian Buster rootfs image from "https://github.com/caninos-loucos/Debian.git" and paste it in the folder "debian/system".

Environment setup (Debian): sudo apt-get install zlib1g-dev libncurses5-dev pkg-config bison flex libssl-dev git make build-essential gparted

# Firmware Creation

1) Go to the folder "owl"
2) Type ./configure.sh and choose "debian" and "coreV2"
3) The generated .fw image will be located at the folder "images" at "owl/out/labrador_debian_coreV2"


