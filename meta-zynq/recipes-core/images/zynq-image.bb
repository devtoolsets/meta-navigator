DESCRIPTION = "zynq image definition for Xilinx zynq 7020 boards"
LICENSE = "MIT"

inherit core-image

IMAGE_AUTOLOGIN ?= "0"

AUTOLOGIN = "${@ 'autologin' if d.getVar('IMAGE_AUTOLOGIN') == '1' else '' }"

inherit ${AUTOLOGIN}

#Create devfs entries for initramfs(bundle) image
USE_DEVFS = "${@'0' if d.getVar('INITRAMFS_IMAGE_BUNDLE') == '1' else '1'}"

inherit extrausers 

#EXTRA_USERS_PARAMS = "passwd-expire root;"
EXTRA_USERS_PARAMS = "useradd -p dev dev;usermod -p root root;"

IMAGE_LINGUAS = " "

IMAGE_INSTALL = "\
		packagegroup-core-boot \
		packagegroup-core-ssh-dropbear \
		openssh-sftp-server \
		kernel-modules \
        u-boot-tools \
        linux-xlnx-udev-rules \
		udev-extraconf \
		mtd-utils \
		bash \
		bash-completion \
		bash-completion-extra \
		file \
		strace \
		run-postinsts \
		gdb \
		gdbserver \
		glibc \
		ldd \
		tcpdump \
        haveged \
        e2fsprogs-mke2fs \
        bridge-utils \
		"
