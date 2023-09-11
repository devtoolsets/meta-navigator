DESCRIPTION = "navigator image definition for Xilinx zynq 7020 boards"
LICENSE = "MIT"

inherit core-image

USE_DEVFS = "${@'0' if d.getVar('INITRAMFS_IMAGE_BUNDLE') == '1' else '1'}"
IMAGE_LINGUAS = " zh-cn"

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
		glibc \
		ldd \
		tcpdump \
        haveged \
        ${@'dnf' if d.getVar('PACKAGE_CLASSES') == 'package_rpm' else ''} \
        ${CORE_IMAGE_EXTRA_INSTALL} \
		"
