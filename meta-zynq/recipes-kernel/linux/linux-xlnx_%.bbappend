KERNEL_CONFIG_COMMAND = ""
FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI += "file://user.cfg \
            file://igb-uio/ \
            "
RDEPENDS_kernel-base = ""
KERNEL_IMAGETYPE:zynq ?= "zImage"

do_configure:append () {
	merge_config.sh -m ${B}/.config ${@" ".join(find_sccs(d))}
	oe_runmake -C ${S} O=${B} oldconfig
}

do_install:append() {
    for dir in igb-uio; do
        oe_runmake ARCH=${ARCH} CROSS_COMPILE=${CROSS_COMPILE} M=${B}/../${dir} -C ${B} modules
        install ${B}/../${dir}/${dir}.ko ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}
    done
}

do_deploy:append () {
	install -m 0644 ${D}/boot/System.map-${KERNEL_VERSION} ${DEPLOYDIR}/System.map.linux
}
