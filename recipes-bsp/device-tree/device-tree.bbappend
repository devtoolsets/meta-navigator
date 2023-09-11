
PETALINUX := "${THISDIR}/${PN}"
FILESEXTRAPATHS:prepend := "${PETALINUX}/:"

SRC_URI:append = " file://config file://system-user.dtsi"

export PETALINUX

do_configure:append () {
    script="${PETALINUX}/etc/hsm/scripts/petalinux_hsm_bridge.tcl"
    data=${PETALINUX}/etc/hsm/data/
    eval xsct -sdx -nodisp ${script} -c ${WORKDIR}/config \
        -hdf ${DT_FILES_PATH}/hardware_description.${HDF_EXT} -repo ${S} \
        -data ${data} -sw ${DT_FILES_PATH} -o ${DT_FILES_PATH} -a "soc_mapping"
}
