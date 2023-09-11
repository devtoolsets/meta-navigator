FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI += "file://dropbear_ecdsa_host_key"

do_install:append () {
    install ${WORKDIR}/dropbear_ecdsa_host_key ${D}${sysconfdir}/dropbear/
    sed -e 's/RSA/ECDSA/g' \
        -e 's/rsa/ecdsa/g' \
        -i ${D}${sysconfdir}/init.d/dropbear
}
