FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
SRC_URI:append := "file://login_defs_user.sed"

do_install:append() {
    sed -i -f ${WORKDIR}/login_defs_user.sed ${D}${sysconfdir}/login.defs
}
