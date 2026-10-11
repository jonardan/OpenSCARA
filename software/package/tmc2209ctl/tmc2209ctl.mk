################################################################################
#
# tmc2209ctl
#
################################################################################

TMC2209CTL_VERSION = 2.0
TMC2209CTL_SITE = $(TMC2209CTL_PKGDIR)/src
TMC2209CTL_SITE_METHOD = local
TMC2209CTL_LICENSE = MIT

define TMC2209CTL_BUILD_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) $(TARGET_CONFIGURE_OPTS) -C $(@D)
endef

define TMC2209CTL_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/tmc2209ctl $(TARGET_DIR)/usr/bin/tmc2209ctl
	$(INSTALL) -D -m 0755 $(@D)/scara $(TARGET_DIR)/usr/bin/scara
	$(INSTALL) -D -m 0644 $(@D)/square.gcode \
		$(TARGET_DIR)/usr/share/scara/square.gcode
	$(INSTALL) -D -m 0644 $(TMC2209CTL_PKGDIR)/tmc2209.conf \
		$(TARGET_DIR)/etc/tmc2209.conf
endef

define TMC2209CTL_INSTALL_INIT_SYSV
	$(INSTALL) -D -m 0755 $(TMC2209CTL_PKGDIR)/S95tmc2209 \
		$(TARGET_DIR)/etc/init.d/S95tmc2209
endef

$(eval $(generic-package))
