include ../config.mk

SUBDIRS = deasdb3-homing deasdb3-homing-csv generic-complex swm-fm45a sync-units

install-examples:
	mkdir -p $(DESTDIR)$(EMC2_HOME)/share/linuxcnc-ethercat/examples
	cp -R $(SUBDIRS) $(DESTDIR)$(EMC2_HOME)/share/linuxcnc-ethercat/examples

