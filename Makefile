.PHONY: all configure install clean test docs

build: configure
	@$(MAKE) -C src all

all: build docs

clean:
	@$(MAKE) -C src -f Makefile.clean clean
	rm -f config.mk config.mk.tmp

test:
	@$(MAKE) -C src test

install: configure
	@$(MAKE) -C src install
	@$(MAKE) -C examples install-examples

configure: config.mk

config.mk: configure.mk
	@$(MAKE) -s -f configure.mk > config.mk.tmp
	@mv config.mk.tmp config.mk

docs: documentation/DEVICES.md

documentation/DEVICES.md: src/lcec_devices documentation/devices/*.yml
	(cd scripts; ./update-devicelist.sh)
	(cd scripts; ./update-devicetable.sh)
