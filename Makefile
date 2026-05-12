.PHONY: all configure install install-rip clean test docs

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

# Install into a LinuxCNC RIP (run-in-place) tree.
# Resolves the target tree from (in order):
#   1. LINUXCNC_HOME passed explicitly (make var or env, e.g. set by sourcing
#      $RIP/scripts/rip-environment)
#   2. config.mk EMC2_HOME, if RUN_IN_PLACE=yes there
#   3. error with usage hint
install-rip: configure
	@LCH="$(LINUXCNC_HOME)"; \
	SRC="explicit LINUXCNC_HOME"; \
	if [ -z "$$LCH" ] && grep -q '^RUN_IN_PLACE = yes' config.mk 2>/dev/null; then \
	    LCH=$$(sed -n 's/^EMC2_HOME = //p' config.mk); \
	    SRC="config.mk EMC2_HOME (RUN_IN_PLACE=yes)"; \
	fi; \
	if [ -z "$$LCH" ]; then \
	    echo "Error: cannot resolve target LinuxCNC RIP tree."; \
	    echo "  Either source the RIP environment first:"; \
	    echo "    source /path/to/linuxcnc-rip/scripts/rip-environment"; \
	    echo "    make install-rip"; \
	    echo "  or pass it explicitly:"; \
	    echo "    make install-rip LINUXCNC_HOME=/path/to/linuxcnc-rip"; \
	    exit 1; \
	fi; \
	echo "Installing into RIP tree: $$LCH ($$SRC)"; \
	$(MAKE) -C src install EMC2_HOME=$$LCH RTLIBDIR=$$LCH/rtlib; \
	$(MAKE) -C examples install-examples EMC2_HOME=$$LCH

configure: config.mk

config.mk: configure.mk
	@$(MAKE) -s -f configure.mk > config.mk.tmp
	@mv config.mk.tmp config.mk

docs: documentation/DEVICES.md

documentation/DEVICES.md: src/lcec_devices documentation/devices/*.yml
	(cd scripts; ./update-devicelist.sh)
	(cd scripts; ./update-devicetable.sh)
