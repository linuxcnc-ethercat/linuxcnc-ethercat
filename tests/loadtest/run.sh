#!/bin/bash
# Load test: lcec must load and activate with generic slaves that have no
# HAL pins.  hal_malloc(0) was benign until LinuxCNC 2.10.0~pre2 rejected
# it, breaking every config with a pin-less generic slave (#529).  Runs
# without hardware: the IgH master is bound to a veth pair, so the master
# activates and cyclic frames run with zero slaves answering.
#
# Expects: linuxcnc-uspace installed, etherlab built at $ETHERCAT_SRC
# (default /opt/ethercat), lcec installed, root (kernel modules + veth).

set -u
cd "$(dirname "$0")/../.." || exit 1
OUT=/tmp/lcec-loadtest.out
ETHERCAT_SRC=${ETHERCAT_SRC:-/opt/ethercat}

fail() {
    echo "FAIL: $*"
    halrun -U > /dev/null 2>&1
    exit 1
}

echo "=== veth pair"
ip link del veth-ec0 2> /dev/null
# Fixed, locally administered MACs: with a random one, systemd-udev's
# MACAddressPolicy=persistent can rewrite the address after we read it, and
# the master then waits forever for a device that no longer exists.
ip link add veth-ec0 address 02:00:00:00:ec:00 type veth peer name veth-ec1 address 02:00:00:00:ec:01
command -v udevadm > /dev/null && udevadm settle
ip link set veth-ec0 up
ip link set veth-ec1 up
MAC=$(cat /sys/class/net/veth-ec0/address)

echo "=== ethercat master on veth-ec0 ($MAC)"
# ec_generic uses ec_master symbols: master first.
insmod "$ETHERCAT_SRC/master/ec_master.ko" main_devices="$MAC" || fail "insmod ec_master"
insmod "$ETHERCAT_SRC/devices/ec_generic.ko" || fail "insmod ec_generic"
# ec_master creates /dev/EtherCAT0 as soon as it loads, but ec_generic binds
# veth-ec0 asynchronously. Requesting the master before the device is
# attached fails with "Failed to reserve master: No such device", so wait
# for the attach rather than a fixed time.
ETHERCAT_TOOL=$(command -v ethercat || echo "$ETHERCAT_SRC/tool/ethercat")
attached=0
for _ in $(seq 1 30); do
    if [ -c /dev/EtherCAT0 ] && "$ETHERCAT_TOOL" master 2> /dev/null | grep -q "(attached)"; then
        attached=1
        break
    fi
    sleep 1
done
[ -c /dev/EtherCAT0 ] || fail "/dev/EtherCAT0 missing"
if [ "$attached" != 1 ]; then
    "$ETHERCAT_TOOL" master
    echo "master expects $MAC; veth-ec0 is now: $(ip -br link show veth-ec0)"
    dmesg | grep -i ethercat | tail -n 20
    fail "master device not attached to veth-ec0 after 30 s"
fi
# rtapi_app drops to RTAPI_UID before lcec opens the master device;
# production grants access via the etherlab udev rule, the test just
# opens it up.
chmod 666 /dev/EtherCAT0 || fail "chmod /dev/EtherCAT0"

echo "=== halrun"
halrun -f tests/loadtest/pinless.hal > "$OUT" 2>&1 &
# halcmd show pin exits 0 with empty output when nothing matches, so
# every pin assertion must grep the output, not trust the exit code.
for _ in $(seq 1 30); do
    halcmd show pin | grep -q "lcec.0.withpin.dout-0" && break
    sleep 1
done
if grep -q "hal_malloc bad size" "$OUT"; then
    cat "$OUT"
    fail "hal_malloc(0) called during load"
fi
halcmd show pin | grep -q "lcec.0.withpin.dout-0" || { cat "$OUT"; fail "pin lcec.0.withpin.dout-0 missing, lcec did not load"; }
halcmd show pin | grep -q "lcec.0.coupler.slave-state-op" || fail "state pins for pin-less slave missing"

echo "=== cyclic thread running (master activated)"
t1=$(halcmd getp lcec.read-all.time)
sleep 1
t2=$(halcmd getp lcec.read-all.time)
[ "$t1" != "$t2" ] || { cat "$OUT"; fail "lcec.read-all not running"; }

halrun -U
rmmod ec_generic
rmmod ec_master
echo "PASS: lcec loaded, master activated, pins exported"
