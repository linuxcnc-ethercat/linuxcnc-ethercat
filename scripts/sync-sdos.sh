#!/bin/bash

device="$*"

#
# This dumps all of the SDO values for one or more devices.  Call it with '-p 2' to show all of the SDOs from slave #2.

function fetch {
    if echo "$line" | egrep -q '^  0x....:..'; then
	obj=$(echo "$line" | sed 's/:\(..\), .*/ 0x\1/' )
	val=$(ethercat $device --type $type upload $obj  2> /dev/null)
	printf "%-80s   %s\n" "$(echo "$line" | cat -vT)" "$val" 2> /dev/null
    else
	echo "$line"
    fi
}

echo 'SDO 0x1c32, "SM output parameter"'
type=uint16 line='  0x1c32:01, rwr-r-, uint16, 16 bit, "Synchronization Type"' fetch
type=uint32 line='  0x1c32:02, r-r-r-, uint32, 32 bit, "Cycle Time"' fetch
type=uint16 line='  0x1c32:04, r-r-r-, uint16, 16 bit, "Synchronization Types supported"' fetch
type=uint32 line='  0x1c32:05, r-r-r-, uint32, 32 bit, "Minimum Cycle Time"' fetch
type=uint32 line='  0x1c32:06, r-r-r-, uint32, 32 bit, "Calc and Copy Time"' fetch
type=uint16 line='  0x1c32:08, rwrwrw, uint16, 16 bit, "Get Cycle Time"' fetch
type=uint32 line='  0x1c32:09, r-r-r-, uint32, 32 bit, "Delay Time"' fetch
type=uint32 line='  0x1c32:0a, rwrwrw, uint32, 32 bit, "Sync0 Cycle Time"' fetch
type=uint16 line='  0x1c32:0b, r-r-r-, uint16, 16 bit, "SM-Event Missed"' fetch
type=uint16 line='  0x1c32:0c, r-r-r-, uint16, 16 bit, "Cycle Time Too Small"' fetch
type=uint16 line='  0x1c32:0d, ------, type 0000, 16 bit, "Shift Time Too Short"' fetch
type=uint8 line='  0x1c32:20, r-r-r-, bool, 1 bit, "Sync Error"' fetch
echo 'SDO 0x1c33, "SM input parameter"'
type=uint16 line='  0x1c33:01, rwr-r-, uint16, 16 bit, "Synchronization Type"' fetch
type=uint32 line='  0x1c33:02, r-r-r-, uint32, 32 bit, "Cycle Time"' fetch
type=uint16 line='  0x1c33:04, r-r-r-, uint16, 16 bit, "Synchronization Types supported"' fetch
type=uint32 line='  0x1c33:05, r-r-r-, uint32, 32 bit, "Minimum Cycle Time"' fetch
type=uint32 line='  0x1c33:06, r-r-r-, uint32, 32 bit, "Calc and Copy Time"' fetch
type=uint16 line='  0x1c33:08, rwrwrw, uint16, 16 bit, "Get Cycle Time"' fetch
type=uint32 line='  0x1c33:09, r-r-r-, uint32, 32 bit, "Delay Time"' fetch
type=uint32 line='  0x1c33:0a, rwrwrw, uint32, 32 bit, "Sync0 Cycle Time"' fetch
type=uint16 line='  0x1c33:0b, r-r-r-, uint16, 16 bit, "SM-Event Missed"' fetch
type=uint16 line='  0x1c33:0c, r-r-r-, uint16, 16 bit, "Cycle Time Too Small"' fetch
type=uint16 line='  0x1c33:0d, ------, type 0000, 16 bit, "Shift Time Too Short"' fetch
type=uint8 line='  0x1c33:20, r-r-r-, bool, 1 bit, "Sync Error"' fetch
