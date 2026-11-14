# Orderly EtherCAT shutdown

Some drives report a synchronization-loss fault if cyclic process data stops
while they are still in OP. Requesting PREOP before LinuxCNC stops its realtime
threads avoids that unannounced communication gap.

LinuxCNC executes the optional `[HAL]SHUTDOWN` file before `halcmd stop`. Add
this to the INI file:

```ini
[HAL]
SHUTDOWN = shutdown.hal
```

Copy [examples/shutdown.hal](../examples/shutdown.hal) into the configuration
directory, or create `shutdown.hal` with these commands:

```sh
loadusr -w ethercat states PREOP
loadusr -w timeout 6 sh -c 'until test "`ethercat slaves | grep -cv " PREOP "`" = 0; do sleep 0.05; done'
```

The first command submits the PREOP request from a non-realtime process. The
normal lcec cyclic functions keep running, so the IgH master can send the AL
control datagrams and receive their replies. The second command waits until
`ethercat slaves` reports PREOP for every slave, bounded to six seconds.
LinuxCNC then stops the realtime threads and unloads lcec.

This uses only the standard IgH `ethercat` tool and works with both upstream
IgH and the linuxcnc-ethercat fork. No additional lcec API, HAL pins, userspace
component, or realtime ioctl is involved.

For configurations where confirmation is unnecessary, a fixed delay is a
shorter alternative:

```sh
loadusr -w ethercat states PREOP
loadusr -w sleep 1
```

Backticks in the confirmed-wait example are intentional. halcmd performs its
own `$` and `[...]` substitution before executing `loadusr`, so the embedded
shell command avoids both.
