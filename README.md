# linuxcnc-ethercat

This is a set of [LinuxCNC](https://linuxcnc.org/) drivers for
EtherCAT devices, intended to be used to help build a [CNC
machine](https://en.wikipedia.org/wiki/Numerical_control).

[EtherCAT](https://en.wikipedia.org/wiki/EtherCAT) is a standard for
connecting industrial control equipment to PCs using Ethernet.
EtherCAT uses dedicated Ethernet networks and achieves consistently
low latency without requiring special hardware.  A number of
manufacturers produce EtherCAT equipment for driving servos, steppers,
digital I/O, and reading sensors.

This tree was forked from
[sittner/linuxcnc-ethercat](https://github.com/sittner/linuxcnc-ethercat)
in 2023, and is the new home for most LinuxCNC EtherCAT development.

## Installing

The recommended way to install this driver is via the project's own
apt repository at <https://linuxcnc-ethercat.github.io/apt/>, served
from GitHub Pages.  It ships both `linuxcnc-ethercat` and the matching
`ethercat-master` (the IgH EtherLab master rebuilt with fixes that
have not been picked up upstream) for **Debian 11 / 12 / 13** on
**amd64**.

> The previously-recommended openSUSE `science:EtherLab` repository is
> no longer the preferred source: it serves the unmodified IgH build
> and is missing several fixes important to LinuxCNC users (DC-sync
> cold-start, reference-clock poll storm, etc.). The packages in our
> apt repo carry a Debian epoch (`1:1.6.9-…`) that supersedes the
> openSUSE version on a normal `apt upgrade`, so switching is a
> single-step operation.

### Initial setup

Run as root or with `sudo`:

```sh
# 1. Install the archive signing key
curl -fsSL https://linuxcnc-ethercat.github.io/apt/linuxcnc-ethercat-apt.gpg \
    -o /usr/share/keyrings/linuxcnc-ethercat-apt.gpg

# 2. Detect the codename and add the apt source
. /etc/os-release
case "$VERSION_CODENAME" in
    bullseye|bookworm|trixie) ;;
    *) echo "Unsupported codename: $VERSION_CODENAME (supported: bullseye, bookworm, trixie)" >&2; exit 1 ;;
esac
echo "deb [signed-by=/usr/share/keyrings/linuxcnc-ethercat-apt.gpg] \
https://linuxcnc-ethercat.github.io/apt/ $VERSION_CODENAME main" \
    > /etc/apt/sources.list.d/linuxcnc-ethercat.list

# 3. Install
apt update
apt install -y linux-headers-$(uname -r) ethercat-master linuxcnc-ethercat
```

**Note:** If you previously followed older instructions and added the
openSUSE `science:EtherLab` source, you can leave it in place — apt
will prefer our packages because of the version epoch — or remove it
with `rm /etc/apt/sources.list.d/ighvh.sources`. The official
LinuxCNC 2.9.x ISO ships with the openSUSE source pre-installed;
adding the lines above is enough to migrate.

You will then need to do a bit of setup for Ethercat; at a minimum
you'll need to edit `/etc/ethercat.conf` to tell it which interface it
should use.  See the [LinuxCNC
forum](https://forum.linuxcnc.org/ethercat/45336-ethercat-installation-from-repositories-how-to-step-by-step)
for additional details.

You can verify that Ethercat is working when `ethercat slaves` shows
the devices attached to your system.

### Updates

Ongoing updates are handled by `apt`: `sudo apt update && sudo apt
upgrade`. If the kernel is upgraded, you may need to re-run

```
sudo apt install -y linux-headers-$(uname -r)
```

to get the matching headers, otherwise the `ethercat` DKMS module
cannot rebuild against the new kernel. Reboot or
`sudo systemctl start ethercat` afterwards.

### Manual Installation

If you would rather build from source, you need a working
[Ethercat Master](https://github.com/linuxcnc-ethercat/ethercat) and
LinuxCNC (with development headers) installed.  Then clone
linuxcnc-ethercat and run `make install`.


## Configuring

At a minimum, you will need two files.  First, you'll need an XML file
(commonly called `ethercat.xml`) that describes your hardware.  Then
you'll need a LinuxCNC HAL file that loads the LinuxCNC-Ethercat
driver and tells LinuxCNC about your CNC.

There are two ways to use EtherCAT hardware with this driver.  First,
many devices have dedicated drivers which know about all of the
details of devices.  For instance, you can tell it to use a [Beckhoff
EL7041 Stepper controller](http://beckhoff.com/EL7041) as `x-axis` by
saying

```XML
   <slave idx="3" type="EL7041" name="x-axis"/>
```

This will create a number of LinuxCNC
[`pins`](https://linuxcnc.org/docs/html/hal/intro.html) that talk
directly to the EL7041 and control the stepper connected to it.  You
will still need to tell LinuxCNC what to do with the new hardware, but
the low-level details will be handled automatically.

The second way to use EtherCAT hardware is with the "generic" driver.
You can tell LinuxCNC-Ethercat about your hardware entirely in XML,
and it will let you send EtherCAT messages to any hardware, even if
we've never seen it before.  This is easier than writing a new driver,
but more difficult than using a pre-written driver.

A [reference guide to LinuxCNC-Ethercat's XML
configuration](documentation/configuration-reference.md) file is
available.

Several examples are available in the [`examples/`](examples/)
directory, but they're somewhat dated.  The [LinuxCNC
Forum](https://forum.linuxcnc.org/ethercat) is a better place to
start.

There is also a new, experimental tool included called
`lcec_configgen` that will attempt to automatically create an XML
configuration for you by examining the EtherCAT devices attached to
the system.  It should recognize all devices with pre-compiled
drivers, and will attempt to create generic drivers for other devices.
It's not always perfect, but it's usually an OK starting point.  The
configgen tool will not overwrite any files, so it should be safe to
run.

## Devices Supported

See [the device documentation](documentation/DEVICES.md) for a partial
list of Ethercat devices supported by this project.  Not all devices
are equally supported.  If you have any problems, please [file a
bug](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/issues/new/choose).

## Breaking Changes

We try not to deliberately break working systems while we're working
on LinuxCNC-Ethercat, but there are times when it's simply
unavoidable.  Sometimes this happens due to the nature of the bug, and
there's no safe way *not* to break things.  Sometimes it happens
because the existing behavior is so broken that it's not reasonable to
leave it in place, and other times it happens because we believe that
there are no impacted users.

See [the changes file](documentation/changes.md) for a list of
potentially-breaking changes.  In general, we try to communicate
potentially breaking changes via the LinuxCNC forums.

## Contributing

See [the contributing documentation](CONTRIBUTING.md) for details.  If
you have any issues with the contributing process, *please* file an
issue here.  Everything is new, and it may be broken.

[API
Documentation](https://linuxcnc-ethercat.github.io/linuxcnc-ethercat/doxygen/)
via Doxygen is available, but incomplete.
