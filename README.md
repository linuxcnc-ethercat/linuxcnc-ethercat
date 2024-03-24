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

The recommended way to install this driver is to use the `.deb` apt
repository managed by the Etherlab folks.  It should contain
everything that you need to install Ethercat support for LinuxCNC with
minimal manual work.

### Initial setup

First, you need to tell `apt` how to find the Etherlab repository,
hosted at https://build.opensuse.org/project/show/science:EtherLab.  This
is the preferred mechanism from the [LinuxCNC
forum](https://forum.linuxcnc.org/ethercat/45336-ethercat-installation-from-repositories-how-to-step-by-step):


```
sudo mkdir -p /usr/local/share/keyrings/
wget -O- https://build.opensuse.org/projects/science:EtherLab/signing_keys/download?kind=gpg | gpg --dearmor | sudo dd of=/etc/apt/trusted.gpg.d/science_EtherLab.gpg
sudo tee -a /etc/apt/sources.list.d/ighvh.sources > /dev/null <<EOT
Types: deb
Signed-By: /etc/apt/trusted.gpg.d/science_EtherLab.gpg
Suites: ./
URIs: http://download.opensuse.org/repositories/science:/EtherLab/Debian_12/
EOT
sudo apt update
sudo apt install -y linux-headers-$(uname -r) ethercat-master linuxcnc-ethercat
```

(These directions are for Debian 12.  Debian 11 should be very similar,
just change `Debian_12` to `Debian_11`.)

You will then need to do a bit of setup for Ethercat; at a minimum
you'll need to edit `/etc/ethercat.conf` to tell it which interface it
should use.  See the forum link (above) for additional details.

You can verify that Ethercat is working when `ethercat slaves` shows
the devices attached to your system.  See the forum link above for
additional helpful steps.

### Updates

Ongoing updates should be easy and *mostly* handled automatically by
`apt`.  Just run `sudo apt update` followed by `sudo apt upgrade` and
things will mostly work, with one possible exception.  If the kernel
is upgraded, then you *may* need to re-run this command in order to
get Ethercat working again:

```
sudo apt install -y linux-headers-$(uname -r)
```

This is because the real-time kernel that LinuxCNC prefers doesn't get
its headers installed by default, and this breaks compiling the
Ethercat modules for the new kernel.  Just run this `apt` command and
then either reboot or run `sudo systemctl start ethercat`.

### Manual Installation

If you decide that you want to install this manually and not use a
package manager, then first you'll need to make sure that you have the
[Ethercat Master](https://gitlab.com/etherlab.org/ethercat) and
LinuxCNC (with its development tools) installed.  Then download
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
