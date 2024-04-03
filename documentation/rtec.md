# RTelligent ECR and ECT Stepper Drivers

The `lcec_rtec` driver supports a number of
[RTelligent](http://www.rtelligent.net/index.htm) EtherCAT stepper
drives, including the ECT60 and ECR60.

## Setup

In your XML file, you should have an entry somewhat like this:

```xml
<masters>
  <master idx="0" appTimePeriod="2000000" refClockSyncCycles="1000">
    ...
    <slave idx="1" type="ECT60" name="x-axis">
      <modParam name="peakCurrent_amps" value="1.0"/>
      <modParam name="controlMode" value="closedloop"/>
      <modParam name="homeOffset" value="1"/>
    </slave>
	...
  </master>
</masters>
```

See the [CiA 402](cia402.md) documentation for additional details
about how to configure CiA 402 devices in LinuxCNC.  At a minimum, you
will need to include the `cia402` HAL component.

## Devices

This driver supports all known (as of early 2024) RTelligent EC-series
EtherCAT stepper drives, including:

- [ECR60](http://www.rtelligent.net/ECR60.html) (one-axis 6A open-loop driver)
- [ECR60x2](https://www.rtelligentglobal.com/fieldbus-open-loop-stepper-drive-ecr60x2a-product/) (two-axis 6A open-loop driver)
- [ECR86](http://www.rtelligent.net/ECR86.html) (one-axis 7A open-loop driver)
- [ECT60](http://www.rtelligent.net/ECT60.html) (one-axis 6A closed-loop driver)
- [ECT60x2](https://www.rtelligentglobal.com/fieldbus-open-loop-stepper-drive-ect60x2-product/) (two-axis 6A closed-loop driver)
- [ECT86](http://www.rtelligent.net/ECT86.html) (one-axis 7A closed-loop driver)

Only the ECT60 and ECR60x2 have been tested.

### Caveats

My ECR60x2 (technically an EXR60X2A) has a few EtherCAT shortcomings
that moderately impact its utility.  First, `ethercat sdos` doesn't
show anything for the device.  This makes is somewhat tricky to
develop against, but shouldn't impact its use.  More annoyingly, it
maps PDOs at 0x16n0 and 0x1An0 instead of 0x160n and 0x1A0n, so a bit
of special configuration is needed to get the second axis to work.
Finally, it only appears to support 2 TX PDOs, which means that we've
had to disable some optional features to save PDO space.

From the docs, it appears that in closed-loop mode, EC* drivers
measure distances in terms of encoder steps (usually 4000x per
revolution), and in open-loop mode they measure in terms of
`motorResolution_pulses` (usually 10000x per revolution).  There does
not appear to be a way to change these at the drive level, so scaling
will need to be applied at some point in LinuxCNC to get reasonable
distance units.

## Configuration

This driver takes a number of `<modParam>` options that control its
operation.  There are a number of `rtec`-specific parameters, plus a
number of additional [`cia402` modParams](cia402.md).

The parameters are listed in their single-axis form; dual-axis devices
like the ECR60X2 and ECT60X2 have two sets of these parameters, one
starting with `ch1` and one starting with `ch2`.  For example,
`ch1peakCurrent_amps` and `ch2peakCurrent_amps`.

<dl>
<dt>&lt;modParam name="peakCurrent_amps" value="3.0"/&gt</dt>
<dd>Set the maximum current for the motor, in amps.</dd>

<dt>&lt;modParam name="motorResolution_pulses" value="10000"/&gt</dt>
<dd>Set the number of pulses for a full motor rotation.</dd>

<dt>&lt;modParam name="standbyTime_ms" value="500"/&gt</dt>
<dd>Set the standby time, before the drive drops down to standby current.  See below.</dd>

<dt>&lt;modParam name="standbyCurrent_pct" value="50"/&gt</dt>
<dd>Set the percentage of current used when in standby mode.</dd>

<dt>&lt;modParam name="output1Func" value="general|alarm|brake|in-place"/&gt</dt>
<dd>Set the function for output port 1.  See below for options.</dd>

<dt>&lt;modParam name="output2Func" value="general|alarm|brake|in-place"/&gt</dt>
<dd>Set the function for output port 2.  See below for options.</dd>

<dt>&lt;modParam name="output1Polarity" value="nc|no"/&gt</dt>
<dd>Set the polarity for output port 1, either <tt>nc</tt> for
normally-closed or <tt>no</tt> for normally open.</dd>

<dt>&lt;modParam name="output2Polarity" value="nc|no"/&gt</dt>
<dd>Set the polarity for output port 2, either <tt>nc</tt> for
normally-closed or <tt>no</tt> for normally open.</dd>

<dt>&lt;modParam name="input3Func" value="general|..."/&gt</dt>
<dd>Set the function for input port 3.  See below for options.</dd>

<dt>&lt;modParam name="input4Func" value="general|..."/&gt</dt>
<dd>Set the function for input port 4.  See below for options.</dd>

<dt>&lt;modParam name="input5Func" value="general|..."/&gt</dt>
<dd>Set the function for input port 5.  See below for options.</dd>

<dt>&lt;modParam name="input6Func" value="general|..."/&gt</dt>
<dd>Set the function for input port 6.  See below for options.</dd>

<dt>&lt;modParam name="input3Polarity" value="nc|no"/&gt</dt>
<dd>Set the polarity for input port 3, either <tt>nc</tt> for
normally-closed or <tt>no</tt> for normally open.</dd>

<dt>&lt;modParam name="input4Polarity" value="nc|no/&gt</dt>
<dd>Set the polarity for input port 4, either <tt>nc</tt> for
normally-closed or <tt>no</tt> for normally open.</dd>

<dt>&lt;modParam name="input5Polarity" value="nc|no/&gt</dt>
<dd>Set the polarity for input port 5, either <tt>nc</tt> for
normally-closed or <tt>no</tt> for normally open.</dd>

<dt>&lt;modParam name="input6Polarity" value="nc|no/&gt</dt>
<dd>Set the polarity for input port 6, either <tt>nc</tt> for
normally-closed or <tt>no</tt> for normally open.</dd>

<dt>&lt;modParam name="filterTime_us"  value="6400"/&gt</dt>

<dd>Set the filter time in microseconds.</dd>

<dt>&lt;modParam name="shaftLockTime_us" value="1000"/&gt</dt>

<dd>Set the shaft lock time in microseconds.</dd>

<dt>&lt;modParam name="motorAutoTune" value="true|false"/&gt</dt>

<dd>Enable or disable motor autotuning.</dd>

<dt>&lt;modParam name="stepperPhases" value="2"/&gt</dt>

<dd>Set the number of stepper phases for the motor.</dd>

<dt>&lt;modParam name="controlMode"  value="openloop|closedloop|foc"/&gt</dt>

<dd>Set the control mode for the motor.</dd>

<dt>&lt;modParam name="encoderResolution" value="4000"/&gt</dt>

<dd>The number of encoder pulses per revolution.</dd>

<dt>&lt;modParam name="positionErrorLimit" value="4000"/&gt</dt>
</dl>

## Inputs

The single-axis ECR/ECT devices have 6 inputs.  Inputs 1 and 2 are
used for the encoder (and don't appear to be available on the
open-loop ECR models), while inputs 3, 4, 5, and 6 are available for
use.

The `inputXFunc` (where X is 3, 4, 5, or 6) modParam controls the function of each input.  The available options are:

<dl>
<dt>general</dt>
<dd>The input is available to be used as a general-purpose input.  Use
this if you want LinuxCNC to take actions based on the state of the
input rather than having the stepper drive itself take action.</dd>

<dt>cw-limit</dt>
<dd>The input is used for the clockwise limit/homing sensor, and is
automatically used as part of CiA 402 homing mode.</dd>

<dt>ccw-limit</dt>
<dd>The input is used for the counter-clockwise limit/homing sensor,
and is automatically used as part of CiA 402 homing mode.</dd> 

<dt>home</dt>
<dd>The input is used for the home sensor, and is presumably used as
part of CiA 402 homing mode.</dd> 

<dt>clear-fault</dt>
<dd>The input is used to clear faults.</dd>

<dt>emergency-stop</dt>
<dd>The input is used as an emergency stop.</dd>

<dt>motor-offline</dt>
<dd>The input is used to take the motor offline.</dd>

<dt>probe1<dt>
<dd>The input is connected to probe #1.</dd>

<dt>probe2<dt>
<dd>The input is connected to probe #2.</dd>
</dl>

On my ECT60, the pins come pre-assigned with various functions:

- input 3 is `cw-limit`
- input 4 is `ccw-limit`
- input 5 is `home`
- input 6 is `motor-offline`

If you want to use these as general-purpose inputs, then you'll want
to add `<modParam name="input3Func" value="general"/>` to your XML
config.

On my system, all of the inputs default to "normally-closed"
operation.  You may also want to change this via `<modParam
name="input3Polarity" value="nc"/>`.

## Outputs

The single-axis ECR/ECT devices have 2 outputs.  The `output1Func` and `output2Func` modParams control the function of each output.  The available options are:

<dl>
<dt>general</dt>
<dd>This output is a general-purpose output, available to LinuxCNC.</dd>
<dt>alarm</dt>
<dd>This output is triggered automatically when the drive triggers an alarm.</dd>
<dt> brake</dt>
<dd>This output is triggered automatically when the drive wishes to
engage the brake.</dd>
<dt>in-place</dt>
<dd>This output is triggered automatically when the motor has reached
its target position.</dd>
</dl>

On my ECT60, the pins come pre-assigned as:

- output 1 is `alarm`
- output 2 is `brake`

To use these as general-purpose outputs, you'll probably want to add `<modParam name="output1Func" value="general">`.

Like the input pins, you can also change the polarity between
normally-open and normally-closed using `<modParam
name="output1Polarity" value="no"/>`.  My ECT60 defaults to `nc` on
both ports.
