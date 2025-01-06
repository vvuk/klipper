# eddy-ng

_eddy-ng_ is my attempt at improving Eddy current probe support in Klipper. These probes are very accurate, but suffer from thermal drifts due to changes in conductivity in the target surface as well as changes in coil parameters as temperatures change. Instead of doing temperature compensation, the approach _eddy-ng_ takes is this:

1. Calibration will be performed at any temperature (cold).
2. Z-homing via the eddy current sensor will always happen using this cold calibration. This is a "coarse" Z-home.
3. A precise Z-home (thus "Z-offset") will be done using a "tap" just before printing, with the bed at print temps, and the nozzle warm (but not hot -- you don't want filament drooling).
4. This offset will also be taken into account when doing a bed mesh, because it indicates the delta (due to temperatures) between what height the sensor thinks it is vs. where it actually is.

This means that there will be nozzle-to-bed contact during step 3. The max limit of this contact is configurable; by default, it is _at most_ z=-0.250. This is not any worse than manually setting Z and stepping down one notch too far, and in general successful taps will not go beyond -0.050 or -0.100.

> WARNING: _THIS IS A WORK IN PROGRESS_. It may drill your nozzle into your build plate. It may set your printer on fire. It may set _you_ on fire. It has been tested so far on a sample of 2 printers: one (mine) where it works fine, and a second where it has some issues, though they may be unrelated to this code. More testing is needed!

This has only been tested with a BTT Eddy Duo. _For now, I believe in order for tap to work, the Eddy coil must be mounted about 2.95mm above the tip of the nozzle, +/- 0.15mm._ This is only based on a sample size of one -- if it's mounted too high or too low, homing will still work, but tap may not be accurate or cause errors. (See troubleshooting later if you do. The data for these errors is still useful though! So feel free to try things out even if your eddy is mounted higher.) I'm not 100% sure about this though, and need to do more testing. I'm setting up a sacrificial Ender 3 for said testing soon. :) 

> WARNING: Again, this is at a "needs early testers" stage. But if you do get adventurous and want to try things out, please let me know how things go. The best place is [this thread](https://discord.com/channels/788873913953157131/1322806494714003556) on the Sovol Printers Discord, or you can file issues in my github repo (do _not_ file in upstream klipper): [https://github.com/vvuk/klipper/issues](https://github.com/vvuk/klipper/issues))

## Klipper setup

This repository is a fairly recent Klipper. It's easiest to use it directly if you're already on mainline klipper. You can do this by adding my repository as a remote and checking out this branch:

```
cd ~/klipper
git remote add vlad-eddy http://github.com/vvuk/klipper
git fetch vlad-eddy
git checkout vlad-eddy/vlad/eddy-ng
```

If you'd like to integrate this code into your own version, assuming it is similarly recent, you will need to copy in:
  * `src/ldc1612_ng.c`, `src/printf.c`, `src/printf.h`
  * `klippy/extras/probe_eddy_ng.py`, `klippy/extras/ldc1612_ng.py`
  * you will need to edit two files:
    * `src/Makefile`: add `printf.c` to `src-y`, and `sensor_ldc1612_ng.c` to the end of the `src..LDC1612` line
    * `klippy/extras/bed_mesh.py`: search for the `can_scan = ...` line and change it to `can_scan = "eddy" in probe_name`

This has only been tested with BTT Eddy Duo. In theory it should/will work with Cartographer etc., but that hasn't been done yet (and some of the default values will not work). Eddy Coil/USB should work fine as well, but has not been tested.

Build and flash your BTT Eddy like normal. If you were on a wildly different Klipper you may need to flash the rest of your MCUs.

If you install `plotly` in your klipper python venv (often `~/klippy-venv/bin/pip3 install plotly`), you'll get some handy graphs generated for calibration and tap operations that can help diagnose issues.

## Configuration

There's a configuration reference at the top of `klippy/extras/probe_eddy_ng.py`. Many values are configurable.

The original BTT Eddy code is retained. You should be able to swap between the original eddy code and this just by changing configuration sections. If you want to do this, I recommend making two files, one for this config, and one for the original eddy and then including the appropriate one. When you switch, Klipper will give errors about missing required config options for `probe_eddy_current` and/or `probe_eddy_ng`. This is because the saved values section "defines" these sections, even if they're not present elsewhere. The easiest way around this is to copy the saved values directly into your config (removing the `#*#` prefix).

Replace your `probe_eddy_current` config section with something like this (BTT Eddy values):

```
[probe_eddy_ng btt_eddy]
sensor_type: ldc1612
reg_drive_current: 15
# number of samples to fetch per second
samples_per_second: 500
i2c_mcu: eddy
i2c_bus: i2c0f
# x/y probe offsets
x_offset: -16.0
y_offset: 11.5
```
Change your `bed_mesh` and `quad_gantry_level` sections' `horizontal_move_z` to be `2.0` -- this parameter sets the height that bed meshing is done at, and it's best if it's the same as the home trigger height which is `2.0` by default.

### Coming from BTT's configs

You don't need/want most of the macros in BTT's eddy config files. You do need the hardware config. The sections you need are:

```
[mcu eddy]
[temperature_sensor btt_eddy_mcu]
[temperature_probe btt_eddy]
```

in addition to the new `probe_eddy_ng` section added above. Comment out/remove all the macros, and add a new SET_Z_FROM_PROBE macro:

```
[gcode_macro SET_Z_FROM_PROBE]
gcode:
    {% set probe_z_actual = printer.probe.last_z_result %}
    RESPOND TYPE=echo MSG="Set Z={probe_z_actual}"
    SET_KINEMATIC_POSITION Z={probe_z_actual}
```

## Initial Setup and Homing

After flashing and configuring everything, run:

```
PROBE_EDDY_NG_STATUS
```

(`PES` for short -- there's a few short commands because I've been typing them a ton)

If you see something like:

```
Last coil value: 3189314.38 (-infmm) raw: 0x4409e8b (Not calibrated) status: 0x48 UNREADCONV1 DRDY
```
or `status: 0x0`, you're good. If you see `raw: 0x0` then no data's coming through yet. Give it a sec and try `PES` a a few more times.

Then, home XY: `G28 XY`

Move your toolhead to the center or to some convenient paper-calibration position. Start calibration -- do this cold.

```
PROBE_EDDY_NG_CALIBRATE
```

You don't have to be super precise here -- doesn't have to be the perfect friction feel of the paper. Just "good enough" is fine. Calibration writes out `/tmp/eddy-calibrate.csv` -- this file is useful for debugging, as well as `/tmp/eddy-calibrate.html` if you have Plotly installed.

If everything completes successfully, don't save yet! Run:

```
PROBE_EDDY_NG_PROBE_STATIC      (PEPS for short)
```

You should see a reasonable height come back.

Now, you can try homing Z: ***this is the point at which you should be most careful and watch your printer with a finger on the e-stop in case the print head starts drilling down to Narnia***:

```
G28 Z
```

Everything good? After this, if you move the toolhead to the 1mm-5mm range, the probed height you get back from `PEPS` should be close to the toolhead's Z position, within +/- 0.025 or so. It should be most accurate in the 1mm-3mm range. If it's not, try recalibrating.

## Safety/functionality checks

`PROBE_ACCURACY` should work fine, and should report back a fairly accurate probe.

`QUAD_GANTRY_LEVEL` should succeed. If it doesn't, verify that `horizontal_move_z` matches the `home_trigger_height` (default 2.0).

`BED_MESH_CALIBRATE METHOD=rapid_scan` should succeed. If it doesn't, verify that `horizontal_move_z` matches `home_trigger_height`.

## Tap testing and calibration

The "tap threshold" is essentially how long to wait after the system thinks that the toolhead has physically stopped moving down. It's not based on time, but instead on the rate of change of the sensor's frequency values. Higher threshold values will result in a longer "wait" before a tap is recognized.

The default `TARGET_Z` is `-0.250` and the default threshold is `1000`. These default values are configurable in the settings, or you can specify them to the commands to fine tune.

With XYZ homed, try:

```
PROBE_EDDY_NG_TAP TARGET_Z=-0.150 THRESHOLD=1000
```

This will make contact with your build plate (maximum contact -0.150, at a default 3mm/s; the risk of build plate or nozzle damage is minimal). If it is successful, it will set a Z-offset as well as a "tap offset", which is used for more accurate bed meshing following a tap.

This command will write out `/tmp/tap-samples.csv`. Save a few copies of this file after every tap, successful or unsuccessul and send to me -- this will help fine-tune the algorithm. If you install Plotly, you will also have `/tmp/tap.html` that you can look at.

Lower threshold values are beneficial only in order to have the tap operation make shorter contact with the build plate, but lower values also increase the chance of false positives. The graph should show you the current "accumulator" value, which is what's compared to the threshold. You can eyeball an appropriate threshold for your setup. (TODO: docs about reading the graphs)

After a successful tap, you should be able to do a `BED_MESH_CALIBRATE METHOD=rapid_scan`. If everything is working correctly, the bed mesh deviation at the tap point should be very close to 0 (+/-0.005).

### Adjusting Tap Z

When the toolhead comes into contact with the build plate and the steppers keep moving, the entire system takes on a small amount of flex. Depending on threshold, stiffness of the rest of the system, and other factors, the detected tap Z offset may be (usually) slightly too low or (rarely) too high. You can use the config parameter `tap_adjust_z` to specify a fixed value that will be added to the calculated Z offset when tapping, e.g.:

```
tap_adjust_z: 0.025
```

## Macros

### Homing

Your Z homing sequence should look something like this (via `homing_override`):

```
    G28 Z                      ; do the coarse home
    G90                        ; set absolute positioning
    G0 Z2 F1000                ; to 2mm -- home height, for maximum sensor accuracy
    G4 S1                      ; chill for a sec
    M400                       ; wait for move to finish
    PROBE_EDDY_NG_PROBE_STATIC ; read the current exact height from sensor
    SET_Z_FROM_PROBE           ; set Z
    G0 Z5 F1000 ; to 5mm as home
```

The probe and set_z_from_probe aren't strictly necessary, as tap should take care of
setting an accurate offset.

### Start Print

A good start print flow is:

1. Home (G28)
2. Heat soak
3. QGL
4. Clean nozzle at 150C (most nozzle cleans are done at 200C! adjust your macro to let you specify temperature)
5. "Smart Park" or return to center/Z position
6. `PROBE_EDDY_NG_TAP` 
7. Clean nozzle again at regular temp 
8. Bed mesh
9. Print

### End Print

`PROBE_EDDY_NG_TAP` sets both the z offset and a "tap offset" that's applied to bed mesh and other probe operations.
These are only valid for the previous tap. I suggest clearing the tap offset at the end of a print (and probably the
z offset as well) to avoid confusion:

```
PROBE_EDDY_NG_SET_TAP_OFFSET VALUE=0
```

I also `BED_MESH_CLEAR` here, because I build an adaptive bed mesh, and it doesn't make sense to keep it around.

## Troubleshooting

### "Already sampling" error

If you see errors like this:

```
EDDYng Already sampling
```

This often happens after a `QUAD_GANTRY_LEVEL` aborts due to increasing offset values. Unfortunately the way
that error is reported doesn't seem to go through the normal command error paths. A `RESTART` is required.
(TODO: add a `PROBE_EDDY_NG_STOP_SAMPLING`) If you see this and there wasn't an aborted QGL or bed mesh,
please file a bug.

### Sensor errors during homing

If you see errors like this:

```
Error during homing z: Sensor error (ldc status: UNREADCONV1 ERR_ALE)
```

This indicates an "amplitude error", and is usually caused by one of two things:

- Eddy coil mounted at wrong offset from nozzle (either too high or to low)
- Wrong drive current is set for coil

The default drive current is 15. In some cases, some additional current can help, but it can also introduce errors in the other direction.
You can set the drive current via the `reg_drive_current` config option. You need to do a separate calibration for each drive current. `PROBE_EDDY_NG_CALIBRATE`
will calibrate whatever the current dirve current is, but you can also specify `DRIVE_CURRENT=16` to calibrate a specific drive current.

### Sensor errors during tap operations

These are the same cause as errors during homing, but if the errors appear only during tapping, you can increase your drive current for tap operationns only, e.g. in your config:

```
reg_drive_current: 15
tap_drive_current: 16
```

If you set a different drive current for tap, you will need to explicitly calibrate it via `PROBE_EDDY_NG_CALIBRATE DRIVE_CURRENT=16`.

### Tap stopping too early with the toolhead in the air

This is caused by the tap threshold being too low and triggering before nozzle-plate contact occurs. Looking at the plot can help identify what's going on.

Also verify that there is no filament on your nozzle. Tap detects physical contact, but if there's a bit of filament at the tip, that will count, too.

### Tap erroring with "Probe completed movement before triggering"

This is caused by one of two things:

1. The tap threshold is too low, and a tap isn't detected before the toolhead reaches the target position (default -0.250).
2. The base sensor calibration is too far off, and actual zero is lower than -0.250 on the Z axis.

To check the second one, put a piece of paper under the toolhead and manually move the toolhead to 0. Then keep stepping it downwards by 0.1 until you can no
longer move the paper (or it's very difficult). If this is below -0.250, you can redo calibration, or you can set a lower tap target Z; but be aware that this
increases the amount of nozzle-build plate contact.

If it's not the second one, then looking at the plot can help identify what's going on with the threshold value.
