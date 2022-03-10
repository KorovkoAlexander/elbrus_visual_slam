GPU max performance setup
=========================

Run `sudo nvpmodel -q --verbose | grep GPU | grep freq` to get min and max GPU frequencies:

```
$ sudo nvpmodel -q --verbose | grep GPU | grep freq
NVPM VERB: PARAM GPU: ARG MIN_FREQ: PATH /sys/devices/17000000.gv11b/devfreq/17000000.gv11b/min_freq: REAL_VAL: 318750000 CONF_VAL: 318750000
NVPM VERB: PARAM GPU: ARG MAX_FREQ: PATH /sys/devices/17000000.gv11b/devfreq/17000000.gv11b/max_freq: REAL_VAL: 1377000000 CONF_VAL: 2147483647
```

To configure GPU for maximum preformance one can set both frequences to max real val (1377000000 in our example). 

Run `sudo nvpmodel -q` to figure out current power mode.

```
$ sudo nvpmodel -q
NV Fan Mode:quiet
NV Power Mode: MAXN
0
```

If current power mode is not MAXN (0) turn it on:

```
$ sudo nvpmodel -m 0
$ sudo nvpmodel -q
NV Fan Mode:quiet
NV Power Mode: MAXN
0
```

Power modes are defined in `/etc/nvpmodel.conf` configuration file. Make backup copy.

Open the file in text editor and find section "< POWER_MODEL ID=0 NAME=MAXN >". 

Change line "GPU MIN_FREQ ..." to "GPU MIN_FREQ 1377000000" and line "GPU MAX_FREQ ..." to "GPU MAX_FREQ 1377000000".

Diff in our example:

```
$ diff nvpmodel.conf.bak /etc/nvpmodel.conf
153,154c153,154
< GPU MIN_FREQ 318750000
< GPU MAX_FREQ -1
---
> GPU MIN_FREQ 1377000000
> GPU MAX_FREQ 1377000000
```

Apply changes and check result

```
$ sudo nvpmodel -m 0
$ sudo nvpmodel -q --verbose | grep GPU | grep freq
NVPM VERB: PARAM GPU: ARG MIN_FREQ: PATH /sys/devices/17000000.gv11b/devfreq/17000000.gv11b/min_freq: REAL_VAL: 1377000000 CONF_VAL: 1377000000
NVPM VERB: PARAM GPU: ARG MAX_FREQ: PATH /sys/devices/17000000.gv11b/devfreq/17000000.gv11b/max_freq: REAL_VAL: 1377000000 CONF_VAL: 1377000000
```

Now GPU is configured for max performance (and max power consumption).

To revert your changes restore `/etc/nvpmodel.conf` from backup and apply changes with `sudo nvpmodel -m 0`. Check your changes with `sudo nvpmodel -q --verbose | grep GPU | grep freq`.
