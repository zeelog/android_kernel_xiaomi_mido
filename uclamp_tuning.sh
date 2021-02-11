#!/system/bin/sh

#Uclamp tuning
sysctl -w kernel.sched_util_clamp_min_rt_default=96
sysctl -w kernel.sched_util_clamp_min=128

#top-app
echo max > /dev/cpuset/top-app/uclamp.max
echo 20  > /dev/cpuset/top-app/uclamp.min
echo 1   > /dev/cpuset/top-app/uclamp.boosted
echo 1   > /dev/cpuset/top-app/uclamp.latency_sensitive

#foreground
echo 50 > /dev/cpuset/foreground/uclamp.max
echo 20 > /dev/cpuset/foreground/uclamp.min
echo 0  > /dev/cpuset/foreground/uclamp.boosted
echo 0  > /dev/cpuset/foreground/uclamp.latency_sensitive

#background
echo max > /dev/cpuset/background/uclamp.max
echo 20  > /dev/cpuset/background/uclamp.min
echo 0   > /dev/cpuset/background/uclamp.boosted
echo 0   > /dev/cpuset/background/uclamp.latency_sensitive

#system-background
echo 50 > /dev/cpuset/system-background/uclamp.max
echo 10 > /dev/cpuset/system-background/uclamp.min
echo 0  > /dev/cpuset/system-background/uclamp.boosted
echo 0  > /dev/cpuset/system-background/uclamp.latency_sensitive
