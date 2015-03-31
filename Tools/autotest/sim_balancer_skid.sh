#!/bin/bash

set -x

killall -q APMbalancer.elf
pkill -f sim_rover.py
set -e

autotest=$(dirname $(readlink -e $0))
pushd $autotest/../../APMbalancer
make clean sitl

tfile=$(mktemp)
(
echo r
) > $tfile
#gnome-terminal -e "gdb -x $tfile --args /tmp/APMbalancer.build/APMbalancer.elf"
gnome-terminal -e /tmp/APMbalancer.build/APMbalancer.elf
#gnome-terminal -e "valgrind --db-attach=yes -q /tmp/APMbalancer.build/APMbalancer.elf"
sleep 2
rm -f $tfile
#gnome-terminal -e "../Tools/autotest/pysim/sim_rover.py --home=-35.362938,149.165085,584,270 --rate=400"
gnome-terminal -e "../Tools/autotest/pysim/sim_rover.py --skid-steering --home=-35.362938,149.165085,584,270 --rate=400"
sleep 2
popd
mavproxy.py --aircraft=test --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551
