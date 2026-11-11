#!/usr/bin/env bash
# Start the picosat ejection sequence in picosat_deploy.sdf.

if ! command -v gz >/dev/null 2>&1; then
    source /opt/ros/lyrical/setup.bash
fi

gz topic -t /eject -m gz.msgs.Empty -p ' '
