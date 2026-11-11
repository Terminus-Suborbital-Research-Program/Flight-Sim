#!/usr/bin/env bash
# Point the Gazebo GUI camera at a picosat while picosat_deploy.sdf is running.
#
#   ./camera.sh a [dist]       follow picosat_a (camera rides along behind it)
#   ./camera.sh b [dist]       follow picosat_b
#   ./camera.sh watch-a        camera stays put, turns to keep picosat_a in view
#   ./camera.sh watch-b        same for picosat_b
#   ./camera.sh free           stop tracking, normal mouse control
#
# dist scales the follow distance (default 1.0 ~= 0.5 m behind and above).
# Follow mode orbits with the picosat's spin, so the spin is hidden; use
# watch-a, watch-b or free to see it.

if ! command -v gz >/dev/null 2>&1; then
    source /opt/ros/lyrical/setup.bash
fi

set -euo pipefail

mode="${1:-}"
dist="${2:-1.0}"

send() {
    gz topic -t /gui/track -m gz.msgs.CameraTrack -p "$1"
}

follow() {
    local target="$1"
    # Offset is in the picosat's own frame: behind (-x), to the side, above.
    local x y z
    x=$(awk -v d="$dist" 'BEGIN{print -0.4*d}')
    y=$(awk -v d="$dist" 'BEGIN{print -0.25*d}')
    z=$(awk -v d="$dist" 'BEGIN{print 0.25*d}')
    send "track_mode: FOLLOW_LOOK_AT,
          follow_target: {name: \"$target\"}, track_target: {name: \"$target\"},
          follow_offset: {x: $x, y: $y, z: $z},
          follow_pgain: 1.0, track_pgain: 1.0"
}

watch() {
    send "track_mode: TRACK, track_target: {name: \"$1\"}, track_pgain: 1.0"
}

case "$mode" in
    a)       follow picosat_a ;;
    b)       follow picosat_b ;;
    watch-a) watch picosat_a ;;
    watch-b) watch picosat_b ;;
    free)    send "track_mode: NONE" ;;
    *)
        sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
        exit 1
        ;;
esac
