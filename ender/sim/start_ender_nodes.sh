#!/usr/bin/env bash
# Startet alle Nodes, die QGC braucht, in einem Befehl, jede in einem eigenen tmux-Fenster:
#
#   ender-bridge      bridge_node            Scan (UDP 19900) an die Bodenstation
#   ender-relay       map_relay_node         Pose und Karte (TCP 19902), auf Wunsch von QGC
#   ender-goal        goal_node              Ziele von QGC (UDP 19901) an SUPER
#   ender-compressor  pointcloud_compressor_node (Birk)  packt die Hindernis-Karte
#
# Aufruf (im Container, oder auf dem Onboard-PC):
#   ender/sim/start_ender_nodes.sh <IP der Bodenstation> [max_points]
#   ender/sim/start_ender_nodes.sh 192.168.178.42          # Sim: IP des Laptops, 3000 Punkte
#   ender/sim/start_ender_nodes.sh 192.168.144.20 6000     # echt: IP der Funke im Siyi-Netz
#   ender/sim/start_ender_nodes.sh --stop                  # alle vier Fenster schließen
#
# Läuft das Skript in tmux, entstehen die Fenster in der aktuellen Sitzung (im Hintergrund, es wird nicht
# umgeschaltet). Sonst entsteht die tmux-Sitzung "ender", an die sich das Skript anhängt.
# Zwischen den Fenstern wechselt man mit Strg+b und der Fensternummer, oder Strg+b n / p.
#
# Voraussetzung: der Workspace ist gebaut (colcon build --packages-select ender_lidar_bridge --symlink-install)
# und SUPER läuft (sl, launch_super).
#
# Umgebungsvariablen (alle optional):
#   ENDER_ROS_WS   Workspace mit ender_lidar_bridge  (Standard: /workspaces/ender-qgc/ender/ros2)
#   ENDER_SESSION  Name der tmux-Sitzung, falls das Skript nicht in tmux läuft  (Standard: ender)
#   ENDER_COMPRESSOR  0 = den Kompressor nicht starten (Standard: 1). Nötig, wenn Birks Launch-File ihn schon
#                  startet, zwei Kompressoren auf demselben Topic gehen nicht.
set -euo pipefail

WS="${ENDER_ROS_WS:-/workspaces/ender-qgc/ender/ros2}"
SESSION="${ENDER_SESSION:-ender}"
ALL_WINDOWS=(ender-bridge ender-relay ender-goal ender-compressor)
WINDOWS=(ender-bridge ender-relay ender-goal)
if [ "${ENDER_COMPRESSOR:-1}" != "0" ]; then
    WINDOWS+=(ender-compressor)
fi

usage() {
    sed -n '2,26p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-1}"
}

command -v tmux >/dev/null || { echo "Fehler: tmux ist nicht installiert." >&2; exit 1; }

# Fenster schließen. Außerhalb von tmux heißt das: die Sitzung beenden, falls es sie gibt.
stop_windows() {
    if [ -n "${TMUX:-}" ]; then
        local current
        current="$(tmux display-message -p '#S')"
        for name in "${ALL_WINDOWS[@]}"; do
            tmux kill-window -t "${current}:${name}" 2>/dev/null || true
        done
    fi
    tmux kill-session -t "$SESSION" 2>/dev/null || true
}

case "${1:-}" in
    -h|--help|"") usage "$([ -z "${1:-}" ] && echo 1 || echo 0)" ;;
    --stop) stop_windows; echo "Nodes beendet."; exit 0 ;;
esac

DEST_IP="$1"
MAX_POINTS="${2:-3000}"

# Die Shell, in der man sonst arbeitet (zsh im Container). Sie liest ihre Startdatei und kennt damit ROS.
SHELL_BIN="${SHELL:-/bin/bash}"
case "$(basename "$SHELL_BIN")" in
    zsh) SETUP="$WS/install/setup.zsh" ;;
    *)   SETUP="$WS/install/setup.bash" ;;
esac
if [ ! -f "$SETUP" ]; then
    echo "Fehler: ${SETUP} nicht gefunden. Erst bauen:" >&2
    echo "  cd ${WS} && colcon build --packages-select ender_lidar_bridge --symlink-install" >&2
    exit 1
fi

# Befehle der vier Nodes. Der Kompressor bekommt seine Werte per -p (Raster 0,4 m, 1 Hz): --params-file lehnt
# seine Node ab, und bei der Liste dürfen keine Anführungszeichen um den Eintrag stehen.
CMD_BRIDGE="ros2 run ender_lidar_bridge bridge_node --ros-args -p dest_ip:=${DEST_IP} -p max_points:=${MAX_POINTS}"
CMD_RELAY="ros2 run ender_lidar_bridge map_relay_node"
CMD_GOAL="ros2 run ender_lidar_bridge goal_node"
CMD_COMPRESSOR="ros2 run drone_control_cpp pointcloud_compressor_node --ros-args -p compression_level:=3 -p 'topic_keys:=[rog_map_inf_occ]' -p rog_map_inf_occ.input_topic:=/rog_map/inf_occ -p rog_map_inf_occ.downsample_leaf_size:=0.4 -p rog_map_inf_occ.max_frequency_hz:=1.0"

# Was im Fenster läuft: Workspace laden, Node starten, danach das Fenster offen lassen, damit man die Meldung liest
window_command() {
    printf 'source %q; %s; echo; echo "[%s beendet, Enter schließt das Fenster]"; read' "$SETUP" "$1" "$2"
}

stop_windows  # ein früherer Start würde sonst doppelt laufen

declare -A COMMANDS=(
    [ender-bridge]="$CMD_BRIDGE"
    [ender-relay]="$CMD_RELAY"
    [ender-goal]="$CMD_GOAL"
    [ender-compressor]="$CMD_COMPRESSOR"
)

first=1
for name in "${WINDOWS[@]}"; do
    full="$(window_command "${COMMANDS[$name]}" "$name")"
    if [ -n "${TMUX:-}" ]; then
        tmux new-window -d -n "$name" -- "$SHELL_BIN" -ic "$full"
    elif [ "$first" = 1 ]; then
        tmux new-session -d -s "$SESSION" -n "$name" -- "$SHELL_BIN" -ic "$full"
    else
        tmux new-window -d -t "${SESSION}:" -n "$name" -- "$SHELL_BIN" -ic "$full"
    fi
    first=0
    echo "gestartet: ${name}"
done

if [ -n "${TMUX:-}" ]; then
    echo "Fertig. Die Fenster ender-* laufen in dieser tmux-Sitzung (Strg+b w zeigt die Liste)."
else
    echo "Fertig. Hänge mich an die Sitzung '${SESSION}' an (Strg+b d löst sich wieder)."
    exec tmux attach -t "$SESSION"
fi
