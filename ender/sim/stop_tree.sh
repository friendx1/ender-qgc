#!/usr/bin/env bash
# Beendet einen Prozess samt allen Unterprozessen, dessen PID in einer Datei steht (Starter-Dienst, ender/launcher).
#
# Aufruf im Container:  ender/sim/stop_tree.sh <pid-datei> [sekunden]
# Ablauf: erst SIGINT (wie Strg+C, ROS fährt geordnet herunter), nach der Wartezeit SIGTERM, zuletzt SIGKILL.
# Läuft der Prozess nicht mehr, endet das Skript ohne Fehler. Bleibt etwas übrig, steht es in der Ausgabe.
set -u

PIDFILE="${1:?Aufruf: stop_tree.sh <pid-datei> [sekunden]}"
WAIT="${2:-15}"

[ -f "$PIDFILE" ] || { echo "keine PID-Datei, läuft nicht"; exit 0; }
ROOT="$(cat "$PIDFILE")"
case "$ROOT" in '' | *[!0-9]*) echo "PID-Datei ist ungültig" >&2; exit 1 ;; esac

# Der Prozess und alle Nachkommen, jede Ebene über pgrep -P
tree() {
    local pid="$1" child
    echo "$pid"
    for child in $(pgrep -P "$pid" 2>/dev/null); do
        tree "$child"
    done
}

alive() {
    local pid
    for pid in "$@"; do
        kill -0 "$pid" 2>/dev/null && echo "$pid"
    done
}

kill -0 "$ROOT" 2>/dev/null || { rm -f "$PIDFILE"; echo "läuft nicht mehr"; exit 0; }

PIDS="$(tree "$ROOT")"
send() {
    local pid
    for pid in $PIDS; do kill "-$1" "$pid" 2>/dev/null; done
}

wait_gone() {
    local i
    for ((i = 0; i < $1 * 2; i++)); do
        [ -z "$(alive $PIDS)" ] && return 0
        sleep 0.5
    done
    return 1
}

send INT
wait_gone "$WAIT" || { send TERM; wait_gone 5; } || { send KILL; wait_gone 3; }

LEFT="$(alive $PIDS | tr '\n' ' ')"
if [ -n "$LEFT" ]; then
    echo "läuft noch: ${LEFT}" >&2
    exit 1
fi
rm -f "$PIDFILE"
echo "beendet"
