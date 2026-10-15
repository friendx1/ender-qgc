#!/usr/bin/env bash
# Misst, wie viele Daten der Linux-Rechner an die Funke schickt, aufgeschluesselt nach Ender-Port.
#
#   ender/tools/measure_bandwidth.sh <IP der Funke> [Sekunden]      (Standard: 20 s)
#
# Ports: 19900 = Scan (UDP), 19901 = Ziel-Quittungen (UDP), 19902 = Karte, Pose, Bahn (TCP).
# Gezaehlt wird die Nutzlast ohne IP/UDP/TCP-Kopf (je Paket etwa 28 bis 66 Byte mehr auf der Leitung),
# bei 1200-Byte-Paketen ist der Unterschied klein. Braucht tcpdump und sudo.
set -euo pipefail

HOST="${1:?Aufruf: $0 <IP der Funke> [Sekunden]}"
SECS="${2:-20}"

command -v tcpdump >/dev/null || { echo "tcpdump fehlt: sudo apt install tcpdump"; exit 1; }
IFACE="$(ip route get "$HOST" | awk '{for (i = 1; i <= NF; i++) if ($i == "dev") { print $(i + 1); exit }}')"
[[ -n "$IFACE" ]] || { echo "Keine Schnittstelle zu $HOST gefunden"; exit 1; }

echo "Messe $SECS s auf $IFACE, alles was an $HOST geht ..."
# timeout beendet tcpdump nach SECS, awk bekommt dann das Ende der Eingabe und druckt die Summen.
sudo timeout "$SECS" tcpdump -i "$IFACE" -nn -q -l "ip and dst host $HOST" 2>/dev/null |
  awk -v secs="$SECS" '
    function port_of(addr,   n, parts) {   # "192.168.178.50.43210" oder "...19900:" -> Port
      sub(/:$/, "", addr)
      n = split(addr, parts, ".")
      return parts[n]
    }
    function known(p) { return p == 19900 || p == 19901 || p == 19902 }
    {
      src = port_of($3)
      dst = port_of($5)
      key = known(src) ? src : (known(dst) ? dst : "andere")
      len = $NF + 0
      bytes[key] += len
      pkts[key]++
      total += len
      allpkts++
    }
    END {
      printf "\n%-10s %10s %12s %12s\n", "Port", "Pakete/s", "kB/s", "kbit/s"
      for (k in bytes) {
        printf "%-10s %10.1f %12.1f %12.0f\n", k, pkts[k] / secs, bytes[k] / secs / 1000, bytes[k] * 8 / secs / 1000
      }
      printf "%-10s %10.1f %12.1f %12.0f\n", "gesamt", allpkts / secs, total / secs / 1000, total * 8 / secs / 1000
    }'
