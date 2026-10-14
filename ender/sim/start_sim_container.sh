#!/usr/bin/env bash
# Startet den Simulations-Container der Drohnen-Software (Repo drone-project) mit NVIDIA-Grafik.
#
# Nutzt das schon gebaute Image aus dem devcontainer, ohne etwas im drone-project-Repo zu ändern:
#   - drone-project wird nach /workspaces/drone-project gemountet (wie im devcontainer)
#   - dieses Repo (ender-qgc) wird nach /workspaces/ender-qgc gemountet
#   - Host-Netzwerk, damit UDP 19901 (Ziele) und 19900 (Punktwolke) vom Laptop erreichbar sind
#
# Aufruf auf dem Linux-Rechner:  ender/sim/start_sim_container.sh
# Läuft der Container schon, öffnet der Aufruf nur ein weiteres Terminal darin.
#
# Umgebungsvariablen (alle optional):
#   ENDER_SIM_IMAGE   Image-Name   (Standard: neuestes vsc-drone-project-*-uid Image)
#   DRONE_PROJECT_DIR Pfad zu drone-project (Standard: ~/drone-project)
set -euo pipefail

NAME="ender-sim"
ENDER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DRONE_DIR="${DRONE_PROJECT_DIR:-$HOME/drone-project}"

if [ "$(docker ps -q -f "name=^${NAME}$")" ]; then
    echo "Container ${NAME} läuft schon, öffne ein weiteres Terminal darin."
    exec docker exec -it -u devuser -w /workspaces/drone-project "$NAME" zsh
fi

if [ ! -d "$DRONE_DIR/ros2_ws" ]; then
    echo "Fehler: ${DRONE_DIR}/ros2_ws nicht gefunden. Pfad mit DRONE_PROJECT_DIR angeben." >&2
    exit 1
fi

IMAGE="${ENDER_SIM_IMAGE:-}"
if [ -z "$IMAGE" ]; then
    # Neuestes devcontainer-Image von VS Code (Variante mit angepasster Benutzer-ID)
    IMAGE="$(docker images --format '{{.CreatedAt}}|{{.Repository}}:{{.Tag}}' \
        | grep -E '\|vsc-drone-project-.*-uid:latest$' | sort -r | head -n1 | cut -d'|' -f2)"
fi
if [ -z "$IMAGE" ]; then
    echo "Fehler: kein Simulations-Image gefunden. In VS Code einmal 'Reopen in Container' ausführen" >&2
    echo "oder ENDER_SIM_IMAGE setzen." >&2
    exit 1
fi
echo "Image: ${IMAGE}"

# Erlaubt dem Container, Fenster auf dem Bildschirm zu öffnen (Gazebo, RViz)
if command -v xhost >/dev/null 2>&1; then
    xhost +local: >/dev/null
else
    echo "Hinweis: 'xhost' fehlt (sudo apt install x11-xserver-utils), Fenster öffnen sich evtl. nicht." >&2
fi

exec docker run -it --rm --name "$NAME" \
    --gpus all \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    --net=host --ipc=host --shm-size=2gb \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "${DRONE_DIR}:/workspaces/drone-project" \
    -v "${ENDER_DIR}:/workspaces/ender-qgc" \
    -u devuser \
    -w /workspaces/drone-project \
    "$IMAGE" zsh
