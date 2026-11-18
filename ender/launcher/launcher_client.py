#!/usr/bin/env python3
"""Test client for the Ender launcher: sends a command and prints the state.

    python launcher_client.py 127.0.0.1 status
    python launcher_client.py 127.0.0.1 start --watch     # run all steps that are not done yet, follow them
    python launcher_client.py 127.0.0.1 step 2 --watch    # run only step 2 (number as in status, or its name)
    python launcher_client.py 127.0.0.1 step Planer --watch
    python launcher_client.py 127.0.0.1 stop-step 3       # stop only step 3 (if the profile says how)
    python launcher_client.py 127.0.0.1 stop

The same datagrams the ground station sends. Uses only the standard library.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time

DEFAULT_PORT = 19903
SYMBOLS = {"pending": " ", "running": "…", "ok": "✓", "failed": "✗"}


def ask(
    host: str,
    port: int,
    command: str,
    seq: int,
    token: str = "",
    attempts: int = 5,
    step: int | None = None,
) -> dict | None:
    """Sends a request and waits for the answer with the same seq. A lost datagram is simply sent again."""
    request = {"v": 1, "seq": seq, "cmd": command}
    if step is not None:
        request["step"] = step
    if token:
        request["token"] = token
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(0.7)
        for _ in range(attempts):
            sock.sendto(json.dumps(request).encode(), (host, port))
            try:
                data, _ = sock.recvfrom(2048)
            except TimeoutError:
                continue
            try:
                reply = json.loads(data)
            except ValueError:
                continue
            if isinstance(reply, dict) and reply.get("seq") == seq:
                return reply
    return None


def show(reply: dict) -> None:
    print(
        f"\n[{reply.get('profile', '?')}] {reply.get('state', reply.get('error', '?'))}  {reply.get('message', '')}"
    )
    for index, step in enumerate(reply.get("steps", [])):
        stop = "  (einzeln stoppbar)" if step.get("can_stop") else ""
        print(f"  {index} {SYMBOLS.get(step['state'], '?')} {step['name']}  {step['detail']}{stop}")


def resolve_step(reply: dict, wanted: str) -> int | None:
    """Turns a step number or (part of) a name into the number of the step."""
    names = [step["name"] for step in reply.get("steps", [])]
    if wanted.isdigit() and int(wanted) < len(names):
        return int(wanted)
    matches = [i for i, name in enumerate(names) if wanted.lower() in name.lower()]
    return matches[0] if len(matches) == 1 else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n", maxsplit=1)[0])
    parser.add_argument("host")
    parser.add_argument("command", choices=["start", "stop", "status", "step", "stop-step"])
    parser.add_argument("step", nargs="?", help="step number or name (for step and stop-step)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--token", default="")
    parser.add_argument(
        "--watch",
        action="store_true",
        help="keep asking until the state no longer changes by itself",
    )
    args = parser.parse_args()
    needs_step = args.command in ("step", "stop-step")
    if needs_step != (args.step is not None):
        parser.error("step and stop-step need a step number or name, the others do not")

    seq = int(time.time()) % 1_000_000
    step_index = None
    if needs_step:
        status = ask(args.host, args.port, "status", seq, args.token)
        if status is None:
            print(
                "keine Antwort (läuft der Dienst, stimmen Adresse, Port und Firewall?)",
                file=sys.stderr,
            )
            return 1
        step_index = resolve_step(status, args.step)
        if step_index is None:
            show(status)
            print(f"\nSchritt '{args.step}' ist nicht eindeutig oder unbekannt", file=sys.stderr)
            return 1
        seq += 1
    command = args.command.replace("-", "_")
    reply = ask(args.host, args.port, command, seq, args.token, step=step_index)
    if reply is None:
        print(
            "keine Antwort (läuft der Dienst, stimmen Adresse, Port und Firewall?)", file=sys.stderr
        )
        return 1
    show(reply)
    while args.watch and reply.get("state") in ("starting", "stopping"):
        time.sleep(2)
        seq += 1
        reply = ask(args.host, args.port, "status", seq, args.token)
        if reply is None:
            print("Verbindung verloren", file=sys.stderr)
            return 1
        show(reply)
    return 0 if reply.get("state") != "error" else 2


if __name__ == "__main__":
    sys.exit(main())
