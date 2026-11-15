#!/usr/bin/env python3
"""Test client for the Ender launcher: sends start, stop or status and prints the state.

    python launcher_client.py 127.0.0.1 start --watch     # start and follow until it is running or failed
    python launcher_client.py 192.168.178.42 status
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
    host: str, port: int, command: str, seq: int, token: str = "", attempts: int = 5
) -> dict | None:
    """Sends a request and waits for the answer with the same seq. A lost datagram is simply sent again."""
    request = {"v": 1, "seq": seq, "cmd": command}
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
    for step in reply.get("steps", []):
        print(f"  {SYMBOLS.get(step['state'], '?')} {step['name']}  {step['detail']}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n", maxsplit=1)[0])
    parser.add_argument("host")
    parser.add_argument("command", choices=["start", "stop", "status"])
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--token", default="")
    parser.add_argument(
        "--watch",
        action="store_true",
        help="keep asking until the state no longer changes by itself",
    )
    args = parser.parse_args()

    seq = int(time.time()) % 1_000_000
    reply = ask(args.host, args.port, args.command, seq, args.token)
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
