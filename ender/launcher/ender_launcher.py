#!/usr/bin/env python3
"""Ender launcher: starts and stops the software stack of the drone on a command from the ground station.

It runs on the host of the onboard computer (or, for tests, on the Linux machine with the simulation), outside of
the ROS container. The ground station sends one of three commands over UDP (protocol "ELCH" v1, JSON):

    {"v":1,"seq":7,"cmd":"start"}      start everything (idempotent: while it runs, nothing happens twice)
    {"v":1,"seq":8,"cmd":"stop"}       stop everything (refused while the optional guard command fails)
    {"v":1,"seq":9,"cmd":"status"}     only report

Every request is answered with the current state, so a lost datagram is simply asked again:

    {"v":1,"seq":9,"profile":"sim","state":"starting","message":"",
     "steps":[{"name":"Container","state":"ok","detail":""}, ...]}

state is one of stopped, starting, running, stopping, error. The steps and their commands come from a profile
file (profiles/*.json). The network can only pick one of the three commands above, never a command line: what runs
is fixed by the profile on this machine. The placeholder {client_ip} in a command is replaced by the address the
start request came from, which is where the point cloud is sent to.

Uses only the standard library.
"""

from __future__ import annotations

import argparse
import hmac
import ipaddress
import json
import logging
import socket
import subprocess
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path

DEFAULT_PORT = 19903
MAX_DATAGRAM = 1200
MAX_DETAIL = 100

log = logging.getLogger("ender_launcher")


@dataclass(frozen=True)
class Step:
    """One command of a profile, optionally followed by a condition that must become true."""

    name: str
    run: str
    timeout: float = 60.0
    wait_for: str | None = None
    wait_timeout: float = 0.0
    optional: bool = False


@dataclass(frozen=True)
class Profile:
    name: str
    steps: tuple[Step, ...]
    stop_steps: tuple[Step, ...]
    stop_guard: str | None = None


def _step_from_json(raw: dict) -> Step:
    return Step(
        name=str(raw["name"]),
        run=str(raw["run"]),
        timeout=float(raw.get("timeout", 60.0)),
        wait_for=raw.get("wait_for"),
        wait_timeout=float(raw.get("wait_timeout", 0.0)),
        optional=bool(raw.get("optional", False)),
    )


def load_profile(path: Path) -> Profile:
    raw = json.loads(path.read_text(encoding="utf-8"))
    return Profile(
        name=str(raw["name"]),
        steps=tuple(_step_from_json(s) for s in raw["steps"]),
        stop_steps=tuple(_step_from_json(s) for s in raw.get("stop_steps", [])),
        stop_guard=raw.get("stop_guard"),
    )


def _short(text: str) -> str:
    return text if len(text) <= MAX_DETAIL else text[: MAX_DETAIL - 1] + "…"


class Launcher:
    """Runs the steps of a profile in a worker thread and keeps the state for the status reply."""

    def __init__(self, profile: Profile, workdir: Path, poll_interval: float = 2.0) -> None:
        self._profile = profile
        self._workdir = workdir
        self._poll = poll_interval
        self._lock = threading.Lock()
        self._state = "stopped"
        self._message = ""
        self._steps: list[dict] = []
        self._worker: threading.Thread | None = None
        self._cancel = threading.Event()

    # -- what the server asks for ------------------------------------------------------------------------------

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "profile": self._profile.name,
                "state": self._state,
                "message": self._message,
                "steps": [dict(s) for s in self._steps],
            }

    def start(self, client_ip: str) -> None:
        with self._lock:
            if self._state in ("starting", "running", "stopping"):
                return
            self._state = "starting"
            self._message = ""
            self._steps = [
                {"name": s.name, "state": "pending", "detail": ""} for s in self._profile.steps
            ]
            self._cancel.clear()
            self._worker = threading.Thread(
                target=self._run_sequence,
                args=(self._profile.steps, "running", client_ip),
                name="launcher-start",
                daemon=True,
            )
            self._worker.start()

    def stop(self, client_ip: str) -> None:
        with self._lock:
            if self._state in ("stopped", "stopping"):
                return
        if self._profile.stop_guard:
            code, tail = self._run_shell(self._profile.stop_guard, 5.0, client_ip)
            if code != 0:
                with self._lock:
                    self._message = _short(
                        f"Stoppen gesperrt: {tail}" if tail else "Stoppen gesperrt"
                    )
                log.warning("stop refused by the guard: %s", tail)
                return
        with self._lock:
            previous = self._worker
            self._state = "stopping"
            self._message = ""
            self._cancel.set()
            self._worker = threading.Thread(
                target=self._stop_sequence,
                args=(previous, client_ip),
                name="launcher-stop",
                daemon=True,
            )
            self._worker.start()

    # -- worker ------------------------------------------------------------------------------------------------

    def _stop_sequence(self, previous: threading.Thread | None, client_ip: str) -> None:
        if previous is not None:
            previous.join()
        with self._lock:
            self._steps = [
                {"name": s.name, "state": "pending", "detail": ""} for s in self._profile.stop_steps
            ]
        self._cancel.clear()
        self._run_sequence(self._profile.stop_steps, "stopped", client_ip)

    def _run_sequence(self, steps: tuple[Step, ...], final_state: str, client_ip: str) -> None:
        for index, step in enumerate(steps):
            if self._cancel.is_set():
                return
            self._set_step(index, "running", "")
            ok, detail = self._run_step(step, client_ip)
            if ok:
                self._set_step(index, "ok", detail)
                continue
            if step.optional:
                self._set_step(index, "ok", _short(f"übersprungen: {detail}"))
                continue
            if self._cancel.is_set():
                return  # cancelled by a stop request, which takes over the state
            self._set_step(index, "failed", detail)
            with self._lock:
                self._state = "error"
                self._message = _short(f"{step.name}: {detail}")
            log.error("step %s failed: %s", step.name, detail)
            return
        with self._lock:
            self._state = final_state
        log.info("state: %s", final_state)

    def _run_step(self, step: Step, client_ip: str) -> tuple[bool, str]:
        log.info("step %s: %s", step.name, step.run)
        code, tail = self._run_shell(step.run, step.timeout, client_ip)
        if code != 0:
            return False, tail or f"Exit-Code {code}"
        if step.wait_for:
            deadline = time.monotonic() + step.wait_timeout
            while True:
                if self._cancel.is_set():
                    return False, "abgebrochen"
                code, _ = self._run_shell(step.wait_for, max(5.0, self._poll), client_ip)
                if code == 0:
                    break
                if time.monotonic() >= deadline:
                    return False, "Zeitüberschreitung beim Warten"
                time.sleep(self._poll)
        return True, ""

    def _run_shell(self, command: str, timeout: float, client_ip: str) -> tuple[int | None, str]:
        """Runs a profile command with bash. Output goes to a file, so a background process that keeps the
        output open cannot hold us up. Returns the exit code (None if it did not run) and the last output line."""
        command = command.replace("{client_ip}", client_ip)
        with tempfile.TemporaryFile("w+", encoding="utf-8", errors="replace") as out:
            try:
                done = subprocess.run(
                    ["bash", "-c", command],
                    cwd=self._workdir,
                    stdin=subprocess.DEVNULL,
                    stdout=out,
                    stderr=subprocess.STDOUT,
                    timeout=timeout,
                    check=False,
                )
            except subprocess.TimeoutExpired:
                return None, "Zeitüberschreitung"
            except OSError as error:
                return None, _short(str(error))
            out.seek(0)
            lines = [line.strip() for line in out.read().splitlines() if line.strip()]
        return done.returncode, _short(lines[-1]) if lines else ""

    def _set_step(self, index: int, state: str, detail: str) -> None:
        with self._lock:
            if index < len(self._steps):
                self._steps[index]["state"] = state
                self._steps[index]["detail"] = detail


class LauncherServer:
    """UDP front of the launcher: three commands, every request answered with the state."""

    def __init__(
        self, launcher: Launcher, host: str = "0.0.0.0", port: int = DEFAULT_PORT, token: str = ""
    ) -> None:
        self._launcher = launcher
        self._token = token
        self._stopping = threading.Event()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, port))
        self._sock.settimeout(0.5)

    @property
    def port(self) -> int:
        return self._sock.getsockname()[1]

    def serve_forever(self) -> None:
        log.info("listening on UDP %d", self.port)
        while not self._stopping.is_set():
            try:
                data, address = self._sock.recvfrom(2048)
            except TimeoutError:
                continue
            except OSError:
                break
            reply = self.handle(data, address[0])
            if reply is not None:
                self._sock.sendto(reply, address)
        self._sock.close()

    def shutdown(self) -> None:
        self._stopping.set()

    def close(self) -> None:
        self._stopping.set()
        self._sock.close()

    def handle(self, data: bytes, client_ip: str) -> bytes | None:
        """Returns the reply datagram, or None if the request is not for us (garbage, wrong token)."""
        try:
            request = json.loads(data)
        except ValueError:
            return None
        if (
            not isinstance(request, dict)
            or request.get("v") != 1
            or not isinstance(request.get("seq"), int)
        ):
            return None
        if self._token and not hmac.compare_digest(str(request.get("token", "")), self._token):
            log.warning("request from %s with a wrong token", client_ip)
            return None
        try:
            ipaddress.ip_address(client_ip)
        except ValueError:
            return None

        command = request.get("cmd")
        log.info("%s from %s", command, client_ip)
        if command == "start":
            self._launcher.start(client_ip)
        elif command == "stop":
            self._launcher.stop(client_ip)
        elif command != "status":
            return self._encode({"v": 1, "seq": request["seq"], "error": "unknown command"})
        return self._encode({"v": 1, "seq": request["seq"], **self._launcher.snapshot()})

    @staticmethod
    def _encode(reply: dict) -> bytes:
        payload = json.dumps(reply, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
        if len(payload) > MAX_DATAGRAM:
            for step in reply.get("steps", []):
                step["detail"] = step["detail"][:30]
            payload = json.dumps(reply, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
        return payload[:MAX_DATAGRAM]


def main() -> None:
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n", maxsplit=1)[0])
    parser.add_argument(
        "--profile",
        default="sim",
        help="name in profiles/ or path of a profile file (default: sim)",
    )
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--bind", default="0.0.0.0", help="address to listen on (default: all)")
    parser.add_argument(
        "--token", default="", help="shared secret the requests must carry (optional)"
    )
    parser.add_argument(
        "--repo", type=Path, default=here.parents[1], help="working directory of the commands"
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    path = Path(args.profile)
    if not path.suffix:
        path = here / "profiles" / f"{args.profile}.json"
    server = LauncherServer(
        Launcher(load_profile(path), args.repo), args.bind, args.port, args.token
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()


if __name__ == "__main__":
    main()
