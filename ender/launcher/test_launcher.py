"""Tests of the Ender launcher with harmless shell commands (no docker, no ROS).

python -m unittest discover -s ender/launcher -v
"""

from __future__ import annotations

import json
import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path

from ender_launcher import Launcher, LauncherServer, Profile, Step, load_profile
from launcher_client import ask


def wait_for_state(launcher: Launcher, states: tuple[str, ...], timeout: float = 10.0) -> dict:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        snapshot = launcher.snapshot()
        if snapshot["state"] in states:
            return snapshot
        time.sleep(0.02)
    raise AssertionError(f"state {states} not reached, last: {launcher.snapshot()}")


class LauncherTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)
        self.addCleanup(self._tmp.cleanup)

    def make(self, steps, stop_steps=(), guard=None) -> Launcher:
        profile = Profile("test", tuple(steps), tuple(stop_steps), guard)
        return Launcher(profile, self.dir, poll_interval=0.05)

    def test_steps_run_in_order(self) -> None:
        launcher = self.make([Step("a", "echo a >> log"), Step("b", "echo b >> log")])
        launcher.start("10.0.0.1")
        snapshot = wait_for_state(launcher, ("running", "error"))
        self.assertEqual(snapshot["state"], "running")
        self.assertEqual([s["state"] for s in snapshot["steps"]], ["ok", "ok"])
        self.assertEqual((self.dir / "log").read_text().split(), ["a", "b"])

    def test_failing_step_stops_the_sequence(self) -> None:
        launcher = self.make(
            [
                Step("a", "true"),
                Step("kaputt", "echo kein Flight Controller; exit 3"),
                Step("c", "echo c > c"),
            ]
        )
        launcher.start("10.0.0.1")
        snapshot = wait_for_state(launcher, ("error",))
        self.assertEqual([s["state"] for s in snapshot["steps"]], ["ok", "failed", "pending"])
        self.assertIn("kein Flight Controller", snapshot["message"])
        self.assertFalse((self.dir / "c").exists())

    def test_start_twice_runs_the_steps_once(self) -> None:
        launcher = self.make([Step("a", "sleep 0.3; echo x >> log")])
        launcher.start("10.0.0.1")
        launcher.start("10.0.0.1")
        wait_for_state(launcher, ("running",))
        launcher.start("10.0.0.1")
        time.sleep(0.2)
        self.assertEqual((self.dir / "log").read_text().split(), ["x"])

    def test_client_ip_is_filled_in(self) -> None:
        launcher = self.make([Step("a", "echo {client_ip} > ip")])
        launcher.start("192.168.178.42")
        wait_for_state(launcher, ("running",))
        self.assertEqual((self.dir / "ip").read_text().strip(), "192.168.178.42")

    def test_wait_for_condition(self) -> None:
        launcher = self.make(
            [Step("a", "(sleep 0.3; touch ready) &", wait_for="test -f ready", wait_timeout=5)]
        )
        launcher.start("10.0.0.1")
        self.assertEqual(wait_for_state(launcher, ("running", "error"))["state"], "running")

    def test_wait_for_times_out(self) -> None:
        launcher = self.make([Step("a", "true", wait_for="false", wait_timeout=0.3)])
        launcher.start("10.0.0.1")
        snapshot = wait_for_state(launcher, ("error",))
        self.assertIn("Zeitüberschreitung", snapshot["message"])

    def test_step_timeout(self) -> None:
        launcher = self.make([Step("a", "sleep 5", timeout=0.2)])
        launcher.start("10.0.0.1")
        self.assertIn("Zeitüberschreitung", wait_for_state(launcher, ("error",))["message"])

    def test_background_process_does_not_block_the_step(self) -> None:
        launcher = self.make([Step("a", "sleep 3 &")])
        started = time.monotonic()
        launcher.start("10.0.0.1")
        wait_for_state(launcher, ("running",))
        self.assertLess(time.monotonic() - started, 2.0)

    def test_optional_step_may_fail(self) -> None:
        launcher = self.make([Step("a", "exit 1", optional=True), Step("b", "true")])
        launcher.start("10.0.0.1")
        self.assertEqual(wait_for_state(launcher, ("running", "error"))["state"], "running")

    def test_stop_runs_stop_steps(self) -> None:
        launcher = self.make([Step("a", "true")], [Step("s", "echo stopped > s")])
        launcher.start("10.0.0.1")
        wait_for_state(launcher, ("running",))
        launcher.stop("10.0.0.1")
        self.assertEqual(wait_for_state(launcher, ("stopped",))["state"], "stopped")
        self.assertTrue((self.dir / "s").exists())

    def test_stop_while_starting_cancels_the_start(self) -> None:
        launcher = self.make(
            [Step("a", "true", wait_for="false", wait_timeout=30), Step("b", "echo b > b")],
            [Step("s", "true")],
        )
        launcher.start("10.0.0.1")
        time.sleep(0.2)
        launcher.stop("10.0.0.1")
        self.assertEqual(wait_for_state(launcher, ("stopped", "error"))["state"], "stopped")
        self.assertFalse((self.dir / "b").exists())

    def test_guard_can_refuse_stop(self) -> None:
        launcher = self.make(
            [Step("a", "true")], [Step("s", "true")], guard="echo Drohne ist scharf; exit 1"
        )
        launcher.start("10.0.0.1")
        wait_for_state(launcher, ("running",))
        launcher.stop("10.0.0.1")
        snapshot = launcher.snapshot()
        self.assertEqual(snapshot["state"], "running")
        self.assertIn("Drohne ist scharf", snapshot["message"])

    def test_stop_when_stopped_does_nothing(self) -> None:
        launcher = self.make([Step("a", "true")], [Step("s", "echo x > s")])
        launcher.stop("10.0.0.1")
        time.sleep(0.1)
        self.assertEqual(launcher.snapshot()["state"], "stopped")
        self.assertFalse((self.dir / "s").exists())

    def test_start_again_after_error(self) -> None:
        launcher = self.make([Step("a", "test -f go")])
        launcher.start("10.0.0.1")
        wait_for_state(launcher, ("error",))
        (self.dir / "go").write_text("")
        launcher.start("10.0.0.1")
        self.assertEqual(wait_for_state(launcher, ("running", "error"))["state"], "running")

    def test_steps_are_listed_before_anything_runs(self) -> None:
        launcher = self.make([Step("a", "true"), Step("b", "true", stop="true")])
        snapshot = launcher.snapshot()
        self.assertEqual(snapshot["state"], "stopped")
        self.assertEqual([s["name"] for s in snapshot["steps"]], ["a", "b"])
        self.assertEqual([s["state"] for s in snapshot["steps"]], ["pending", "pending"])
        self.assertEqual([s["can_stop"] for s in snapshot["steps"]], [False, True])

    def test_single_step_runs_alone(self) -> None:
        launcher = self.make([Step("a", "echo a >> log"), Step("b", "echo b >> log")])
        launcher.run_step(1, "10.0.0.1")
        snapshot = wait_for_state(launcher, ("partial", "error"))
        self.assertEqual(snapshot["state"], "partial")
        self.assertEqual([s["state"] for s in snapshot["steps"]], ["pending", "ok"])
        self.assertEqual((self.dir / "log").read_text().split(), ["b"])

    def test_steps_one_after_the_other_end_in_running(self) -> None:
        launcher = self.make([Step("a", "true"), Step("b", "true")])
        launcher.run_step(0, "10.0.0.1")
        wait_for_state(launcher, ("partial",))
        launcher.run_step(1, "10.0.0.1")
        self.assertEqual(wait_for_state(launcher, ("running", "error"))["state"], "running")

    def test_start_skips_steps_that_are_done(self) -> None:
        launcher = self.make([Step("a", "echo a >> log"), Step("b", "echo b >> log")])
        launcher.run_step(0, "10.0.0.1")
        wait_for_state(launcher, ("partial",))
        launcher.start("10.0.0.1")
        self.assertEqual(wait_for_state(launcher, ("running", "error"))["state"], "running")
        self.assertEqual((self.dir / "log").read_text().split(), ["a", "b"])

    def test_single_step_can_be_run_again(self) -> None:
        launcher = self.make([Step("a", "echo x >> log")])
        launcher.run_step(0, "10.0.0.1")
        wait_for_state(launcher, ("running",))
        launcher.run_step(0, "10.0.0.1")
        time.sleep(0.3)
        wait_for_state(launcher, ("running",))
        self.assertEqual((self.dir / "log").read_text().split(), ["x", "x"])

    def test_failed_step_can_be_retried_alone(self) -> None:
        launcher = self.make([Step("a", "true"), Step("b", "test -f go")])
        launcher.start("10.0.0.1")
        snapshot = wait_for_state(launcher, ("error",))
        self.assertEqual([s["state"] for s in snapshot["steps"]], ["ok", "failed"])
        (self.dir / "go").write_text("")
        launcher.run_step(1, "10.0.0.1")
        self.assertEqual(wait_for_state(launcher, ("running", "error"))["state"], "running")

    def test_single_step_is_refused_while_busy(self) -> None:
        launcher = self.make([Step("a", "sleep 0.5"), Step("b", "echo b > b")])
        launcher.run_step(0, "10.0.0.1")
        launcher.run_step(1, "10.0.0.1")
        self.assertIn("Beschäftigt", launcher.snapshot()["message"])
        wait_for_state(launcher, ("partial",))
        self.assertFalse((self.dir / "b").exists())

    def test_stop_step_stops_only_that_step(self) -> None:
        launcher = self.make(
            [Step("a", "true"), Step("b", "true", stop="echo gestoppt > stopped")],
            [Step("s", "true")],
        )
        launcher.start("10.0.0.1")
        wait_for_state(launcher, ("running",))
        launcher.stop_step(1, "10.0.0.1")
        snapshot = wait_for_state(launcher, ("partial",))
        self.assertEqual([s["state"] for s in snapshot["steps"]], ["ok", "pending"])
        self.assertTrue((self.dir / "stopped").exists())

    def test_stop_step_without_a_stop_command(self) -> None:
        launcher = self.make([Step("a", "true")])
        launcher.start("10.0.0.1")
        wait_for_state(launcher, ("running",))
        launcher.stop_step(0, "10.0.0.1")
        snapshot = launcher.snapshot()
        self.assertEqual(snapshot["state"], "running")
        self.assertIn("keinen Einzel-Stopp", snapshot["message"])

    def test_stop_step_failure_is_reported(self) -> None:
        launcher = self.make([Step("a", "true", stop="echo geht nicht; exit 1")])
        launcher.start("10.0.0.1")
        wait_for_state(launcher, ("running",))
        launcher.stop_step(0, "10.0.0.1")
        time.sleep(0.3)
        snapshot = launcher.snapshot()
        self.assertEqual(snapshot["state"], "running")
        self.assertIn("geht nicht", snapshot["message"])

    def test_guard_also_covers_stop_step(self) -> None:
        launcher = self.make([Step("a", "true", stop="echo x > x")], guard="exit 1")
        launcher.start("10.0.0.1")
        wait_for_state(launcher, ("running",))
        launcher.stop_step(0, "10.0.0.1")
        self.assertIn("Stoppen gesperrt", launcher.snapshot()["message"])
        self.assertFalse((self.dir / "x").exists())

    def test_stop_after_partial_start_resets_everything(self) -> None:
        launcher = self.make([Step("a", "true"), Step("b", "true")], [Step("s", "true")])
        launcher.run_step(0, "10.0.0.1")
        wait_for_state(launcher, ("partial",))
        launcher.stop("10.0.0.1")
        snapshot = wait_for_state(launcher, ("stopped",))
        self.assertEqual([s["state"] for s in snapshot["steps"]], ["pending", "pending"])


class ServerTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        profile = Profile("test", (Step("a", "true"),), (Step("s", "true"),))
        self.launcher = Launcher(profile, Path(self._tmp.name), poll_interval=0.05)

    def serve(self, token: str = "") -> LauncherServer:
        server = LauncherServer(self.launcher, "127.0.0.1", 0, token)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.close)
        self.addCleanup(thread.join, 2)
        self.addCleanup(server.shutdown)
        return server

    def test_start_and_status_over_udp(self) -> None:
        server = self.serve()
        reply = ask("127.0.0.1", server.port, "start", 11)
        self.assertIsNotNone(reply)
        self.assertEqual(reply["seq"], 11)
        self.assertIn(reply["state"], ("starting", "running"))
        wait_for_state(self.launcher, ("running",))
        reply = ask("127.0.0.1", server.port, "status", 12)
        self.assertEqual(reply["state"], "running")
        self.assertEqual(reply["steps"][0]["name"], "a")

    def test_unknown_command_is_not_executed(self) -> None:
        server = self.serve()
        reply = ask("127.0.0.1", server.port, "rm -rf /", 5)
        self.assertEqual(reply["error"], "unknown command")
        self.assertEqual(self.launcher.snapshot()["state"], "stopped")

    def test_garbage_is_ignored(self) -> None:
        server = self.serve()
        self.assertIsNone(server.handle(b"\xff\xfe", "127.0.0.1"))
        self.assertIsNone(server.handle(b"[1,2]", "127.0.0.1"))
        self.assertIsNone(server.handle(b'{"v":2,"seq":1,"cmd":"start"}', "127.0.0.1"))
        self.assertIsNone(server.handle(b'{"v":1,"seq":"x","cmd":"start"}', "127.0.0.1"))
        self.assertEqual(self.launcher.snapshot()["state"], "stopped")

    def test_token(self) -> None:
        server = self.serve(token="geheim")
        self.assertIsNone(ask("127.0.0.1", server.port, "start", 1, attempts=1))
        self.assertIsNone(ask("127.0.0.1", server.port, "start", 2, token="falsch", attempts=1))
        self.assertEqual(self.launcher.snapshot()["state"], "stopped")
        self.assertIsNotNone(ask("127.0.0.1", server.port, "status", 3, token="geheim"))

    def test_single_step_over_udp(self) -> None:
        profile = Profile("test", (Step("a", "true"), Step("b", "true", stop="true")), ())
        self.launcher = Launcher(profile, Path(self._tmp.name), poll_interval=0.05)
        server = self.serve()
        reply = ask("127.0.0.1", server.port, "step", 20, step=1)
        self.assertEqual(reply["seq"], 20)
        wait_for_state(self.launcher, ("partial",))
        reply = ask("127.0.0.1", server.port, "status", 21)
        self.assertEqual([s["state"] for s in reply["steps"]], ["pending", "ok"])
        reply = ask("127.0.0.1", server.port, "stop_step", 22, step=1)
        wait_for_state(self.launcher, ("stopped",))

    def test_unknown_step_is_refused(self) -> None:
        server = self.serve()
        for bad in (-1, 5, "a", True, None):
            request = {"v": 1, "seq": 1, "cmd": "step", "step": bad}
            reply = json.loads(server.handle(json.dumps(request).encode(), "127.0.0.1"))
            self.assertEqual(reply["error"], "unknown step", bad)
        self.assertEqual(self.launcher.snapshot()["state"], "stopped")

    def test_reply_fits_into_one_datagram(self) -> None:
        steps = tuple(Step(f"Schritt {i}", "echo " + "x" * 90 + "; exit 1") for i in range(12))
        launcher = Launcher(Profile("test", steps, ()), Path(self._tmp.name), poll_interval=0.05)
        server = LauncherServer(launcher, "127.0.0.1", 0)
        self.addCleanup(server.close)
        reply = server.handle(json.dumps({"v": 1, "seq": 1, "cmd": "start"}).encode(), "127.0.0.1")
        wait_for_state(launcher, ("error",))
        reply = server.handle(json.dumps({"v": 1, "seq": 2, "cmd": "status"}).encode(), "127.0.0.1")
        self.assertLessEqual(len(reply), 1200)
        json.loads(reply)

    def test_server_socket_is_udp(self) -> None:
        server = LauncherServer(self.launcher, "127.0.0.1", 0)
        self.addCleanup(server.close)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("127.0.0.1", 0))
            self.assertNotEqual(probe.getsockname()[1], server.port)


class ProfileTest(unittest.TestCase):
    def test_sim_profile_loads(self) -> None:
        profile = load_profile(Path(__file__).parent / "profiles" / "sim.json")
        self.assertEqual(profile.name, "sim")
        self.assertTrue(profile.steps)
        self.assertTrue(any("{client_ip}" in s.run for s in profile.steps))

    def test_step_names_must_be_unique(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "p.json"
            path.write_text(
                json.dumps({"name": "p", "steps": [{"name": "a", "run": "true"}] * 2}),
                encoding="utf-8",
            )
            with self.assertRaises(ValueError):
                load_profile(path)

    def test_sim_profile_can_stop_the_planner_and_the_nodes_alone(self) -> None:
        profile = load_profile(Path(__file__).parent / "profiles" / "sim.json")
        stoppable = [s.name for s in profile.steps if s.stop]
        self.assertEqual(stoppable, ["Planer (SUPER)", "Ender-Nodes"])


if __name__ == "__main__":
    unittest.main()
