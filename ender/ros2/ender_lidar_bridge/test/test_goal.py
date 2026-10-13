import math
import unittest

from ender_lidar_bridge.goal_logic import (
    Pose, PoseHistory, body_to_world, evaluate_goal, rotate, unwrap_stamp,
)
from ender_lidar_bridge.goal_protocol import (
    ACK, GOAL, STATUS_BAD_VALUE, STATUS_NO_POSE, STATUS_OK, STATUS_STALE, STATUS_TOO_FAR,
    Ack, Goal, decode_ack, decode_goal, encode_ack, encode_goal,
)


def yaw_quat(deg):
    h = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(h), math.cos(h))


class GoalProtocolTest(unittest.TestCase):
    def test_goal_roundtrip(self):
        packet = encode_goal(Goal(5, 123456, 3.5, -2.25))
        self.assertEqual(len(packet), GOAL.size)
        goal = decode_goal(packet)
        self.assertEqual((goal.goal_seq, goal.scan_stamp_ms, goal.x, goal.y), (5, 123456, 3.5, -2.25))

    def test_ack_roundtrip(self):
        packet = encode_ack(Ack(STATUS_STALE, 9, 1.5, -4.0))
        self.assertEqual(len(packet), ACK.size)
        ack = decode_ack(packet)
        self.assertEqual((ack.status, ack.goal_seq, ack.world_x, ack.world_y), (STATUS_STALE, 9, 1.5, -4.0))

    def test_rejects_garbage(self):
        good = encode_goal(Goal(1, 1, 1.0, 1.0))
        self.assertIsNone(decode_goal(b""))
        self.assertIsNone(decode_goal(good[:-1]))
        self.assertIsNone(decode_goal(b"XXXX" + good[4:]))
        self.assertIsNone(decode_goal(good[:4] + b"\x02" + good[5:]))  # fremde Version
        self.assertIsNone(decode_goal(good[:5] + b"\x09" + good[6:]))  # unbekannte Art
        self.assertIsNone(decode_ack(good))  # Ziel ist keine Bestätigung

    def test_sequence_wraps_to_32_bit(self):
        self.assertEqual(decode_goal(encode_goal(Goal(1 << 32, 0, 0.0, 0.0))).goal_seq, 0)


class GoalLogicTest(unittest.TestCase):
    def history(self, *poses):
        h = PoseHistory()
        for p in poses:
            h.add(p)
        return h

    def test_rotate_yaw_90(self):
        x, y, z = rotate(yaw_quat(90), (1.0, 0.0, 0.0))
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 1.0, places=6)
        self.assertAlmostEqual(z, 0.0, places=6)

    def test_body_to_world_with_offset_and_yaw(self):
        pose = Pose(0, (10.0, 5.0, 1.0), yaw_quat(90))
        # 2 m vor der Drohne zeigt bei Kurs 90 Grad nach +y
        wx, wy, _ = body_to_world(pose, 2.0, 0.0)
        self.assertAlmostEqual(wx, 10.0, places=6)
        self.assertAlmostEqual(wy, 7.0, places=6)

    def test_goal_uses_pose_at_scan_time_not_latest(self):
        h = self.history(
            Pose(10_000, (0.0, 0.0, 1.0), yaw_quat(0)),
            Pose(11_000, (5.0, 0.0, 1.0), yaw_quat(0)),  # Drohne ist seitdem 5 m weiter
        )
        d = evaluate_goal(Goal(1, 10_000, 3.0, 1.0), h)
        self.assertEqual(d.status, STATUS_OK)
        self.assertAlmostEqual(d.world_x, 3.0, places=6)
        self.assertAlmostEqual(d.world_y, 1.0, places=6)

    def test_rejects_stale_scan(self):
        h = self.history(Pose(1_000, (0.0, 0.0, 0.0), yaw_quat(0)), Pose(31_000, (0.0, 0.0, 0.0), yaw_quat(0)))
        self.assertEqual(evaluate_goal(Goal(1, 1_000, 1.0, 0.0), h).status, STATUS_STALE)

    def test_rejects_too_far(self):
        h = self.history(Pose(1_000, (0.0, 0.0, 0.0), yaw_quat(0)))
        self.assertEqual(evaluate_goal(Goal(1, 1_000, 30.0, 30.0), h).status, STATUS_TOO_FAR)

    def test_rejects_non_finite(self):
        h = self.history(Pose(1_000, (0.0, 0.0, 0.0), yaw_quat(0)))
        self.assertEqual(evaluate_goal(Goal(1, 1_000, float("nan"), 0.0), h).status, STATUS_BAD_VALUE)
        self.assertEqual(evaluate_goal(Goal(1, 1_000, 0.0, float("inf")), h).status, STATUS_BAD_VALUE)

    def test_no_pose(self):
        self.assertEqual(evaluate_goal(Goal(1, 1_000, 1.0, 0.0), PoseHistory()).status, STATUS_NO_POSE)

    def test_no_pose_close_to_scan_time(self):
        h = self.history(Pose(1_000, (0.0, 0.0, 0.0), yaw_quat(0)), Pose(2_000, (0.0, 0.0, 0.0), yaw_quat(0)))
        # Scan bei 1500 ms liegt 500 ms von beiden Posen entfernt, erlaubt sind 300 ms
        self.assertEqual(evaluate_goal(Goal(1, 1_500, 1.0, 0.0), h).status, STATUS_NO_POSE)

    def test_unwrap_stamp_across_32_bit_overflow(self):
        reference = (1 << 32) + 100  # Odometrie ist schon über den Überlauf, der Scan davor
        self.assertEqual(unwrap_stamp((1 << 32) - 400, reference), reference - 500)
        self.assertEqual(unwrap_stamp(100, reference), reference)

    def test_goal_works_across_overflow(self):
        ref = (1 << 32) + 100
        h = self.history(Pose(ref, (1.0, 2.0, 0.0), yaw_quat(0)))
        d = evaluate_goal(Goal(1, ref & 0xFFFFFFFF, 1.0, 0.0), h)
        self.assertEqual(d.status, STATUS_OK)
        self.assertAlmostEqual(d.world_x, 2.0, places=6)


if __name__ == "__main__":
    unittest.main()
