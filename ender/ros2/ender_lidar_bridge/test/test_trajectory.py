import math
import unittest

from ender_lidar_bridge.map_protocol import (
    KIND_BACKUP, KIND_EXP, TYPE_CONTROL, TYPE_POSE, TYPE_TRAJECTORY, OutQueue, StreamDecoder, decode_control,
    decode_control_wishes, decode_trajectory, encode_control, encode_pose, encode_trajectory,
)
from ender_lidar_bridge.trajectory import (
    ACTION_ADD, ACTION_DELETE, ACTION_DELETEALL, TYPE_ARROW, TYPE_LINE_LIST, TYPE_LINE_STRIP, MarkerTrajectory,
    limit_points, rotate, transform_points,
)


def arrow(state, marker_id, start, end, frame="world", ns="exp_traj"):
    state.apply(ACTION_ADD, ns, marker_id, TYPE_ARROW, frame, [start, end])


class MarkerTrajectoryTest(unittest.TestCase):
    def test_arrows_that_touch_become_one_line_in_id_order(self):
        state = MarkerTrajectory()
        # in falscher Reihenfolge angekommen, die ids ordnen sie
        arrow(state, 12, (2, 0, 0), (3, 0, 0))
        arrow(state, 10, (0, 0, 0), (1, 0, 0))
        arrow(state, 11, (1, 0, 0), (2, 0, 0))
        self.assertEqual(state.polylines(), [("world", [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)])])

    def test_a_gap_starts_a_new_line(self):
        state = MarkerTrajectory()
        arrow(state, 1, (0, 0, 0), (1, 0, 0))
        arrow(state, 2, (5, 0, 0), (6, 0, 0))
        lines = state.polylines()
        self.assertEqual([points for _, points in lines], [[(0, 0, 0), (1, 0, 0)], [(5, 0, 0), (6, 0, 0)]])

    def test_deleteall_and_delete_work_like_in_rviz(self):
        state = MarkerTrajectory()
        arrow(state, 1, (0, 0, 0), (1, 0, 0))
        arrow(state, 2, (1, 0, 0), (2, 0, 0))
        state.apply(ACTION_DELETE, "exp_traj", 2, TYPE_ARROW, "", [])
        self.assertEqual(state.polylines(), [("world", [(0, 0, 0), (1, 0, 0)])])
        state.apply(ACTION_DELETEALL, "", 0, TYPE_ARROW, "", [])
        self.assertEqual(state.polylines(), [])
        # ein ADD mit derselben id ersetzt den Marker
        arrow(state, 1, (0, 0, 0), (1, 0, 0))
        arrow(state, 1, (0, 0, 0), (0, 2, 0))
        self.assertEqual(state.polylines(), [("world", [(0, 0, 0), (0, 2, 0)])])

    def test_dirty_flag(self):
        state = MarkerTrajectory()
        self.assertFalse(state.dirty)
        arrow(state, 1, (0, 0, 0), (1, 0, 0))
        self.assertTrue(state.dirty)
        state.polylines()
        self.assertFalse(state.dirty)
        state.apply(ACTION_DELETE, "exp_traj", 99, TYPE_ARROW, "", [])  # gibt es nicht: nichts ändert sich
        self.assertFalse(state.dirty)
        state.apply(ACTION_DELETEALL, "", 0, TYPE_ARROW, "", [])
        self.assertTrue(state.dirty)

    def test_line_strip_and_line_list(self):
        state = MarkerTrajectory()
        state.apply(ACTION_ADD, "a", 1, TYPE_LINE_STRIP, "world", [(0, 0, 0), (1, 0, 0), (1, 1, 0)])
        state.apply(ACTION_ADD, "b", 1, TYPE_LINE_LIST, "world", [(5, 0, 0), (6, 0, 0), (6, 0, 0), (7, 0, 0)])
        lines = [points for _, points in state.polylines()]
        self.assertEqual(lines, [[(0, 0, 0), (1, 0, 0), (1, 1, 0)], [(5, 0, 0), (6, 0, 0), (7, 0, 0)]])

    def test_other_marker_types_and_bad_points_are_ignored(self):
        state = MarkerTrajectory()
        state.apply(ACTION_ADD, "x", 1, 2, "world", [(0, 0, 0), (1, 0, 0)])           # SPHERE
        state.apply(ACTION_ADD, "x", 2, TYPE_ARROW, "world", [(0, 0, 0)])            # Pfeil aus Pose, ein Punkt
        state.apply(ACTION_ADD, "x", 3, TYPE_ARROW, "world", [(0, 0, 0), (math.nan, 0, 0)])
        self.assertEqual(state.polylines(), [])

    def test_different_frames_are_not_joined(self):
        state = MarkerTrajectory()
        arrow(state, 1, (0, 0, 0), (1, 0, 0), frame="world")
        arrow(state, 2, (1, 0, 0), (2, 0, 0), frame="map")
        self.assertEqual([f for f, _ in state.polylines()], ["world", "map"])


class TransformTest(unittest.TestCase):
    def test_translation_only_like_the_sim(self):
        # world -> camera_init in der Sim: camera_init liegt 0,195 m über world, also fallen die z-Werte um 0,195
        out = transform_points([(1.0, 2.0, 2.25)], (0.0, 0.0, -0.195), (0.0, 0.0, 0.0, 1.0))
        self.assertAlmostEqual(out[0][2], 2.055)
        self.assertEqual(out[0][:2], (1.0, 2.0))

    def test_rotation_about_z_by_90_degrees(self):
        s = math.sqrt(0.5)
        out = rotate((0.0, 0.0, s, s), (1.0, 0.0, 0.0))
        self.assertAlmostEqual(out[0], 0.0)
        self.assertAlmostEqual(out[1], 1.0)
        self.assertAlmostEqual(out[2], 0.0)

    def test_rotation_matches_the_matrix_form_for_arbitrary_quaternions(self):
        import random
        rng = random.Random(7)
        for _ in range(50):
            q = [rng.uniform(-1, 1) for _ in range(4)]
            n = math.sqrt(sum(c * c for c in q))
            x, y, z, w = (c / n for c in q)
            v = tuple(rng.uniform(-5, 5) for _ in range(3))
            matrix = (
                (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
            )
            expected = tuple(sum(matrix[r][c] * v[c] for c in range(3)) for r in range(3))
            got = rotate((x, y, z, w), v)
            for a, b in zip(got, expected):
                self.assertAlmostEqual(a, b, places=9)

    def test_limit_points_keeps_the_ends(self):
        line = [(float(i), 0.0, 0.0) for i in range(1001)]
        out = limit_points([line], 100)
        self.assertLessEqual(len(out[0]), 102)
        self.assertEqual(out[0][0], line[0])
        self.assertEqual(out[0][-1], line[-1])
        self.assertIs(limit_points([line], 5000)[0], line)


class TrajectoryProtocolTest(unittest.TestCase):
    def test_roundtrip(self):
        lines = [[(0.0, 0.0, 0.0), (1.5, -2.0, 0.25)], [(3.0, 3.0, 3.0), (4.0, 4.0, 4.0), (5.0, 5.0, 5.0)]]
        decoded = StreamDecoder().feed(encode_trajectory(KIND_BACKUP, lines))
        self.assertEqual(decoded[0][0], TYPE_TRAJECTORY)
        kind, got = decode_trajectory(decoded[0][1])
        self.assertEqual(kind, KIND_BACKUP)
        self.assertEqual(got, lines)

    def test_empty_trajectory_is_valid_and_short_lines_are_dropped(self):
        payload = StreamDecoder().feed(encode_trajectory(KIND_EXP, [[(1.0, 1.0, 1.0)]]))[0][1]
        self.assertEqual(decode_trajectory(payload), (KIND_EXP, []))

    def test_truncated_or_trailing_bytes_are_rejected(self):
        payload = StreamDecoder().feed(encode_trajectory(KIND_EXP, [[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]]))[0][1]
        self.assertIsNone(decode_trajectory(payload[:-1]))
        self.assertIsNone(decode_trajectory(payload + b"\x00"))
        self.assertIsNone(decode_trajectory(b""))

    def test_control_carries_both_wishes(self):
        for want_map in (True, False):
            for want_traj in (True, False):
                payload = StreamDecoder().feed(encode_control(want_map, want_traj))[0][1]
                self.assertEqual(decode_control_wishes(payload), (want_map, want_traj))
                self.assertEqual(decode_control(payload), want_map)
        self.assertIsNone(decode_control_wishes(b""))
        self.assertEqual(StreamDecoder().feed(encode_control(True))[0][0], TYPE_CONTROL)

    def test_queue_sends_pose_then_trajectories_then_map_and_keeps_only_the_newest(self):
        q = OutQueue()
        q.put_map(b"EMAP\x01\x01\x00\x00\x01\x00\x00\x00m")
        q.put_trajectory(KIND_EXP, encode_trajectory(KIND_EXP, [[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]]))
        q.put_trajectory(KIND_EXP, encode_trajectory(KIND_EXP, [[(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)]]))
        q.put_trajectory(KIND_BACKUP, encode_trajectory(KIND_BACKUP, [[(0.0, 0.0, 0.0), (3.0, 0.0, 0.0)]]))
        q.put_pose(encode_pose(1, (0, 0, 0), (0, 0, 0, 1)))
        out = b""
        while len(q.peek()):
            pending = bytes(q.peek())
            out += pending
            q.consume(len(pending))
        got = StreamDecoder().feed(out)
        self.assertEqual([t for t, _ in got], [TYPE_POSE, TYPE_TRAJECTORY, TYPE_TRAJECTORY, 1])
        self.assertEqual(decode_trajectory(got[1][1])[1][0][1], (2.0, 0.0, 0.0))
        self.assertEqual(decode_trajectory(got[2][1])[0], KIND_BACKUP)


if __name__ == "__main__":
    unittest.main()
