import unittest

import numpy as np

from ender_lidar_bridge.map_protocol import (
    HEADER, MAGIC, MAX_PAYLOAD, OutQueue, StreamDecoder, TYPE_MAP, TYPE_POSE,
    decode_pose, encode_map, encode_message, encode_pose,
)

try:
    from ender_lidar_bridge.compressed_cloud import HEADER as CLOUD_HEADER, pack_cloud, unpack_cloud
    import zstandard  # noqa: F401
    HAVE_ZSTD = True
except ImportError:
    HAVE_ZSTD = False


class FramingTest(unittest.TestCase):
    def test_pose_roundtrip(self):
        message = encode_pose(4_000_000_123, (1.5, -2.25, 0.75), (0.0, 0.0, 0.7071068, 0.7071068))
        decoded = StreamDecoder().feed(message)
        self.assertEqual(len(decoded), 1)
        self.assertEqual(decoded[0][0], TYPE_POSE)
        pose = decode_pose(decoded[0][1])
        self.assertEqual(pose.stamp_ms, 4_000_000_123)
        self.assertEqual(pose.position, (1.5, -2.25, 0.75))
        self.assertAlmostEqual(pose.orientation[2], 0.7071068, places=6)

    def test_pose_with_wrong_length_is_rejected(self):
        self.assertIsNone(decode_pose(b"\x00" * 10))

    def test_header_is_12_bytes_and_starts_with_magic(self):
        message = encode_map(b"abc")
        self.assertEqual(HEADER.size, 12)
        self.assertEqual(message[:4], MAGIC)
        self.assertEqual(len(message), 12 + 3)

    def test_messages_split_at_any_byte(self):
        stream = encode_pose(1, (0, 0, 0), (0, 0, 0, 1)) + encode_map(b"x" * 1000) + encode_pose(2, (1, 0, 0), (0, 0, 0, 1))
        decoder = StreamDecoder()
        got = []
        for i in range(0, len(stream), 7):  # in kleinen Stücken, wie es TCP liefern darf
            got += decoder.feed(stream[i:i + 7])
        self.assertEqual([t for t, _ in got], [TYPE_POSE, TYPE_MAP, TYPE_POSE])
        self.assertEqual(got[1][1], b"x" * 1000)

    def test_recovers_after_garbage(self):
        decoder = StreamDecoder()
        got = decoder.feed(b"\x01\x02garbage" + encode_map(b"ok"))
        self.assertEqual(got, [(TYPE_MAP, b"ok")])
        self.assertGreater(decoder.bad_bytes, 0)

    def test_absurd_length_is_not_waited_for(self):
        bad = HEADER.pack(MAGIC, 1, TYPE_MAP, 0, MAX_PAYLOAD + 1)
        got = StreamDecoder().feed(bad + encode_map(b"ok"))
        self.assertEqual(got, [(TYPE_MAP, b"ok")])

    def test_unknown_version_is_skipped(self):
        bad = HEADER.pack(MAGIC, 9, TYPE_MAP, 0, 2) + b"zz"
        got = StreamDecoder().feed(bad + encode_map(b"ok"))
        self.assertEqual(got, [(TYPE_MAP, b"ok")])

    def test_payload_over_limit_is_not_encoded(self):
        with self.assertRaises(ValueError):
            encode_message(TYPE_MAP, b"x" * (MAX_PAYLOAD + 1))


class OutQueueTest(unittest.TestCase):
    def drain(self, queue, chunk=1000):
        out = b""
        while True:
            pending = queue.peek()
            if not len(pending):
                return out
            n = min(chunk, len(pending))
            out += bytes(pending[:n])
            queue.consume(n)

    def test_newest_map_replaces_unsent_one(self):
        q = OutQueue()
        q.put_map(encode_map(b"old"))
        q.put_map(encode_map(b"new"))
        self.assertEqual(q.dropped_maps, 1)
        self.assertEqual(StreamDecoder().feed(self.drain(q)), [(TYPE_MAP, b"new")])

    def test_half_sent_map_is_finished_before_the_next_message(self):
        q = OutQueue()
        q.put_map(encode_map(b"A" * 500))
        first = bytes(q.peek()[:100])
        q.consume(100)
        q.put_map(encode_map(b"B" * 500))
        q.put_pose(encode_pose(1, (0, 0, 0), (0, 0, 0, 1)))
        rest = self.drain(q, chunk=64)
        got = StreamDecoder().feed(first + rest)
        self.assertEqual([t for t, _ in got], [TYPE_MAP, TYPE_POSE, TYPE_MAP])
        self.assertEqual(got[0][1], b"A" * 500)
        self.assertEqual(got[2][1], b"B" * 500)

    def test_pose_goes_before_a_waiting_map(self):
        q = OutQueue()
        q.put_map(encode_map(b"m"))
        q.put_pose(encode_pose(1, (0, 0, 0), (0, 0, 0, 1)))
        self.assertEqual([t for t, _ in StreamDecoder().feed(self.drain(q))], [TYPE_POSE, TYPE_MAP])

    def test_idle_queue_is_empty(self):
        self.assertEqual(len(OutQueue().peek()), 0)


@unittest.skipUnless(HAVE_ZSTD, "zstandard nicht installiert")
class CompressedCloudTest(unittest.TestCase):
    def test_roundtrip_in_centimeters(self):
        rng = np.random.default_rng(3)
        pts = rng.uniform(-20, 20, size=(4000, 3)) + np.array([100.0, -50.0, 2.0])
        sec, nsec, frame_id, out = unpack_cloud(pack_cloud(pts, 12, 345, "world"))
        self.assertEqual((sec, nsec, frame_id), (12, 345, "world"))
        self.assertTrue(np.allclose(out, pts, atol=0.006))

    def test_header_size_matches_birks_struct(self):
        self.assertEqual(CLOUD_HEADER.size, 38)  # 4+4+3*8+4+2

    def test_empty_cloud(self):
        self.assertEqual(unpack_cloud(pack_cloud(np.zeros((0, 3)), 0, 0))[3].shape, (0, 3))


if __name__ == "__main__":
    unittest.main()
