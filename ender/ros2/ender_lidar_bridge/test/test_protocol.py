import unittest

import numpy as np

from ender_lidar_bridge.downsample import reduce_points
from ender_lidar_bridge.protocol import FrameAssembler, HEADER, MAX_PAYLOAD, encode_frame


class ProtocolTest(unittest.TestCase):
    def setUp(self):
        self.rng = np.random.default_rng(1)

    def cloud(self, n):
        return self.rng.uniform(-30, 30, size=(n, 3))

    def test_roundtrip_in_centimeters(self):
        pts = self.cloud(5000)
        asm = FrameAssembler()
        frame = None
        for packet in encode_frame(pts, 7, 123456, None):
            frame = asm.feed(packet) or frame
        self.assertEqual(frame.frame_seq, 7)
        self.assertEqual(frame.stamp_ms, 123456)
        self.assertEqual(frame.points_m.shape, (5000, 3))
        self.assertTrue(np.allclose(frame.points_m, pts, atol=0.006))  # halber Zentimeter

    def test_packets_stay_below_limit(self):
        for packet in encode_frame(self.cloud(8000), 0, 0, None):
            self.assertLessEqual(len(packet), MAX_PAYLOAD)

    def test_intensity_roundtrip(self):
        pts = self.cloud(700)
        inten = self.rng.integers(0, 256, 700)
        asm = FrameAssembler()
        frame = None
        for packet in encode_frame(pts, 1, 0, inten):
            frame = asm.feed(packet) or frame
        self.assertTrue(np.array_equal(frame.intensity, inten.astype("u1")))

    def test_empty_frame(self):
        asm = FrameAssembler()
        packets = encode_frame(np.zeros((0, 3)), 1, 0, None)
        self.assertEqual(len(packets), 1)
        frame = asm.feed(packets[0])
        self.assertEqual(frame.points_m.shape, (0, 3))

    def test_out_of_range_points_are_dropped(self):
        pts = np.array([[1, 2, 3], [400, 0, 0], [0, -400, 0]], dtype=float)
        asm = FrameAssembler()
        frame = None
        for packet in encode_frame(pts, 1, 0, None):
            frame = asm.feed(packet) or frame
        self.assertEqual(len(frame.points_m), 1)

    def test_order_of_packets_does_not_matter(self):
        pts = self.cloud(3000)
        packets = encode_frame(pts, 3, 0, None)
        self.rng.shuffle(packets)
        asm = FrameAssembler()
        frame = None
        for packet in packets:
            frame = asm.feed(packet) or frame
        self.assertTrue(np.allclose(np.sort(frame.points_m, axis=0), np.sort(pts, axis=0), atol=0.006))

    def test_lost_packet_drops_whole_frame_when_next_starts(self):
        asm = FrameAssembler()
        first = encode_frame(self.cloud(3000), 1, 0, None)
        second = encode_frame(self.cloud(3000), 2, 0, None)
        for packet in first[:-1]:  # letztes Paket geht verloren
            self.assertIsNone(asm.feed(packet))
        frame = None
        for packet in second:
            frame = asm.feed(packet) or frame
        self.assertEqual(frame.frame_seq, 2)
        self.assertEqual(asm.dropped, 1)
        self.assertEqual(asm.completed, 1)

    def test_garbage_is_counted_and_ignored(self):
        asm = FrameAssembler()
        self.assertIsNone(asm.feed(b"hello"))
        self.assertIsNone(asm.feed(b"\x00" * 40))
        packet = encode_frame(self.cloud(10), 1, 0, None)[0]
        self.assertIsNone(asm.feed(packet[:-1]))  # abgeschnitten
        self.assertEqual(asm.bad_packets, 3)

    def test_sequence_wraparound(self):
        asm = FrameAssembler()
        frame = None
        for seq in (0xFFFFFFFF, 0x100000000, 0x100000001):
            for packet in encode_frame(self.cloud(500), seq, 0, None):
                frame = asm.feed(packet) or frame
        self.assertEqual(asm.completed, 3)

    def test_header_size(self):
        self.assertEqual(HEADER.size, 24)


class DownsampleTest(unittest.TestCase):
    def test_one_point_per_voxel(self):
        pts = np.array([[0.01, 0.01, 0.01], [0.02, 0.03, 0.04], [1.0, 1.0, 1.0]])
        out, _ = reduce_points(pts, voxel_size=0.1, max_points=0, max_range=0)
        self.assertEqual(len(out), 2)

    def test_range_and_nan_filter(self):
        pts = np.array([[1, 0, 0], [100, 0, 0], [np.nan, 0, 0]])
        out, _ = reduce_points(pts, 0, 0, max_range=40)
        self.assertEqual(len(out), 1)

    def test_max_points_with_intensity(self):
        rng = np.random.default_rng(0)
        pts = rng.uniform(-10, 10, size=(5000, 3))
        inten = np.arange(5000)
        out, out_i = reduce_points(pts, 0.0, 1000, 0, inten, rng)
        self.assertEqual(len(out), 1000)
        self.assertEqual(len(out_i), 1000)
        # Intensität gehört weiter zum selben Punkt
        idx = {tuple(p): i for p, i in zip(pts.astype(np.float32), inten)}
        for p, i in zip(out, out_i):
            self.assertEqual(idx[tuple(p)], i)


if __name__ == "__main__":
    unittest.main()
