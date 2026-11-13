import socket
import struct
import unittest
import zlib
from types import SimpleNamespace

import ros_udp_bridge as bridge


class ProtocolTest(unittest.TestCase):
    def test_binary_layout_fragmentation_and_real_udp_loopback(self):
        payload = bytes(i % 127 for i in range(128000))
        packets = list(bridge.datagrams(2, 123, 1, payload, 123456))
        self.assertTrue(all(len(p) <= 1200 for p in packets))
        result = bytearray()
        for index, packet in enumerate(packets):
            header = bridge.HEADER.unpack_from(packet)
            self.assertEqual(header[:4], (b"EGUD", 1, 2, 0))
            self.assertEqual(header[6:8], (index, len(packets)))
            self.assertEqual(header[9], zlib.crc32(payload))
            result.extend(packet[36:])
        self.assertEqual(payload, result)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(1)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.sendto(packets[0], receiver.getsockname())
            self.assertEqual(packets[0], receiver.recv(1201))

    def test_padded_big_endian_cloud_and_nan_filter(self):
        fields = [SimpleNamespace(name=n, offset=i * 4, datatype=7, count=1) for i, n in enumerate(("x", "y", "z", "intensity"))]
        data = struct.pack(">4f", 1, 2, 3, 128) + b"padding!" + struct.pack(">4f", float("nan"), 5, 6, 255) + b"padding!"
        message = SimpleNamespace(width=1, height=2, point_step=16, row_step=24, fields=fields, data=data, is_bigendian=True)
        result = bridge.cloud_payload(message, 8000)
        self.assertEqual(len(result), 16)
        self.assertEqual(struct.unpack_from("<3f", result), (1, 2, 3))
        self.assertAlmostEqual(struct.unpack_from("<f", result, 12)[0], 128 / 255, places=6)

    def test_integer_reflectivity_and_sampling_bound(self):
        fields = [SimpleNamespace(name=n, offset=i * 4, datatype=7, count=1) for i, n in enumerate(("x", "y", "z"))]
        fields.append(SimpleNamespace(name="reflectivity", offset=12, datatype=2, count=1))
        data = b"".join(struct.pack("<3fB3x", i, 2, 3, 128) for i in range(100))
        message = SimpleNamespace(width=100, height=1, point_step=16, row_step=1600, fields=fields, data=data, is_bigendian=False)
        result = bridge.cloud_payload(message, 10)
        self.assertEqual(len(result), 160)
        self.assertEqual(struct.unpack_from("<f", result)[0], 0)
        self.assertEqual(struct.unpack_from("<f", result, 144)[0], 99)

    def test_malformed_cloud_and_invalid_frames(self):
        with self.assertRaises(ValueError):
            list(bridge.datagrams(1, 1, 1, b"bad", 1))
        with self.assertRaises(ValueError):
            list(bridge.datagrams(2, 1, 1, bytes(128016), 1))
        m = SimpleNamespace(width=1, height=1, point_step=16, row_step=16, data=b"short")
        with self.assertRaises(ValueError):
            bridge.cloud_payload(m, 8000)


if __name__ == "__main__":
    unittest.main()
