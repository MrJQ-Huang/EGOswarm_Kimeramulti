import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('transport_metrics', Path(__file__).resolve().parents[1]/'distributed/transport_metrics.py')
metrics = importlib.util.module_from_spec(spec); spec.loader.exec_module(metrics)


class TransportMetricsTests(unittest.TestCase):
    def test_only_slam_ports_and_kernel_milliseconds(self):
        rows = metrics.parse_ss('''State Recv-Q Send-Q Local Address:Port Peer Address:Port
ESTAB 0 0 192.168.5.14:50000 192.168.5.6:11740
 cubic rtt:15.795/18.111 bytes_sent:100 bytes_received:800 bytes_retrans:2
CLOSE-WAIT 0 0 192.168.5.14:51000 192.168.5.6:40000
 cubic rtt:1/1 bytes_sent:5000 bytes_received:6000
ESTAB 0 0 192.168.5.14:50001 192.168.5.6:22
 cubic rtt:2/1 bytes_sent:900 bytes_received:900
''')
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['channel'], 'map')
        self.assertEqual(rows[0]['bytes_received'], 800)
        self.assertEqual(rows[0]['smoothed_rtt_ms'], 15.795)

    def test_reconnect_does_not_invent_baseline_or_negative_bytes(self):
        def row(local, rx): return dict(local=local, remote='b:11741', channel='telemetry', bytes_received=rx)
        samples = [dict(monotonic=0, connections=[row('a:1',100)]),
                   dict(monotonic=2, connections=[row('a:1',300)]),
                   dict(monotonic=3, connections=[]),
                   dict(monotonic=4, connections=[row('a:2',500)]),
                   dict(monotonic=5, connections=[row('a:2',550)]),
                   dict(monotonic=6, connections=[row('a:2',10)])]
        result = metrics.summarize(samples)['channels']['telemetry']
        self.assertEqual(result['observed_rx_bytes'], 250)
        self.assertEqual(result['counter_resets'], 1)
        self.assertEqual(result['observed_connections'], 2)
        self.assertAlmostEqual(result['observed_rx_mbit_per_s'], 250*8/6/1e6)
