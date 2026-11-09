from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'distributed'))
from telemetry_metrics import ScalarSummary,TelemetryMetrics

CALLER='/distributed_telemetry_observer'


class TelemetryMetricsTests(unittest.TestCase):
    def test_monotonic_arrival_rate_boundary_silence_and_clock_offset_are_distinct(self):
        metrics=TelemetryMetrics('test',0,10000000000)
        for ms in (100,140,180):
            receive=ms*1000000;source=10000000000+receive-130000000
            metrics.ingest(0,'actual',source,receive,10000000000+receive,CALLER)
        result=metrics.finish(1000000000,11000000000)['streams']['sim_0/actual']
        self.assertAlmostEqual(result['arrival_hz'],25.)
        self.assertAlmostEqual(result['interarrival_gap_ms']['max'],40.)
        self.assertAlmostEqual(result['initial_silence_ms'],100.)
        self.assertAlmostEqual(result['final_silence_ms'],820.)
        self.assertAlmostEqual(result['source_age_including_clock_offset_ms']['mean'],130.)
        self.assertEqual(result['accepted_callbacks_per_window_second'],3.)

    def test_pair_percentages_use_eligible_sampler_ticks_and_strict_thresholds(self):
        metrics=TelemetryMetrics('test',0,1000000000)
        metrics.sample_pair(0)
        for tick,difference in enumerate((40000000,50000000,100000000),1):
            now=tick*20000000
            metrics.ingest(0,'actual',1000000000+now,now,2000000000+now,CALLER)
            metrics.ingest(1,'actual',1000000000+now+difference,now,2000000000+now,CALLER)
            metrics.sample_pair(now)
        metrics.sample_pair(400000000)
        pair=metrics.finish(500000000,2500000000)['actual_pair']
        self.assertEqual(pair['counters']['ticks'],5)
        self.assertEqual(pair['counters']['missing_actual_ticks'],1)
        self.assertEqual(pair['counters']['stale_receipt_ticks'],1)
        self.assertEqual(pair['counters']['eligible_ticks'],3)
        self.assertAlmostEqual(pair['fraction_below_50ms'],1/3)
        self.assertAlmostEqual(pair['fraction_below_100ms'],2/3)
        self.assertEqual(pair['eligible_tick_fraction'],.6)

    def test_negative_source_age_and_source_regression_do_not_change_arrival_clock(self):
        metrics=TelemetryMetrics('test',0,1000000000)
        metrics.ingest(0,'vio',1100000000,10000000,1000000000,CALLER)
        metrics.ingest(0,'vio',1050000000,30000000,900000000,CALLER)
        result=metrics.finish(100000000,1000000000)
        stream=result['streams']['sim_0/vio']
        self.assertEqual(stream['negative_source_age_samples'],2)
        self.assertEqual(stream['source_stamp_regressions'],1)
        self.assertAlmostEqual(stream['arrival_hz'],50.)
        self.assertIn('NOT one-way network latency',result['definitions']['source_age'])

    def test_unexpected_publishers_and_zero_stamps_are_counted_without_polluting_pairs(self):
        metrics=TelemetryMetrics('test',0,1000000000)
        self.assertFalse(metrics.ingest(0,'actual',1000000000,10,1000000010,'/wrong_source'))
        self.assertFalse(metrics.ingest(1,'actual',0,10,1000000010,CALLER))
        metrics.sample_pair(20)
        result=metrics.finish(100,1000000100)
        self.assertEqual(result['streams']['sim_0/actual']['callbacks'],1)
        self.assertEqual(result['streams']['sim_0/actual']['accepted_callbacks'],0)
        self.assertEqual(result['streams']['sim_0/actual']['rejected']['unexpected_caller'],1)
        self.assertEqual(result['actual_pair']['counters']['missing_actual_ticks'],1)
        self.assertFalse(metrics.ingest(0,'actual',1000000000,110,1000000110,CALLER))

    def test_reservoir_is_bounded_but_cumulative_counts_and_mean_are_exact(self):
        summary=ScalarSummary(8)
        for value in range(10000):summary.add(value)
        result=summary.report()
        self.assertEqual(len(summary.values),8)
        self.assertEqual(result['samples'],10000)
        self.assertEqual(result['min'],0.)
        self.assertEqual(result['max'],9999.)
        self.assertEqual(result['mean'],4999.5)
        self.assertEqual(result['percentile_method'],'bounded_uniform_reservoir')

    def test_legacy_combined_caller_requires_explicit_override(self):
        default=TelemetryMetrics('test',0,1000000000)
        self.assertFalse(default.ingest(0,'actual',1000000000,10,1000000010,'/distributed_result_observer'))
        legacy=TelemetryMetrics('test',0,1000000000,expected_caller='/distributed_result_observer')
        self.assertTrue(legacy.ingest(0,'actual',1000000000,10,1000000010,'/distributed_result_observer'))


if __name__=='__main__':unittest.main()
