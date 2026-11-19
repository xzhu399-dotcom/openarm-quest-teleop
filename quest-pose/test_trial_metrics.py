import copy
import math
from pathlib import Path
import unittest

from trial_metrics import summarize_samples, trial_metrics


EXAMPLES = Path(__file__).resolve().parents[1] / "examples"


class TrialMetricTests(unittest.TestCase):
    def test_current_and_legacy_recordings(self):
        current = trial_metrics(EXAMPLES / "mouse-robot")
        legacy = trial_metrics(EXAMPLES / "mouse-right")
        for report in (current, legacy):
            self.assertEqual(report["status"], "saved")
            self.assertEqual(report["recorded_input_frames"], ["mouse-local"])
            self.assertEqual(report["hands"]["right"]["reported_controller_profiles"], ["mouse"])
            self.assertEqual(report["recorded_receive_interval_ms"]["count"], report["recorded_input_samples"] - 1)
            self.assertIsNone(report["hands"]["left"]["clutched_target_vs_fk_position_error_mm"])
        self.assertGreater(current["hands"]["right"]["clutched_target_vs_fk_position_error_mm"]["count"], 0)
        self.assertIsNone(legacy["hands"]["right"]["clutched_target_vs_fk_position_error_mm"])
        self.assertIsNone(legacy["hands"]["right"]["observed_ik_rejection_entries"])

    @staticmethod
    def sample(session, seq, ms, status="clutched"):
        stamp = {"session": session, "seq": seq, "host_monotonic_ns": ms * 1_000_000}
        raw = dict(stamp, sample={"frame": "local-floor", "hands": {
            "left": None, "right": {"profile": "oculus-touch-v3"}}})
        tip = {"p": [0, 0, 0], "q": [0, 0, 0, -1]}
        target = {"p": [.003, .004, 0], "q": [0, 0, math.sin(math.pi/12), math.cos(math.pi/12)]}
        active = {"tracked": True, "clutched": status == "clutched", "status": status, "tool": target}
        action = dict(stamp, state={"hands": {
            "left": {"tracked": False, "clutched": False, "status": "untracked"}, "right": active}})
        output = {"tcp": tip, "robot": {"status": "tracking", "jaw_opening_m": .022}}
        feedback = dict(stamp, hands={"left": output, "right": output})
        return raw, action, feedback

    def test_receive_timing_fk_residuals_and_rejection_periods(self):
        records = [self.sample("a", 1, 0), self.sample("a", 3, 10, "outside simulated reach"),
                   self.sample("a", 4, 30, "outside simulated reach"), self.sample("b", 1, 3500)]
        report = summarize_samples(records)
        self.assertEqual(report["recorded_receive_interval_ms"], {"count": 2, "median": 15., "p95": 19.5, "max": 20.})
        self.assertEqual(report["missing_sequence_numbers_between_records"], 1)
        arm = report["hands"]["right"]
        self.assertEqual(arm["reported_controller_profiles"], ["oculus-touch-v3"])
        self.assertEqual(arm["observed_ik_rejection_entries"], 1)
        self.assertEqual(arm["clutched_target_vs_fk_position_error_mm"], {"count": 2, "median": 5., "p95": 5., "max": 5.})
        self.assertAlmostEqual(arm["clutched_target_vs_fk_orientation_error_deg"]["median"], 30.)
        self.assertEqual(arm["tracked_simulated_jaw_range_mm"], [22., 22.])

    def test_misaligned_structured_rows_are_rejected(self):
        record = self.sample("a", 1, 0)
        wrong = copy.deepcopy(record[2])
        wrong["seq"] = 2
        with self.assertRaisesRegex(ValueError, "do not match"):
            summarize_samples([(record[0], record[1], wrong)])

    def test_j8_encoder_readbacks_are_counted_once_per_bench_timestamp(self):
        records = [self.sample("a", n, n * 20) for n in range(1, 5)]
        for record, timestamp, value in zip(records, [None, 1_000, 1_000, 2_000], [None, 1.2, 1.2, 1.4]):
            record[2]["physical_gripper"] = {"arm": "right", "feedback_host_ns": timestamp,
                                             "measured_deg": value, "mode": "fixture"}
        arm = summarize_samples(records)["hands"]["right"]
        self.assertEqual(arm["recorded_j8_bench"], {"modes": ["fixture"], "distinct_encoder_readbacks": 2,
                                                   "measured_range_deg": [1.2, 1.4]})


if __name__ == "__main__":
    unittest.main()
