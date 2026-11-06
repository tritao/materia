"""Check orbit reporting against known timings without a renderer or display."""

import contextlib
import importlib.util
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("benchmark_orbit", Path(__file__).with_name("benchmark-orbit.py"))
benchmark = importlib.util.module_from_spec(spec)
spec.loader.exec_module(benchmark)


class OrbitReportTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.output = Path(self.temporary.name)
        self.host = []
        self.gpu = []
        self.inputs = []
        # Four measured frames, between two warmup and eight query-drain frames.
        # Nonuniform cadence distinguishes actual throughput from median latency.
        starts = [100, 100.05, 100.10, 100.14, 100.18, 100.28]
        starts += [100.33 + i * .05 for i in range(8)]
        for index, start in enumerate(starts):
            measured = 2 <= index < 6
            value = index - 1 if measured else 1000
            self.host.append(dict(frame=1000 + index, startedAtSeconds=start,
                                  frameSeconds=value * .005, applicationSubmitSeconds=.001,
                                  customPaintSeconds=.002, nativeRenderSeconds=.001, frameGcSeconds=0))
            self.gpu.append(dict(frame=index, startedAtSeconds=start + .001,
                                 prepareMilliseconds=.1, submitMilliseconds=.2,
                                 gpuMilliseconds=value * 2, gpuPassMilliseconds={"surfaces": value * 2},
                                 passWork={"surfaces": {"drawCalls": value * 5, "triangles": value * 50}},
                                 drawCalls=value * 5, triangles=value * 50, uploadBytes=0))
            self.inputs.append(dict(startedAtSeconds=start + .003, inputMilliseconds=1,
                                    orbiting=True, yaw=index * math.pi / 180, pitch=.35,
                                    viewSetupMilliseconds=.5, planPrepareMilliseconds=.3))
        self.state = {"scene": {"objects": [{"id": "project:part"}]},
                      "perspective": {"benchmarkInputs": self.inputs}}

    def report(self):
        (self.output / "app-state.json").write_text(json.dumps(self.state))
        for filename, rows in (("frame-timeline.jsonl", self.host),
                               ("scene-timeline.jsonl", list(reversed(self.gpu)))):
            (self.output / filename).write_text("".join(json.dumps(row) + "\n" for row in rows))
        with contextlib.redirect_stdout(io.StringIO()):
            benchmark.summarize(self.output, warmup=2, count=4)

    def test_reports_measured_window_and_matches_delayed_gpu_results(self):
        self.report()
        summary = json.loads((self.output / "orbit-summary.json").read_text())
        frames = benchmark.records(self.output / "orbit-frames.jsonl")
        self.assertEqual([row["frame"] for row in frames], [1002, 1003, 1004, 1005])
        self.assertAlmostEqual(summary["fps"], 1000 / 60)
        self.assertAlmostEqual(summary["cpuMs"]["mean"], 12.5)
        self.assertAlmostEqual(summary["gpuMs"]["mean"], 5)
        self.assertEqual(summary["drawCalls"]["median"], 12.5)
        self.assertEqual(summary["triangles"]["mean"], 125)
        self.assertEqual(summary["inputMs"]["mean"], 1)
        self.assertEqual(summary["viewSetupMs"]["mean"], .5)
        self.assertEqual(summary["planPrepareMs"]["mean"], .3)
        self.assertEqual(summary["gpuPassMs"]["surfaces"]["median"], 5)
        self.assertEqual(summary["passWork"]["surfaces"], {"drawCalls": 12.5, "triangles": 125})

    def test_rejects_missing_gpu_result(self):
        self.gpu[3]["gpuMilliseconds"] = None
        with self.assertRaisesRegex(RuntimeError, "missing GPU"):
            self.report()

    def test_inventory_distinguishes_occurrences_from_resources(self):
        path = self.output / "scene-timeline.jsonl.inventory.tsv"
        path.write_text("node\tgeometry\tmaterial\ttriangles\tstrokes\thidden\tculled\tname\n"
                        "1\t1\t1\t100\t10\t0\t0\tFirst screw\n"
                        "2\t1\t2\t100\t10\t0\t1\tSecond screw\n"
                        "3\t2\t3\t9999\t99\t1\t0\tHidden part\n")
        self.report()
        census = json.loads((self.output / "orbit-summary.json").read_text())["inventory"]
        self.assertEqual(census["nodes"], 3)
        self.assertEqual(census["shownNodes"], 2)
        self.assertEqual(census["shownGeometries"], 1)
        self.assertEqual(census["shownMaterials"], 2)
        self.assertEqual(census["shownGeometryMaterialPairs"], 2)
        self.assertEqual(census["largestMeshes"][0]["triangles"], 100)

    def test_rejects_missing_input(self):
        del self.inputs[3]
        with self.assertRaisesRegex(RuntimeError, "0 orbit inputs"):
            self.report()

    def test_rejects_uncaptured_or_stationary_camera(self):
        self.inputs[3]["orbiting"] = False
        with self.assertRaisesRegex(RuntimeError, "did not capture"):
            self.report()
        self.inputs[3]["orbiting"] = True
        self.inputs[3]["yaw"] = self.inputs[2]["yaw"]
        with self.assertRaisesRegex(RuntimeError, "one degree"):
            self.report()

    def test_rejects_failed_project_load(self):
        self.state["scene"]["objects"] = []
        with self.assertRaisesRegex(RuntimeError, "project did not load"):
            self.report()

    def test_rejects_pitch_interference(self):
        self.inputs[3]["pitch"] = -1.48
        with self.assertRaisesRegex(RuntimeError, "changed the camera pitch"):
            self.report()

    def test_rejects_power_saving_during_measurement(self):
        path = self.output / "display-timeline.jsonl"
        def display(rows):
            path.write_text("".join(json.dumps(dict(atSeconds=at, state=state)) + "\n" for at, state in rows))
        display([(99, "On"), (100.12, "Off"), (100.3, "On")])
        with self.assertRaisesRegex(RuntimeError, "DPMS power saving"):
            self.report()
        # Sleeping before warmup is safe once the display has woken before measurement.
        display([(99, "Off"), (100.05, "On")])
        self.report()


if __name__ == "__main__":
    unittest.main()
