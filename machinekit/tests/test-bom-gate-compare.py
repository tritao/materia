#!/usr/bin/env python3
"""Exercise gate comparison, including incomplete and changed failures."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
tool = root / "machinekit/scripts/bom-gate-compare"
with tempfile.TemporaryDirectory() as directory:
    base = Path(directory) / "base"
    candidate = Path(directory) / "candidate"
    base.mkdir()
    candidate.mkdir()
    baseline = base / "summary"
    result = candidate / "summary"
    baseline.write_text("PASS 1/2 first\nFAIL 2/2 second\nGATE: 1/2 shards passed\n")
    result.write_text("PASS 1/2 first\nFAIL 2/2 second\nGATE: 1/2 shards passed\n")
    (base / "shard-2.log").write_text("Uncaught exception: known failure\n")
    candidate_failure = candidate / "shard-2.log"
    candidate_failure.write_text("Uncaught exception: known failure\n")
    command = [str(tool), str(baseline), str(base), str(result), str(candidate)]
    assert subprocess.run(command, capture_output=True).returncode == 0
    candidate_failure.write_text("Uncaught exception: different failure\n")
    assert subprocess.run(command, capture_output=True).returncode == 1
    candidate_failure.write_text("process terminated without an exception\n")
    assert subprocess.run(command, capture_output=True).returncode == 2
    result.write_text("PASS 1/2 first\nPASS 2/2 second\nGATE: 2/2 shards passed\n")
    assert subprocess.run(command, capture_output=True).returncode == 0
    result.write_text("FAIL 1/2 first\nFAIL 2/2 second\nGATE: 0/2 shards passed\n")
    candidate_failure.write_text("Uncaught exception: known failure\n")
    assert subprocess.run(command, capture_output=True).returncode == 1
    result.write_text("PASS 1/2 first\n")
    assert subprocess.run(command, capture_output=True).returncode == 2
