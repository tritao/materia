import json
from pathlib import Path
import tempfile
import unittest
from packet import digest
from pose_sweep import reusable


class PoseResumeChecks(unittest.TestCase):
    def test_parallel_result_cannot_be_reused_by_serial_runner(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);manifest=root/'manifest.json';config=root/'config.json';report=root/'report.json'
            manifest.write_text('{}');config.write_text('{}')
            data={'converged':True,'manifestSha256':digest(manifest),'configurationSha256':digest(config),
                  'sourceSha256':{},'solverThreads':4}
            report.write_text(json.dumps(data));self.assertFalse(reusable(report,manifest,config))
            data['solverThreads']=1;report.write_text(json.dumps(data));self.assertTrue(reusable(report,manifest,config))
