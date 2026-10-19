from pathlib import Path
import signal
import tempfile
import unittest
from unittest.mock import patch
from resource_guard import owned_solvers,run

class FakeProcess:
    def __init__(self,codes):self.codes=iter(codes);self.returncode=None
    def poll(self):self.returncode=next(self.codes);return self.returncode
    def wait(self):return self.returncode

class ResourceGuardChecks(unittest.TestCase):
    def test_only_own_large_ccx_in_current_study(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);proc=root/'proc';proc.mkdir();exe=root/'ccx';exe.touch()
            other=root/'other';other.touch();output=root/'study';output.mkdir();outside=root/'outside';outside.mkdir()
            for pid,binary,cwd,group,rss in [(11,exe,output,'heavy-mill-pose-study',2000000),
                (12,other,output,'heavy-mill-pose-study',2000000),(13,exe,outside,'heavy-mill-pose-study',2000000),
                (14,exe,output,'another-task',2000000),(15,exe,output,'heavy-mill-pose-study',1000)]:
                item=proc/str(pid);item.mkdir();(item/'exe').symlink_to(binary);(item/'cwd').symlink_to(cwd)
                (item/'cgroup').write_text('0::/'+group+'.scope\n');(item/'status').write_text(f'VmRSS: {rss} kB\n')
            self.assertEqual([p['pid'] for p in owned_solvers(exe,[output],proc)],[11])

    def test_retry_only_after_own_pressure_stop(self):
        with tempfile.TemporaryDirectory() as folder:
            item={'pid':123,'cwd':Path(folder),'rssBytes':2*1024**3}
            with patch('resource_guard.subprocess.Popen',side_effect=[FakeProcess([None,1]),FakeProcess([0])]) as start, \
                 patch('resource_guard.available_bytes',return_value=3*1024**3), \
                 patch('resource_guard.owned_solvers',return_value=[item]), \
                 patch('resource_guard.os.kill') as kill,patch('resource_guard.time.sleep'):
                self.assertEqual(run(['solver'],Path('/ccx'),[Path(folder)]),0)
                self.assertEqual(start.call_count,2);kill.assert_called_once_with(123,signal.SIGTERM)
                self.assertTrue((Path(folder)/'resource-stop.json').exists())

    def test_solver_error_is_not_hidden_by_a_pressure_stop(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder);(path/'solver.log').write_text('*ERROR invalid model')
            with patch('resource_guard.subprocess.Popen',return_value=FakeProcess([None,1])) as start, \
                 patch('resource_guard.available_bytes',return_value=3*1024**3), \
                 patch('resource_guard.owned_solvers',return_value=[{'pid':123,'cwd':path,'rssBytes':2*1024**3}]), \
                 patch('resource_guard.os.kill'),patch('resource_guard.time.sleep'):
                self.assertEqual(run(['solver'],Path('/ccx'),[path]),1)
                self.assertEqual(start.call_count,1)

    def test_numerical_error_is_not_retried(self):
        with patch('resource_guard.subprocess.Popen',return_value=FakeProcess([1])) as start:
            self.assertEqual(run(['solver'],Path('/ccx'),[Path('/study')]),1)
            self.assertEqual(start.call_count,1)

if __name__=='__main__':unittest.main()
