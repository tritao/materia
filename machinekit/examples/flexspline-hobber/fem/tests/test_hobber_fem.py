"""Known-answer checks of the hobber FE kernel (fe.py): a steel cantilever in bending and its first mode, and rigid-body / reduction identities."""
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import fe


class KernelTests(unittest.TestCase):
    def test_cantilever_tip_and_first_mode(self):
        r = fe.selftest(6.0)
        self.assertAlmostEqual(r["volume_error"], 0.0, places=9)
        self.assertLess(abs(r["tip_ratio"] - 1), 0.03)  # Timoshenko tip deflection, clamped-end Poisson restraint explains the 1 %
        self.assertLess(abs(r["f1_ratio"] - 1), 0.01)

    def test_rigid_master_carries_a_moment_without_stretching(self):
        # A 20 mm cube clamped on one face, the opposite face tied to a master; a unit force on the master 50 mm out gives the cube's
        # rigid-tip stiffness: deflection = F L^3 / (3 E I) with L from the clamp to the load point, within the tet10 mesh error.
        pts, (conn,) = fe.mesh_parts(["%s" % self._box()], 5.0)
        m = fe.Model()
        m.add_nodes(pts)
        m.add_body("b", conn, fe.STEEL)
        for n in fe.nodes_where(pts, lambda p: p[:, 0] < 1e-6):
            m.fix(n)
        mi = m.add_master("tip", (70.0, 10.0, 10.0))
        m.rigid(mi, fe.nodes_where(pts, lambda p: p[:, 0] > 20 - 1e-6))
        K, M, _ = m.system()
        T, free = m.reduction()
        u = m.static_solver(K, T, free, "direct")({m.master_dof(mi, 2): 100.0})
        E, I = 210000.0, 20 * 20 ** 3 / 12
        theory = 100 * 70 ** 3 / (3 * E * I)  # rigid beyond x = 20, clamped at 0: bound
        self.assertLess(u[m.master_dof(mi, 2)], theory * 1.05)
        self.assertGreater(u[m.master_dof(mi, 2)], 0.3 * theory)

    @staticmethod
    def _box():
        import os
        import tempfile
        gmsh = fe.import_gmsh()
        gmsh.initialize()
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("c")
        gmsh.model.occ.addBox(0, 0, 0, 20, 20, 20)
        gmsh.model.occ.synchronize()
        path = os.path.join(tempfile.mkdtemp(dir=os.environ.get("TMPDIR")), "box.step")
        gmsh.write(path)
        gmsh.finalize()
        return path


if __name__ == "__main__":
    unittest.main()
