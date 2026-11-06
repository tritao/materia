"""Z saddle (GE-Z-saddle, 13 mm milled plate with the eRob pocket): stiffness at the eRob seat and first modes with the head mass.

Geometry: the exported STEP with the block-face holes (clearance, counterbores, dowels) filled so they do not drive the mesh (volume +1 %; the 16
M4 holes in the pocket floor stay). Plate normal x: block faces at x = 148, pocket floor (the eRob's seat) at x = 140.
Supports: the four Z blocks (y = +-145, z = 0 and 110), each held in all directions over the patches (r = 8) round its four M4 bolts (`bolts`) or over the
whole bolt rectangle plus a margin (`blocks`, upper bound). The Z nut block is not a support (conservative: the screw's lateral stiffness is not known).
Seat: the pocket floor annulus r 46..56 about the A axis (y = z = 0), rigid, coupled to a 6-DOF master on the axis at the floor (x = 140). The 6 x 6
stiffness at the master gives the tilt (about y, the head's overturning moment), the torque about the A axis (x) and the in-plane stiffnesses; the load
cases put the head weight (at its centre of mass) and 100 N at the hob through the same rigid seat. Modes: the head (36.7 kg, rigid, spherical-inertia
estimate) at its centre of mass on the same seat; the eRob's own bearing and housing compliance are not in the model (upper bound).
"""
import json
import os
import sys
import numpy as np
import fe

HEAD_INERTIA_SIZE = 150.0  # mm, edge of the cube whose inertia stands in for the head's (estimate)


def to_authored(pose, p):
    o = np.array(pose["origin"])
    return np.array([np.dot(a, np.asarray(p) - o) for a in pose["axes"]])


def dir_authored(pose, v):
    return np.array([np.dot(a, v) for a in pose["axes"]])


class Saddle:
    def __init__(self, export_dir, size=12.0, curvature=3):
        self.export = ex = json.load(open(f"{export_dir}/hobber-fem-export.json"))
        part = next(p for p in ex["parts"] if p["name"] == "GE-Z-saddle")
        self.pose = part["pose"]
        holes = [h for h in part["holes"] if h["id"] != "P1" and not (35 <= int(h["id"][1:]) <= 50)]
        self.block_bolts = [(h["position"][1], h["position"][2]) for h in part["holes"] if h["id"][0] == "J" and int(h["id"][1:]) <= 32 and h["kind"] == "clearance"]
        tmp = os.environ.get("TMPDIR", "/tmp")
        filled = f"{tmp}/GE-Z-saddle-filled.step"
        self.vol_filled, self.vol_step = fe.filled_step(f"{export_dir}/GE-Z-saddle.step", holes, filled)
        self.nodes, (self.conn,) = fe.mesh_parts([filled], size, min_size=min(size / 2, 4.0), curvature=curvature)
        self.ids = fe.used_nodes(self.conn)
        self.size = size
        head = ex["head"]
        self.head_mass = head["mass"] * 1e-3
        self.head_com = to_authored(self.pose, [head["com"][0], head["com"][1], head["com"][2]])
        self.hob = to_authored(self.pose, head["hob"])
        self.gravity = dir_authored(self.pose, np.array([0.0, 0.0, -1.0]))

    def model(self, support, seat="ring"):
        m = fe.Model()
        m.add_nodes(self.nodes)
        m.add_body("saddle", self.conn, fe.STEEL)
        p = self.nodes[self.ids]
        face = np.abs(p[:, 0] - 148.0) < 1e-6
        held = set()
        if support == "bolts":
            for y, z in self.block_bolts:
                held |= set(self.ids[face & (np.hypot(p[:, 1] - y, p[:, 2] - z) <= 8.0)].tolist())
        else:
            for cy in (-145.0, 145.0):
                for cz in (0.0, 110.0):
                    held |= set(self.ids[face & (np.abs(p[:, 1] - cy) <= 22) & (np.abs(p[:, 2] - cz) <= 26)].tolist())
        for n in held:
            m.fix(n)
        r = np.hypot(p[:, 1], p[:, 2])
        floor = np.abs(p[:, 0] - 140.0) < 1e-6
        lo, hi = (46.0, 56.0) if seat == "ring" else (0.0, 56.0)
        self.seat_nodes = [int(n) for n in self.ids[floor & (r >= lo) & (r <= hi)]]
        self.held = len(held)
        return m

    def stiffness_matrix(self, support, seat="ring", method="auto"):
        m = self.model(support, seat)
        mi = m.add_master("seat", (140.0, 0.0, 0.0))
        m.rigid(mi, self.seat_nodes)
        K, M, vols = m.system()
        T, free = m.reduction()
        C = np.zeros((6, 6))
        solve = m.static_solver(K, T, free, method)
        for k in range(6):
            u = solve({m.master_dof(mi, k): 1.0})
            C[:, k] = [u[m.master_dof(mi, j)] for j in range(6)]
        Kmat = np.linalg.inv(C)
        return Kmat, C, vols

    def load_cases(self, C):
        """Rotation (rad) and hob-tip displacement (um) from the saddle compliance for the head weight and 100 N cut, applied at their points."""
        seat = np.array([140.0, 0.0, 0.0])
        g = 9.80665
        w = self.head_mass * 1e3 * g * self.gravity  # N (mass kg)
        out = {}
        for name, forces in (("weight", [(self.head_com, w)]), ("weight_and_100N_cut_down", [(self.head_com, w), (self.hob, 100.0 * self.gravity)])):
            W = np.zeros(6)
            for p, f in forces:
                W[:3] += f
                W[3:] += np.cross(p - seat, f)
            u = C @ W
            tip = u[:3] + np.cross(u[3:], self.hob - seat)
            out[name] = {"wrench_N_Nmm": W.tolist(), "moment_Nm": (W[3:] / 1000).tolist(), "rotation_urad": (u[3:] * 1e6).tolist(), "seat_um": (u[:3] * 1e3).tolist(),
                         "hob_tip_um": (tip * 1e3).tolist(), "hob_tip_norm_um": float(np.linalg.norm(tip) * 1e3)}
        return out

    def modes(self, support, seat="ring", count=6, method="auto"):
        m = self.model(support, seat)
        s = HEAD_INERTIA_SIZE
        I = self.head_mass * s * s / 6.0 * np.eye(3)
        mi = m.add_master("head", self.head_com, self.head_mass, I)
        m.rigid(mi, self.seat_nodes)
        K, M, vols = m.system()
        T, _ = m.reduction()
        f, V = m.modes(count, K, M, T, method=method)
        dirs = []
        for i in range(count):
            d = np.array([V[m.master_dof(mi, k), i] for k in range(6)])
            dirs.append({"f": float(f[i]), "head_dir": (d[:3] / (np.linalg.norm(d[:3]) + 1e-30)).round(3).tolist(),
                         "head_rot_rel": (d[3:] / (np.linalg.norm(d[:3]) + 1e-30) * 100).round(3).tolist()})
        return {"frequencies_Hz": [float(x) for x in f], "modes": dirs}


def run(export_dir, size=12.0, method="auto", do_modes=True):
    s = Saddle(export_dir, size)
    out = {"size": size, "elements": len(s.conn), "dofs": 3 * len(s.ids), "volume_step": s.vol_step, "volume_filled": s.vol_filled,
           "head_mass_kg": s.head_mass * 1e3, "head_com_authored": s.head_com.tolist(), "hob_authored": s.hob.tolist(), "gravity_authored": s.gravity.tolist()}
    for support in ("bolts", "blocks"):
        K, C, vols = s.stiffness_matrix(support, method=method)
        res = {"held_nodes": s.held, "seat_nodes": len(s.seat_nodes), "volume_mesh": vols["saddle"],
               "stiffness_diag": {"x_N_per_um": K[0, 0] / 1000, "y_N_per_um": K[1, 1] / 1000, "z_N_per_um": K[2, 2] / 1000,
                                  "Mx_kNm_per_rad": K[3, 3] / 1e6, "My_kNm_per_rad": K[4, 4] / 1e6, "Mz_kNm_per_rad": K[5, 5] / 1e6},
               "compliance_diag_inverse": {"Mx_kNm_per_rad": 1 / C[3, 3] / 1e6, "My_kNm_per_rad": 1 / C[4, 4] / 1e6, "Mz_kNm_per_rad": 1 / C[5, 5] / 1e6},
               "loads": s.load_cases(C)}
        if do_modes:
            res["modes"] = s.modes(support, method=method)
        out[support] = res
    return out


if __name__ == "__main__":
    d, out = sys.argv[1], sys.argv[2]
    sizes = [float(x) for x in sys.argv[3:]] or [14.0, 10.0]
    results = []
    for size in sizes:
        results.append(run(d, size, method="direct"))
        json.dump(results, open(out, "w"), indent=1)
        print(json.dumps(results[-1])[:300], flush=True)
