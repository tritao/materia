#!/usr/bin/env python3
"""Runs the exported G1 policy in plain MuJoCo, the way unitree_rl_gym's
deploy_mujoco.py does, and reports how far the base travels and whether it
stays up. It is the yardstick for what SimKit's run should look like: the
policy was trained in Isaac Gym, and this is Unitree's own sim-to-sim setup.

  reference_unitree_g1.py <unitree_rl_gym checkout> <policy.onnx> <seconds> <vx> <vy> <wz>
      [push-time push-vx push-vy]

Not part of the build (needs `pip install mujoco onnxruntime numpy`).
"""
import sys

import mujoco
import numpy as np
import onnxruntime

KP = np.array([100, 100, 100, 150, 40, 40] * 2, np.float32)
KD = np.array([2, 2, 2, 4, 2, 2] * 2, np.float32)
DEFAULT = np.array([-0.1, 0, 0, 0.3, -0.2, 0] * 2, np.float32)


def gravity(q):
    w, x, y, z = q
    return np.array([2 * (-z * x + w * y), -2 * (z * y + w * x), 1 - 2 * (w * w + z * z)])


def main(repo, policy, seconds, cmd, push):
    m = mujoco.MjModel.from_xml_path(f"{repo}/resources/robots/g1_description/scene.xml")
    d = mujoco.MjData(m)
    m.opt.timestep = 0.002
    session = onnxruntime.InferenceSession(policy, providers=["CPUExecutionProvider"])
    h = np.zeros((1, 1, 64), np.float32)
    c = np.zeros((1, 1, 64), np.float32)
    action = np.zeros(12, np.float32)
    target = DEFAULT.copy()
    cmd = np.array(cmd, np.float32)
    scale = np.array([2.0, 2.0, 0.25], np.float32)
    counter, worst_tilt, lowest = 0, 0.0, 9.0
    start = d.qpos[:2].copy()
    steps = int(seconds / 0.002)
    for _ in range(steps):
        tau = (target - d.qpos[7:]) * KP - d.qvel[6:] * KD
        d.ctrl[:] = tau
        mujoco.mj_step(m, d)
        counter += 1
        if push and abs(counter * 0.002 - push[0]) < 1e-9:
            d.qvel[0] += push[1]
            d.qvel[1] += push[2]
        if counter % 10 == 0:
            phase = (counter * 0.002) % 0.8 / 0.8
            obs = np.concatenate([d.qvel[3:6] * 0.25, gravity(d.qpos[3:7]), cmd * scale,
                                  (d.qpos[7:] - DEFAULT), d.qvel[6:] * 0.05, action,
                                  [np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)]]).astype(np.float32)
            action, h, c = session.run(None, {"obs": obs[None], "h_in": h, "c_in": c})
            action = action[0]
            target = action * 0.25 + DEFAULT
        g = gravity(d.qpos[3:7])
        worst_tilt = max(worst_tilt, float(np.arccos(np.clip(g[2] * -1.0, -1, 1))))
        lowest = min(lowest, d.qpos[2]) if counter > 500 else lowest
    travelled = d.qpos[:2] - start
    print(f"{seconds:g}s cmd={cmd.tolist()}: travelled ({travelled[0]:.3f}, {travelled[1]:.3f}) m, "
          f"final height {d.qpos[2]:.3f}, worst tilt {worst_tilt:.3f} rad, lowest {lowest:.3f}")


if __name__ == "__main__":
    a = sys.argv
    if len(a) < 7:
        sys.exit(__doc__)
    main(a[1], a[2], float(a[3]), [float(a[4]), float(a[5]), float(a[6])],
         [float(x) for x in a[7:10]] if len(a) >= 10 else None)
