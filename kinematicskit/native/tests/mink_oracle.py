"""mink oracle for kinematicskit's native differential IK (see NativeKinematicsTests.testMinkOracle).

Reads one request (JSON) from the file named by argv[1] and writes mink's answer to argv[2]:
  request: {mjcf, q, dt, damping (our lambda^2 = mink's damping), site, target_position,
            target_wxyz, position_cost, orientation_cost, gain, posture_cost, posture_target,
            velocity_limits (per joint, or null), joint_names}
  answer:  {velocity: [...]}
"""
import json
import sys

import mink
import mujoco
import numpy as np


def main() -> None:
    with open(sys.argv[1]) as handle:
        request = json.load(handle)
    model = mujoco.MjModel.from_xml_string(request["mjcf"])
    configuration = mink.Configuration(model, np.array(request["q"]))
    frame = mink.FrameTask(request["site"], "site", position_cost=request["position_cost"],
                           orientation_cost=request["orientation_cost"], gain=request["gain"])
    frame.set_target(mink.SE3.from_rotation_and_translation(
        mink.SO3(np.array(request["target_wxyz"])), np.array(request["target_position"])))
    tasks = [frame]
    if request["posture_cost"] > 0.0:
        posture = mink.PostureTask(model, cost=request["posture_cost"], gain=request["gain"])
        posture.set_target(np.array(request["posture_target"]))
        tasks.append(posture)
    # Our configuration limits are hard with no safety margin: gain 1.
    limits = [mink.ConfigurationLimit(model, gain=1.0)]
    if request["velocity_limits"] is not None:
        limits.append(mink.VelocityLimit(model, dict(zip(request["joint_names"], request["velocity_limits"]))))
    velocity = mink.solve_ik(configuration, tasks, request["dt"], solver="daqp", damping=request["damping"],
                             limits=limits)
    with open(sys.argv[2], "w") as handle:
        json.dump({"velocity": [float(v) for v in velocity]}, handle)


if __name__ == "__main__":
    main()
