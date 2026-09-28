#!/usr/bin/env python3
"""Writes robotkit/policy/tests/fixtures/affine_state.onnx, the tiny model that
the policy library's tests run, and prints golden values for the G1 policy.

  y     = a @ W + b            (a: [1,3], W: [3,2], b: [2])
  s_out = s_in + y             (s: [1,2], an explicit recurrent state)

Not part of the build (needs `pip install torch onnx onnxruntime numpy`).
"""
import sys
import warnings

import numpy as np
import onnxruntime
import torch

warnings.filterwarnings("ignore")


class Affine(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.register_buffer("w", torch.tensor([[1.0, 2.0], [0.5, -1.0], [-2.0, 0.25]]))
        self.register_buffer("b", torch.tensor([0.1, -0.2]))

    def forward(self, a, s):
        y = a @ self.w + self.b
        return y, s + y


def main(output_dir, g1_model):
    torch.onnx.export(Affine().eval(), (torch.zeros(1, 3), torch.zeros(1, 2)), f"{output_dir}/affine_state.onnx",
                      dynamo=False, opset_version=17, input_names=["a", "s"], output_names=["y", "s_out"])
    session = onnxruntime.InferenceSession(g1_model, providers=["CPUExecutionProvider"])
    h = np.zeros((1, 1, 64), np.float32)
    c = np.zeros((1, 1, 64), np.float32)
    for k in range(100):
        obs = (0.5 * np.sin(0.1 * (k * 47 + np.arange(47)))).astype(np.float32)[None]
        action, h, c = session.run(None, {"obs": obs, "h_in": h, "c_in": c})
        if k in (0, 99):
            print(f"step {k}: action =", ", ".join(f"{v:.9g}" for v in action[0]))
            print(f"step {k}: h[0..3] =", ", ".join(f"{v:.9g}" for v in h.flatten()[:4]))


if __name__ == "__main__":
    main(*sys.argv[1:3]) if len(sys.argv) == 3 else sys.exit(__doc__)
