#!/usr/bin/env python3
"""Converts unitree_rl_gym's pretrained G1 locomotion policy (TorchScript) to ONNX.

The TorchScript module keeps its LSTM state in module buffers and updates them
inside forward(). ONNX Runtime sessions are stateless, so the state becomes an
explicit input and output: (obs, h_in, c_in) -> (action, h_out, c_out).

Not part of the build. The exported file is checked in beside its licence, so
this script only needs to run again if the policy changes:

  pip install torch onnx onnxruntime numpy
  export_unitree_g1.py <unitree_rl_gym checkout> <output.onnx>

The export is verified against the original module over a random observation
sequence and refused if the two differ by more than 1e-4.
"""
import sys
import warnings

import numpy as np
import onnxruntime
import torch

warnings.filterwarnings("ignore")


class Stateless(torch.nn.Module):
    def __init__(self, scripted):
        super().__init__()
        self.lstm = torch.nn.LSTM(47, 64)
        self.actor = torch.nn.Sequential(torch.nn.Linear(64, 32), torch.nn.ELU(), torch.nn.Linear(32, 12))
        self.lstm.load_state_dict({k.removeprefix("memory."): v for k, v in scripted.state_dict().items()
                                   if k.startswith("memory.")})
        self.actor.load_state_dict({k.removeprefix("actor."): v for k, v in scripted.state_dict().items()
                                    if k.startswith("actor.")})

    def forward(self, obs, h_in, c_in):
        out, (h, c) = self.lstm(obs.unsqueeze(0), (h_in, c_in))
        return self.actor(out.squeeze(0)), h, c


def main(repo, output):
    scripted = torch.jit.load(f"{repo}/deploy/pre_train/g1/motion.pt").eval()
    model = Stateless(scripted).eval()
    obs, h, c = torch.zeros(1, 47), torch.zeros(1, 1, 64), torch.zeros(1, 1, 64)
    torch.onnx.export(model, (obs, h, c), output, dynamo=False, opset_version=17,
                      input_names=["obs", "h_in", "c_in"], output_names=["action", "h_out", "c_out"])

    session = onnxruntime.InferenceSession(output, providers=["CPUExecutionProvider"])
    rng = np.random.default_rng(1)
    hs, cs = np.zeros((1, 1, 64), np.float32), np.zeros((1, 1, 64), np.float32)
    worst = 0.0
    for _ in range(200):
        x = rng.normal(0.0, 0.5, (1, 47)).astype(np.float32)
        expected = scripted(torch.from_numpy(x)).detach().numpy()
        action, hs, cs = session.run(None, {"obs": x, "h_in": hs, "c_in": cs})
        worst = max(worst, float(np.abs(action - expected).max()))
    print(f"max |onnx - torchscript| over 200 steps: {worst:.3e}")
    if worst > 1e-4:
        sys.exit("export does not match the TorchScript policy")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
