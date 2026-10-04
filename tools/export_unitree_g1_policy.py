#!/usr/bin/env python3
"""Convert Unitree's pretrained G1 walking policy (TorchScript) to ONNX with explicit LSTM state.

Needs torch and onnx; neither is a build or runtime dependency of the workspace. The TorchScript
module keeps its LSTM hidden and cell state in internal buffers, which a plain trace would freeze
into constants. This rebuilds the network (LSTM 47->64, Linear 64->32, ELU, Linear 32->12) with
the state as inputs and outputs, then checks the ONNX model against the original over a recurrent
rollout before writing it.

  python3 tools/export_unitree_g1_policy.py motion.pt policy.onnx

Source: unitreerobotics/unitree_rl_gym, deploy/pre_train/g1/motion.pt (BSD-3-Clause).
"""
import sys

import numpy as np
import onnxruntime
import torch


class StatefulPolicy(torch.nn.Module):
    def __init__(self, scripted):
        super().__init__()
        self.memory = torch.nn.LSTM(47, 64, num_layers=1)
        self.actor = torch.nn.Sequential(
            torch.nn.Linear(64, 32), torch.nn.ELU(), torch.nn.Linear(32, 12))
        state = scripted.state_dict()
        self.memory.load_state_dict({k[len('memory.'):]: v for k, v in state.items()
                                     if k.startswith('memory.')})
        self.actor.load_state_dict({k[len('actor.'):]: v for k, v in state.items()
                                    if k.startswith('actor.')})

    def forward(self, obs, h, c):
        out, (h, c) = self.memory(obs.unsqueeze(0), (h, c))
        return self.actor(out.squeeze(0)), h, c


def main(src, dst):
    scripted = torch.jit.load(src, map_location='cpu')
    model = StatefulPolicy(scripted).eval()
    obs = torch.zeros(1, 47)
    h = torch.zeros(1, 1, 64)
    c = torch.zeros(1, 1, 64)
    torch.onnx.export(
        model, (obs, h, c), dst, input_names=['obs', 'h', 'c'],
        output_names=['action', 'h_out', 'c_out'], opset_version=17, dynamo=False)

    # The original starts from its own stored state, which is zero in the shipped file.
    assert float(scripted.hidden_state.abs().max()) == 0.0
    assert float(scripted.cell_state.abs().max()) == 0.0
    session = onnxruntime.InferenceSession(dst, providers=['CPUExecutionProvider'])
    rng = np.random.default_rng(0)
    worst = 0.0
    hh, cc = np.zeros((1, 1, 64), np.float32), np.zeros((1, 1, 64), np.float32)
    for _ in range(200):
        x = rng.normal(size=(1, 47)).astype(np.float32)
        with torch.no_grad():
            expected = scripted(torch.from_numpy(x)).numpy()
        action, hh, cc = session.run(None, {'obs': x, 'h': hh, 'c': cc})
        worst = max(worst, float(np.abs(action - expected).max()))
    print(f'max |onnx - torchscript| over a 200-step rollout: {worst:.3e}')
    assert worst < 1e-4, 'ONNX model does not reproduce the TorchScript policy'


if __name__ == '__main__':
    main(*sys.argv[1:3])
