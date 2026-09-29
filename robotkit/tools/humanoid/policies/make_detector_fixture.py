#!/usr/bin/env python3
"""Regenerate the tiny detector fixtures with `uv run --with onnx --with numpy python make_detector_fixture.py`.

Each output is [1, 2, 6]: center x/y, width, height, score, class.
The image mean shifts every value by 0.001 * mean so preprocessing is testable.
"""
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper


def make(path: Path, layout: str, size: int = 4, uint8: bool = False, dynamic: bool = False):
    shape = [1, 3, size, size] if layout == "nchw" else [1, size, size, 3]
    if dynamic:
        shape = ["batch", 3, "height", "width"]
    image = helper.make_tensor_value_info("image", TensorProto.UINT8 if uint8 else TensorProto.FLOAT, shape)
    output = helper.make_tensor_value_info("detections", TensorProto.FLOAT, [1, 2, 6])
    base = np.array([[[2, 2, 2, 2, .9, 1], [2.2, 2, 2, 2, .8, 1]]], np.float32)
    nodes = ([helper.make_node("Cast", ["image"], ["float_image"], to=TensorProto.FLOAT)] if uint8 else []) + [
        helper.make_node("ReduceMean", ["float_image" if uint8 else "image"], ["mean"], keepdims=0),
        helper.make_node("Mul", ["mean", "factor"], ["shift"]),
        helper.make_node("Add", ["boxes", "shift"], ["detections"]),
    ]
    graph = helper.make_graph(nodes, "robotkit-detector-fixture", [image], [output], [
        numpy_helper.from_array(base, "boxes"),
        numpy_helper.from_array(np.array(.001, np.float32), "factor"),
    ])
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 10
    onnx.checker.check_model(model)
    onnx.save(model, path)


if __name__ == "__main__":
    root = Path(__file__).resolve().parents[3] / "inference/tests/fixtures"
    make(root / "detector.onnx", "nchw")
    make(root / "detector_nhwc.onnx", "nhwc")
    make(root / "detector_large.onnx", "nchw", 512)
    make(root / "detector_uint8.onnx", "nchw", uint8=True)
    make(root / "detector_dynamic.onnx", "nchw", dynamic=True)
