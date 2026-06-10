#!/usr/bin/env python3
"""Dependency-free ONNX tensor contract inspector for the integrated pose model.

This reads enough protobuf wire format to inspect ModelProto.graph inputs/outputs.
It does not execute the model; runtime output keypoint count is still validated by
RtmPoseSession warm-up + ValidateRtmPoseModelContract when ONNX Runtime is available.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable, List, Tuple, Union

WIRE_VARINT = 0
WIRE_64 = 1
WIRE_LEN = 2
WIRE_32 = 5
FLOAT32 = 1

Dim = Union[int, str, None]
Field = Tuple[int, int, object]


def read_varint(buf: bytes, i: int, end: int) -> Tuple[int, int]:
    shift = 0
    value = 0
    while i < end:
        b = buf[i]
        i += 1
        value |= (b & 0x7F) << shift
        if not (b & 0x80):
            return value, i
        shift += 7
    raise ValueError("truncated varint")


def iter_fields(buf: bytes, start: int = 0, end: int | None = None) -> Iterable[Field]:
    if end is None:
        end = len(buf)
    i = start
    while i < end:
        key, i = read_varint(buf, i, end)
        field = key >> 3
        wire = key & 7
        if wire == WIRE_VARINT:
            value, i = read_varint(buf, i, end)
            yield field, wire, value
        elif wire == WIRE_64:
            yield field, wire, buf[i : i + 8]
            i += 8
        elif wire == WIRE_LEN:
            length, data_start = read_varint(buf, i, end)
            data_end = data_start + length
            yield field, wire, buf[data_start:data_end]
            i = data_end
        elif wire == WIRE_32:
            yield field, wire, buf[i : i + 4]
            i += 4
        else:
            raise ValueError(f"unsupported wire type {wire}")


def parse_shape(buf: bytes) -> List[Dim]:
    dims: List[Dim] = []
    for field, wire, value in iter_fields(buf):
        if field == 1 and wire == WIRE_LEN:
            dim_value: Dim = None
            dim_param: Dim = None
            for dim_field, dim_wire, dim_data in iter_fields(value):
                if dim_field == 1 and dim_wire == WIRE_VARINT:
                    dim_value = int(dim_data)
                elif dim_field == 2 and dim_wire == WIRE_LEN:
                    dim_param = bytes(dim_data).decode("utf-8", "replace")
            dims.append(dim_param if dim_param is not None else dim_value)
    return dims


def parse_type(buf: bytes) -> dict:
    out: dict = {}
    for field, wire, value in iter_fields(buf):
        if field == 1 and wire == WIRE_LEN:
            for tensor_field, tensor_wire, tensor_data in iter_fields(value):
                if tensor_field == 1 and tensor_wire == WIRE_VARINT:
                    out["elem_type"] = int(tensor_data)
                elif tensor_field == 2 and tensor_wire == WIRE_LEN:
                    out["shape"] = parse_shape(tensor_data)
    return out


def parse_value_info(buf: bytes) -> dict:
    out: dict = {}
    for field, wire, value in iter_fields(buf):
        if field == 1 and wire == WIRE_LEN:
            out["name"] = bytes(value).decode("utf-8", "replace")
        elif field == 2 and wire == WIRE_LEN:
            out["type"] = parse_type(value)
    return out


def read_graph(data: bytes) -> bytes:
    for field, wire, value in iter_fields(data):
        if field == 7 and wire == WIRE_LEN:
            return bytes(value)
    raise ValueError("ONNX ModelProto has no graph")


def inspect(path: Path) -> Tuple[List[dict], List[dict]]:
    graph = read_graph(path.read_bytes())
    inputs: List[dict] = []
    outputs: List[dict] = []
    for field, wire, value in iter_fields(graph):
        if field == 11 and wire == WIRE_LEN:
            inputs.append(parse_value_info(value))
        elif field == 12 and wire == WIRE_LEN:
            outputs.append(parse_value_info(value))
    return inputs, outputs


def dim_matches_dynamic_or_value(dim: Dim, expected: int) -> bool:
    return dim == expected or isinstance(dim, str) or dim is None


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(message)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("model", type=Path)
    args = parser.parse_args()

    inputs, outputs = inspect(args.model)
    print("inputs:")
    for item in inputs:
        print(item)
    print("outputs:")
    for item in outputs:
        print(item)

    require(len(inputs) == 1, f"expected 1 input, got {len(inputs)}")
    inp = inputs[0]
    inp_type = inp.get("type", {})
    inp_shape = inp_type.get("shape", [])
    require(inp.get("name") == "input", f"expected input name 'input', got {inp.get('name')}")
    require(inp_type.get("elem_type") == FLOAT32, "input must be float32")
    require(len(inp_shape) == 4, f"input must be rank-4, got {inp_shape}")
    require(dim_matches_dynamic_or_value(inp_shape[0], 1), f"input batch must be dynamic or 1, got {inp_shape}")
    require(inp_shape[1:] == [3, 384, 288], f"input must be [batch,3,384,288], got {inp_shape}")

    # Cocktail14 primary model: 2 outputs, either 26 or 133 keypoints.
    # Cocktail14 26-keypoint variant preserves 26-keypoint structure with multi-dataset training.
    # Cocktail14 133-keypoint variant uses COCO-WholeBody topology.
    # RTMW3D 3D model: 3 outputs, 133 keypoints.
    if len(outputs) == 2:
        expected = [("simcc_x", (26, 133), 576), ("simcc_y", (26, 133), 768)]
    elif len(outputs) == 3:
        expected = [(None, (133,), 576), (None, (133,), 768), (None, (133,), 576)]
    else:
        raise SystemExit(f"expected 2 Cocktail14 SimCC outputs or 3 RTMW3D SimCC outputs, got {len(outputs)}")

    for out, (name, keypoint_options, bins) in zip(outputs, expected):
        out_type = out.get("type", {})
        shape = out_type.get("shape", [])
        label = name or out.get("name") or "output"
        if name is not None:
            require(out.get("name") == name, f"expected output {name}, got {out.get('name')}")
        require(out_type.get("elem_type") == FLOAT32, f"{label} must be float32")
        require(len(shape) == 3, f"{label} must be rank-3, got {shape}")
        require(dim_matches_dynamic_or_value(shape[0], 1), f"{label} batch must be dynamic or 1, got {shape}")
        keypoint_ok = shape[1] in keypoint_options or isinstance(shape[1], str) or shape[1] is None
        require(keypoint_ok, f"{label} keypoint count must be one of {keypoint_options}, got {shape}")
        require(shape[2] == bins, f"{label} bin count must be {bins}, got {shape}")

    keypoint_dims = [out.get("type", {}).get("shape", [None, None, None])[1] for out in outputs]
    if keypoint_dims not in ([26, 26], [133, 133, 133]):
        print("note: ONNX metadata has symbolic keypoint dimensions; runtime warm-up must prove concrete output keypoint counts.")


if __name__ == "__main__":
    main()
