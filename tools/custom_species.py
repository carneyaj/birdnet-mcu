#!/usr/bin/env python3
"""Build a worldwide V2 classifier for a text list of species."""
import argparse
from copy import copy
import importlib.metadata
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import zlib

import flatbuffers
import numpy as np
from ai_edge_litert import schema_py_generated as schema
from ai_edge_litert.interpreter import Interpreter, OpResolverType

from species_profile import DEFAULT_MODELS, HUMAN_VOCAL, WORLDWIDE, load_labels, read_pinned, resolve_species, sha256


ARENA_BYTES = 1088 * 1024


def read_model(data):
    return schema.ModelT.InitFromObj(schema.Model.GetRootAsModel(data, 0))


def head(model, count):
    graph, = model.subgraphs
    op = graph.operators[-1]
    if (model.operatorCodes[op.opcodeIndex].builtinCode != schema.BuiltinOperator.FULLY_CONNECTED
            or len(op.inputs) != 3 or list(op.outputs) != list(graph.outputs)):
        raise ValueError("Expected the BirdNET V2 fully connected classifier")
    embedding, weights, bias, scores = [graph.tensors[i] for i in (*op.inputs, op.outputs[0])]
    for tensor, shape, dtype in (
        (weights, [count, 1024], schema.TensorType.INT8),
        (bias, [count], schema.TensorType.INT32),
        (scores, [1, count], schema.TensorType.INT8),
    ):
        if list(tensor.shape) != shape or tensor.type != dtype:
            raise ValueError("Unexpected classifier tensor shape or type")
    return embedding, weights, bias, scores


def replace_head(model, parent, indices, source_count, dest_count):
    source_embedding, *source = head(parent, source_count)
    dest_embedding, *dest = head(model, dest_count)
    for field in ("scale", "zeroPoint"):
        if not np.array_equal(getattr(source_embedding.quantization, field),
                              getattr(dest_embedding.quantization, field)):
            raise ValueError("Target and parent embedding quantization differ")
    for original, resized in zip(source[:2], dest[:2]):
        buffer = model.buffers[resized.buffer]
        references = sum(t.buffer == resized.buffer for t in model.subgraphs[0].tensors)
        if references != 1:
            raise ValueError("Classifier buffer is shared")
        data = parent.buffers[original.buffer].data.reshape(source_count, -1)
        buffer.data = data[indices].flatten()
        buffer.offset = buffer.size = 0
        resized.shape = original.shape.copy()
        resized.shape[0] = len(indices)
        resized.shapeSignature = None
        resized.quantization = copy(original.quantization)
        for field in ("scale", "zeroPoint", "min", "max"):
            values = getattr(original.quantization, field)
            if values is not None and len(values) > 1:
                if len(values) != source_count or original.quantization.quantizedDimension != 0:
                    raise ValueError("Unexpected per-class quantization")
                values = values[indices]
            setattr(resized.quantization, field, values)
    output = dest[-1]
    output.shape = np.array([1, len(indices)], dtype=np.int32)
    output.shapeSignature = None
    output.quantization = source[-1].quantization


class AlignedBuilder(flatbuffers.Builder):
    """Align byte buffers for the Xtensa SIMD kernels and reuse zero-point vectors."""

    def __init__(self):
        super().__init__(8 << 20)
        self.zero_vectors = {}

    def CreateNumpyVector(self, array):
        if array.dtype == np.uint8:
            self.Prep(16, array.nbytes)
        if array.dtype == np.int64 and not array.any():
            if array.size not in self.zero_vectors:
                self.zero_vectors[array.size] = super().CreateNumpyVector(array)
            return self.zero_vectors[array.size]
        return super().CreateNumpyVector(array)


def serialize(model, aligned=False):
    builder = AlignedBuilder() if aligned else flatbuffers.Builder(0)
    builder.Finish(model.Pack(builder), file_identifier=b"TFL3")
    return bytes(builder.Output())


def esp32_model(template, parent, indices, parent_count, template_count):
    model = read_model(template)
    _, previous_weights, _, _ = head(model, template_count)
    head_buffer = previous_weights.buffer
    untouched = {}
    for number, buffer in enumerate(model.buffers):
        if not buffer.offset or number == head_buffer:
            continue
        length, = struct.unpack_from("<I", template, buffer.offset)
        start = buffer.offset
        untouched[number] = template[start:start + 4 + length]
    replace_head(model, parent, indices, parent_count, template_count)
    new_weights = model.buffers[head_buffer]
    compressed = zlib.compress(new_weights.data.tobytes(), level=9)
    new_weights.size, new_weights.offset, new_weights.data = new_weights.data.nbytes, 8, None
    blocks = {**untouched, head_buffer: struct.pack("<I", len(compressed)) + compressed}

    # Keep the external buffers in the same length-prefixed format as the field image.
    graph_bytes = len(serialize(model, aligned=True))
    offset = graph_bytes
    for number, block in blocks.items():
        model.buffers[number].offset = offset
        offset += len(block)
    graph = serialize(model, aligned=True)
    if len(graph) != graph_bytes:
        raise ValueError("Unstable XIAO model layout")
    return graph + b"".join(blocks.values())


def io_contract(data, count):
    model = read_model(data)
    graph, = model.subgraphs
    input_index, = graph.inputs
    output_index, = graph.outputs
    tensors = {}
    for name, index, shape in (("input", input_index, [1, 96, 511, 2]), ("output", output_index, [1, count])):
        tensor = graph.tensors[index]
        quant = tensor.quantization
        if (list(tensor.shape) != shape or tensor.type != schema.TensorType.INT8
                or len(quant.scale) != 1 or len(quant.zeroPoint) != 1):
            raise ValueError(f"Unexpected {name} tensor in target model")
        tensors[name] = {"shape": shape, "scale": float(quant.scale[0]), "zero_point": int(quant.zeroPoint[0])}
    return tensors


def compile_grove(work, portable, count, expected_io):
    if importlib.metadata.version("ethos-u-vela") != "5.2.0":
        raise ValueError("Grove compilation uses ethos-u-vela 5.2.0")
    tool = Path(sys.executable).with_name("vela")
    stdout = subprocess.run(
        [str(tool), "--config", "Arm/vela.ini", "--accelerator-config", "ethos-u55-64",
         "--system-config", "Ethos_U55_Deep_Embedded", "--memory-mode", "Shared_Sram",
         "--optimise", "Performance", "--output-dir", str(work), str(portable)],
        check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    ).stdout
    packed = (work / "model_int8_vela.tflite").read_bytes()
    model = read_model(packed)
    graph, = model.subgraphs
    if (len(graph.operators) != 1
            or model.operatorCodes[graph.operators[0].opcodeIndex].customCode != b"ethos-u"):
        raise ValueError("Vela must compile the full Grove graph into one Ethos-U operator")
    if io_contract(packed, count) != expected_io:
        raise ValueError("Vela changed the classifier I/O contract")
    scratch = [int(np.prod(tensor.shape)) for tensor in graph.tensors
               if tensor.name in (b"scratch", b"scratch_fast") and tensor.type == schema.TensorType.UINT8]
    if not scratch or sum(scratch) + (96 * 511 * 2) + count >= ARENA_BYTES:
        raise ValueError("Grove scratch and model I/O exceed the firmware tensor arena")
    return packed, sum(scratch), stdout


def verify_scores(parent, custom, indices):
    def engine(data):
        net = Interpreter(model_content=data, num_threads=2,
                          experimental_op_resolver_type=OpResolverType.BUILTIN_WITHOUT_DEFAULT_DELEGATES)
        net.allocate_tensors()
        return net

    def scores(net, sample):
        net.set_tensor(net.get_input_details()[0]["index"], sample)
        net.invoke()
        return net.get_tensor(net.get_output_details()[0]["index"])

    original, reduced = engine(parent), engine(custom)
    shape = original.get_input_details()[0]["shape"]
    random = np.random.default_rng(20261125)
    for sample in (np.zeros(shape, np.int8), random.integers(-128, 128, shape, dtype=np.int8),
                   random.integers(-40, 41, shape, dtype=np.int8)):
        expected = scores(original, sample)[:, indices]
        if not np.array_equal(scores(reduced, sample), expected):
            raise ValueError("Custom portable model changed selected int8 scores")


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def prepare(species, output, board):
    names = load_labels(read_pinned(WORLDWIDE / "labels.txt"))
    chosen = resolve_species(species.read_text(encoding="utf-8-sig").splitlines(), names)
    vocal = names.index(HUMAN_VOCAL)
    guard_only = vocal not in chosen
    indices = sorted({vocal, *chosen})
    labels = ("\n".join(names[index] for index in indices) + "\n").encode("utf-8")
    count = len(indices)
    if output.exists():
        raise ValueError(f"Output folder already exists: {output}. Choose a new folder.")
    source = read_pinned(WORLDWIDE / "model_int8.tflite")
    parent = read_model(source)
    portable = read_model(source)
    replace_head(portable, parent, np.array(indices), len(names), len(names))
    reduced = serialize(portable)
    tensors = io_contract(reduced, count)
    verify_scores(source, reduced, indices)
    shared = {"classes": count, "labels_sha256": sha256(labels), "vocal_guard_only": guard_only,
              "tensors": tensors}

    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".birdnet-species-", dir=output.parent) as temp:
        staging = Path(temp) / "model"
        staging.mkdir()
        path = staging / "model_int8.tflite"
        path.write_bytes(reduced)
        (staging / "labels.txt").write_bytes(labels)
        if board in ("both", "grove"):
            work = Path(temp) / "vela"
            work.mkdir()
            packed, scratch, log = compile_grove(work, path, count, tensors)
            (staging / "grove-ethosu.tflite").write_bytes(packed)
            (staging / "vela.log").write_text(log, encoding="utf-8")
            write_json(staging / "grove-contract.json",
                       dict(shared, model_sha256=sha256(packed), npu_scratch_bytes=scratch))
        if board in ("both", "esp32"):
            template = read_pinned(DEFAULT_MODELS / "esp32-packed.tflite")
            template_count = len(load_labels(read_pinned(DEFAULT_MODELS / "labels.txt")))
            packed = esp32_model(template, parent, np.array(indices), len(names), template_count)
            if io_contract(packed, count) != tensors:
                raise ValueError("XIAO packing changed the classifier I/O contract")
            (staging / "esp32-packed.tflite").write_bytes(packed)
            write_json(staging / "esp32-contract.json", dict(shared, model_sha256=sha256(packed)))
        write_json(staging / "species-selection.json",
                   dict(shared, parent_sha256=sha256(source), portable_sha256=sha256(reduced),
                        selected=[names[index] for index in chosen], parent_indices=indices,
                        parent_score_checks=3))
        staging.rename(output)
    guard = " plus the hidden human-vocal recording guard" if guard_only else ""
    noun = "class" if len(chosen) == 1 else "classes"
    print(f"{len(chosen)} selected {noun}{guard}. Wrote {output}.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("species", type=Path, help="Text file with one species name or full label per line")
    parser.add_argument("--output", type=Path, required=True, help="New folder for the model and its ordered labels")
    parser.add_argument("--board", choices=("both", "grove", "esp32"), default="both")
    args = parser.parse_args()
    try:
        prepare(args.species, args.output.resolve(), args.board)
    except subprocess.CalledProcessError as error:
        parser.exit(1, f"Vela compilation failed:\n{error.stdout or error}\n")
    except importlib.metadata.PackageNotFoundError:
        parser.exit(1, "Install the custom model requirements for Grove compilation.\n")
    except (KeyError, OSError, RuntimeError, ValueError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
