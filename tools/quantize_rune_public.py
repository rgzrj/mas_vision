"""Experimental PTQ from a public video; does not modify live configuration."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import cv2
import nncf
import numpy as np
import openvino as ov


def tensor_from_file(path):
    image = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape != (480, 640, 3):
        raise ValueError(f"Invalid calibration image: {path}")
    return np.ascontiguousarray(image[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255.


def extract(video, output, samples):
    capture = cv2.VideoCapture(str(video))
    count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    validation_count = 60
    if not capture.isOpened() or count < samples + validation_count:
        raise ValueError("Video must contain at least samples+60 readable frames")
    calibration = []
    validation = []
    indices = np.linspace(0, count - 1, samples + validation_count, dtype=int)
    validation_positions = set(np.linspace(0, len(indices) - 1, validation_count, dtype=int))
    for position, index in enumerate(indices):
        capture.set(cv2.CAP_PROP_POS_FRAMES, int(index))
        ok, image = capture.read()
        if not ok:
            raise ValueError(f"Cannot read source frame {index}")
        height, width = image.shape[:2]
        scale = min(640 / width, 480 / height)
        w, h = round(width * scale), round(height * scale)
        x, y = (640 - w) // 2, (480 - h) // 2
        letterbox = np.full((480, 640, 3), 114, np.uint8)
        letterbox[y:y + h, x:x + w] = cv2.resize(image, (w, h), interpolation=cv2.INTER_LINEAR)
        split = "validation" if position in validation_positions else "calibration"
        folder = output / split
        folder.mkdir(exist_ok=True)
        path = folder / f"frame_{index:06d}.png"
        success, encoded = cv2.imencode(".png", letterbox)
        if not success:
            raise ValueError("PNG encoding failed")
        encoded.tofile(path)
        record = {"path": str(path.relative_to(output)), "frame": int(index),
                  "width": width, "height": height, "scale": scale, "pad": [x, y]}
        (validation if split == "validation" else calibration).append(record)
    capture.release()
    return calibration, validation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=300)
    args = parser.parse_args()
    if not args.model.is_file() or not args.video.is_file() or args.samples < 1:
        parser.error("Model/video must exist and samples must be positive")
    if args.output.exists():
        parser.error("Use a new output directory to preserve previous results")
    args.output.mkdir(parents=True)
    cv2.setNumThreads(4)
    calibration, validation = extract(args.video, args.output, args.samples)
    manifest = {
        "source_model": str(args.model.resolve()),
        "model_sha256": hashlib.sha256(args.model.read_bytes()).hexdigest(),
        "source_video": str(args.video.resolve()),
        "video_sha256": hashlib.sha256(args.video.read_bytes()).hexdigest(),
        "preprocess": "BGR letterbox 640x480, pad 114, RGB FP32 NCHW /255",
        "input": [1, 3, 480, 640], "output": [1, 18, 6300],
        "openvino": ov.__version__, "nncf": nncf.__version__, "opencv": cv2.__version__,
        "calibration": calibration, "validation": validation,
        "limitation": "Unlabelled, non-overlapping frames from the SAME public video; "
                      "not independent real-camera accuracy validation.",
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    core = ov.Core()
    model = core.read_model(str(args.model))
    if list(model.input().shape) != [1, 3, 480, 640] or list(model.output().shape) != [1, 18, 6300]:
        raise ValueError("Model does not follow the three-class/five-point protocol")
    paths = [args.output / row["path"] for row in calibration]
    dataset = nncf.Dataset(paths, tensor_from_file)
    print(f"PTQ: {len(calibration)} calibration, {len(validation)} held-out frames", flush=True)
    quantized = nncf.quantize(model, dataset, subset_size=len(paths),
                             preset=nncf.QuantizationPreset.MIXED, target_device=nncf.TargetDevice.CPU)
    target = args.output / "rune_szu_public_int8.xml"
    ov.save_model(quantized, str(target), compress_to_fp16=False)
    reloaded = core.read_model(str(target))
    if (reloaded.input().element_type != ov.Type.f32 or reloaded.output().element_type != ov.Type.f32 or
            list(reloaded.input().shape) != [1, 3, 480, 640] or list(reloaded.output().shape) != [1, 18, 6300]):
        raise ValueError("Export changed the model I/O protocol")
    print("Exported:", target, flush=True)
    print("FP32 I/O preserved. Requires drift, runtime-precision and target-device performance checks.")


if __name__ == "__main__":
    if sys.platform == "win32":
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    main()
