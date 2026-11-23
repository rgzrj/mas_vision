"""Print OpenVINO model I/O and optionally open the original model in Netron."""
import argparse
from datetime import datetime
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path, help="ONNX, or OpenVINO XML with its BIN")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--serve", action="store_true", help="Open a local Netron viewer; Ctrl+C stops it")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    import openvino as ov

    model_path = args.model.resolve(strict=True)
    model = ov.Core().read_model(str(model_path))
    def describe(port):
        return {"names": sorted(port.get_names()), "type": port.get_element_type().get_type_name(),
                "shape": str(port.get_partial_shape())}

    info = {"model": str(model_path), "openvino": ov.__version__,
            "inputs": [describe(p) for p in model.inputs], "outputs": [describe(p) for p in model.outputs]}
    output = args.output or Path(__file__).resolve().parents[1] / "output/profiling" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True, exist_ok=True)
    content = json.dumps(info, ensure_ascii=False, indent=2)
    (output / "model_info.json").write_text(content + "\n", encoding="utf-8")
    print(content)
    print(f"Report: {output.resolve()}")
    if args.serve:
        import netron
        # The model is served locally, without uploading it to a website.
        netron.start(str(model_path), address=("127.0.0.1", args.port), browse=True)
        netron.wait()


if __name__ == "__main__":
    main()
