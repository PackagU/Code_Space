"""Strict saved-map checks used before starting physical Nav2."""

import math
from pathlib import Path
import sys

import yaml


class MapContractError(ValueError):
    pass


def _finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MapContractError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise MapContractError(f"{name} must be finite")
    return result


def _pgm_dimensions(path):
    with path.open("rb") as stream:
        magic = stream.readline().strip()
        if magic not in (b"P2", b"P5"):
            raise MapContractError(f"map image is not PGM P2/P5: {path}")
        tokens = []
        while len(tokens) < 3:
            line = stream.readline()
            if not line:
                break
            line = line.split(b"#", 1)[0]
            tokens.extend(line.split())
    if len(tokens) < 3:
        raise MapContractError(f"incomplete PGM header: {path}")
    try:
        width, height, max_value = (int(tokens[0]), int(tokens[1]), int(tokens[2]))
    except ValueError as exc:
        raise MapContractError(f"invalid PGM header: {path}") from exc
    if width <= 0 or height <= 0 or not 0 < max_value <= 65535:
        raise MapContractError(f"invalid PGM dimensions/range: {path}")
    return width, height


def validate_map_yaml(map_yaml):
    yaml_path = Path(map_yaml).expanduser().resolve()
    if not yaml_path.is_file():
        raise MapContractError(f"map yaml not found: {yaml_path}")
    try:
        data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise MapContractError(f"cannot read map yaml: {yaml_path}: {exc}") from exc
    if not isinstance(data, dict):
        raise MapContractError("map yaml root must be a mapping")

    image_value = data.get("image")
    if not isinstance(image_value, str) or not image_value.strip():
        raise MapContractError("map yaml image must be a non-empty path")
    image_path = Path(image_value).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve()
    if not image_path.is_file():
        raise MapContractError(f"map image not found: {image_path}")
    if image_path.suffix.lower() != ".pgm":
        raise MapContractError(f"field map image must be .pgm: {image_path}")

    resolution = _finite_number(data.get("resolution"), "resolution")
    if resolution <= 0.0:
        raise MapContractError("resolution must be positive")
    origin = data.get("origin")
    if not isinstance(origin, list) or len(origin) != 3:
        raise MapContractError("origin must contain [x, y, yaw]")
    origin = [_finite_number(value, f"origin[{index}]") for index, value in enumerate(origin)]
    free_thresh = _finite_number(data.get("free_thresh"), "free_thresh")
    occupied_thresh = _finite_number(data.get("occupied_thresh"), "occupied_thresh")
    if not 0.0 <= free_thresh < occupied_thresh <= 1.0:
        raise MapContractError("thresholds must satisfy 0 <= free < occupied <= 1")
    if data.get("negate") not in (0, 1, False, True):
        raise MapContractError("negate must be 0 or 1")
    width, height = _pgm_dimensions(image_path)
    return {
        "yaml": str(yaml_path),
        "image": str(image_path),
        "resolution": resolution,
        "origin": origin,
        "width": width,
        "height": height,
    }


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) != 1:
        print("usage: python3 -m slam_pkg.map_contract MAP.yaml", file=sys.stderr)
        return 2
    try:
        result = validate_map_yaml(args[0])
    except MapContractError as exc:
        print(f"INVALID: {exc}", file=sys.stderr)
        return 1
    print(
        "VALID: {yaml} image={image} size={width}x{height} resolution={resolution}".format(
            **result
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
