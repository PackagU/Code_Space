#!/usr/bin/env python3
"""Review-first occupancy-map cleanup for F1/F2/F3 without overwriting input.

``analyze`` only creates an immutable raw copy, candidate JSON, and overlay.
``apply`` requires explicit candidate IDs and creates a new clean_v1 map pair.
"""

import argparse
import hashlib
import json
import math
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_map(yaml_path):
    yaml_path = Path(yaml_path).resolve()
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    required = ("image", "resolution", "origin", "occupied_thresh", "free_thresh", "negate")
    missing = [key for key in required if key not in metadata]
    if missing:
        raise ValueError(f"map YAML missing: {missing}")
    if metadata.get("mode", "trinary") != "trinary":
        raise ValueError("review cleanup currently supports mode=trinary only")
    image_path = (yaml_path.parent / str(metadata["image"])).resolve()
    pixels = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if pixels is None:
        raise ValueError(f"cannot read occupancy image: {image_path}")
    if pixels.ndim != 2:
        raise ValueError("occupancy image must be single-channel")
    resolution = float(metadata["resolution"])
    origin = tuple(float(value) for value in metadata["origin"])
    if resolution <= 0 or len(origin) != 3 or not all(math.isfinite(v) for v in origin):
        raise ValueError("invalid resolution/origin")
    occupied_thresh = float(metadata["occupied_thresh"])
    free_thresh = float(metadata["free_thresh"])
    if not 0.0 <= free_thresh < occupied_thresh <= 1.0:
        raise ValueError("thresholds must satisfy 0 <= free < occupied <= 1")
    negate = int(metadata["negate"])
    if negate not in (0, 1):
        raise ValueError("negate must be 0 or 1")
    probability = pixels.astype(np.float64) / 255.0
    if negate == 0:
        probability = 1.0 - probability
    occupied = probability > occupied_thresh
    free = probability < free_thresh
    unknown = ~(occupied | free)
    return yaml_path, image_path, metadata, pixels, occupied, free, unknown


def pixel_to_world(row, col, height, resolution, origin):
    cell_x = (col + 0.5) * resolution
    cell_y = (height - 1 - row + 0.5) * resolution
    cos_yaw, sin_yaw = math.cos(origin[2]), math.sin(origin[2])
    return (
        origin[0] + cos_yaw * cell_x - sin_yaw * cell_y,
        origin[1] + sin_yaw * cell_x + cos_yaw * cell_y,
    )


def find_candidates(occupied, free, resolution, origin, max_cells=6, min_free_ring=0.9):
    if max_cells < 1 or not 0.0 <= min_free_ring <= 1.0:
        raise ValueError("invalid candidate thresholds")
    count, labels, stats, _centroids = cv2.connectedComponentsWithStats(
        occupied.astype(np.uint8), connectivity=8
    )
    kernel = np.ones((3, 3), dtype=np.uint8)
    height, width = occupied.shape
    candidates = []
    for label in range(1, count):
        cells = int(stats[label, cv2.CC_STAT_AREA])
        if cells > max_cells:
            continue
        component = labels == label
        ring = cv2.dilate(component.astype(np.uint8), kernel, iterations=1).astype(bool)
        ring &= ~component
        ring_count = int(np.count_nonzero(ring))
        if ring_count == 0:
            continue
        free_ratio = float(np.count_nonzero(free & ring)) / ring_count
        if free_ratio < min_free_ring:
            continue
        rows, cols = np.nonzero(component)
        min_col = int(cols.min())
        max_col = int(cols.max())
        min_row = int(rows.min())
        max_row = int(rows.max())
        center_row = float(rows.mean())
        center_col = float(cols.mean())
        world = pixel_to_world(center_row, center_col, height, resolution, origin)
        candidates.append({
            "id": len(candidates) + 1,
            "cells": cells,
            "free_ring_ratio": round(free_ratio, 6),
            "bbox_pixel_xyxy": [min_col, min_row, max_col, max_row],
            "center_pixel_xy": [round(center_col, 3), round(center_row, 3)],
            "center_cell_xy": [round(center_col, 3), round(height - 1 - center_row, 3)],
            "center_world_xy": [round(world[0], 6), round(world[1], 6)],
            "pixels_rc": [[int(row), int(col)] for row, col in zip(rows, cols)],
        })
    return candidates


def ensure_absent(*paths):
    existing = [str(path) for path in paths if Path(path).exists()]
    if existing:
        raise FileExistsError(f"refusing to overwrite existing outputs: {existing}")


def write_yaml(metadata, image_name, target):
    output = dict(metadata)
    output["image"] = image_name
    Path(target).write_text(
        yaml.safe_dump(output, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
        newline="\n",
    )


def analyze(yaml_path, output_dir, max_cells=6, min_free_ring=0.9):
    yaml_path, image_path, metadata, pixels, occupied, free, unknown = load_map(yaml_path)
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = yaml_path.stem
    raw_image = output_dir / f"{stem}_raw{image_path.suffix.lower()}"
    raw_yaml = output_dir / f"{stem}_raw.yaml"
    overlay_path = output_dir / f"{stem}_candidates.png"
    manifest_path = output_dir / f"{stem}_review.json"
    ensure_absent(raw_image, raw_yaml, overlay_path, manifest_path)

    input_yaml_hash = sha256(yaml_path)
    input_image_hash = sha256(image_path)
    shutil.copy2(image_path, raw_image)
    write_yaml(metadata, raw_image.name, raw_yaml)

    candidates = find_candidates(
        occupied, free, float(metadata["resolution"]),
        tuple(float(v) for v in metadata["origin"]), max_cells, min_free_ring,
    )
    overlay = cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)
    for candidate in candidates:
        x0, y0, x1, y1 = candidate["bbox_pixel_xyxy"]
        cv2.rectangle(overlay, (x0 - 2, y0 - 2), (x1 + 2, y1 + 2), (0, 0, 255), 1)
        cv2.putText(
            overlay, str(candidate["id"]), (max(0, x0 - 1), max(9, y0 - 3)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1, cv2.LINE_AA,
        )
    if not cv2.imwrite(str(overlay_path), overlay):
        raise IOError(f"failed to write overlay: {overlay_path}")

    manifest = {
        "status": "review_pending; no candidate removed",
        "input_yaml": str(yaml_path),
        "input_image": str(image_path),
        "input_yaml_sha256": input_yaml_hash,
        "input_image_sha256": input_image_hash,
        "raw_yaml": str(raw_yaml),
        "raw_image": str(raw_image),
        "raw_image_sha256": sha256(raw_image),
        "overlay": str(overlay_path),
        "shape_hw": [int(pixels.shape[0]), int(pixels.shape[1])],
        "resolution": float(metadata["resolution"]),
        "origin": [float(v) for v in metadata["origin"]],
        "mode": metadata.get("mode", "trinary"),
        "negate": int(metadata["negate"]),
        "occupied_thresh": float(metadata["occupied_thresh"]),
        "free_thresh": float(metadata["free_thresh"]),
        "rule": {"max_component_cells": max_cells, "min_free_ring_ratio": min_free_ring},
        "counts": {
            "occupied": int(np.count_nonzero(occupied)),
            "free": int(np.count_nonzero(free)),
            "unknown": int(np.count_nonzero(unknown)),
        },
        "candidates": candidates,
    }
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return manifest_path, manifest


def apply_review(manifest_path, approved_ids, output_stem=None):
    manifest_path = Path(manifest_path).resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not approved_ids:
        raise ValueError("at least one explicitly approved candidate ID is required")
    yaml_path = Path(manifest["input_yaml"])
    image_path = Path(manifest["input_image"])
    if sha256(yaml_path) != manifest["input_yaml_sha256"]:
        raise ValueError("input YAML changed after review")
    if sha256(image_path) != manifest["input_image_sha256"]:
        raise ValueError("input image changed after review")

    yaml_path, image_path, metadata, pixels, occupied, _free, unknown = load_map(yaml_path)
    by_id = {int(item["id"]): item for item in manifest["candidates"]}
    unknown_ids = sorted(set(approved_ids) - set(by_id))
    if unknown_ids:
        raise ValueError(f"unknown candidate IDs: {unknown_ids}")
    approved = [by_id[item_id] for item_id in sorted(set(approved_ids))]

    clean = pixels.copy()
    probability = pixels.astype(np.float64) / 255.0
    if int(metadata["negate"]) == 0:
        probability = 1.0 - probability
    free_mask = probability < float(metadata["free_thresh"])
    free_values = pixels[free_mask]
    if free_values.size == 0:
        raise ValueError("map has no free pixels to use as replacement")
    replacement = int(np.bincount(free_values, minlength=256).argmax())
    removed = []
    for candidate in approved:
        for row, col in candidate["pixels_rc"]:
            if not occupied[row, col]:
                raise ValueError(f"candidate {candidate['id']} is no longer occupied")
            clean[row, col] = replacement
            removed.append((row, col))

    output_stem = output_stem or f"{yaml_path.stem}_clean_v1"
    if Path(output_stem).name != output_stem:
        raise ValueError("output_stem must be a filename stem, not a path")
    out_image = manifest_path.parent / f"{output_stem}{image_path.suffix.lower()}"
    out_yaml = manifest_path.parent / f"{output_stem}.yaml"
    out_report = manifest_path.parent / f"{output_stem}_report.json"
    ensure_absent(out_image, out_yaml, out_report)
    if not cv2.imwrite(str(out_image), clean):
        raise IOError(f"failed to write cleaned image: {out_image}")
    write_yaml(metadata, out_image.name, out_yaml)

    _, _, out_metadata, reread, out_occupied, _out_free, out_unknown = load_map(out_yaml)
    if reread.shape != pixels.shape:
        raise AssertionError("shape changed")
    for key in ("resolution", "origin", "mode", "negate", "occupied_thresh", "free_thresh"):
        if out_metadata.get(key, "trinary" if key == "mode" else None) != metadata.get(
            key, "trinary" if key == "mode" else None
        ):
            raise AssertionError(f"map metadata changed: {key}")
    if not np.array_equal(unknown, out_unknown):
        raise AssertionError("unknown-cell classification changed")
    changed = pixels != reread
    expected = np.zeros_like(changed)
    for row, col in removed:
        expected[row, col] = True
    if not np.array_equal(changed, expected):
        raise AssertionError("pixels outside approved candidates changed")

    report = {
        "status": "clean candidate output; not activated",
        "source_manifest": str(manifest_path),
        "approved_candidate_ids": [item["id"] for item in approved],
        "removed_cells": len(removed),
        "replacement_free_pixel": replacement,
        "occupied_before": int(np.count_nonzero(occupied)),
        "occupied_after": int(np.count_nonzero(out_occupied)),
        "unknown_before": int(np.count_nonzero(unknown)),
        "unknown_after": int(np.count_nonzero(out_unknown)),
        "output_image": str(out_image),
        "output_yaml": str(out_yaml),
        "output_image_sha256": sha256(out_image),
        "output_yaml_sha256": sha256(out_yaml),
        "activation": "requires user review + map contract + map_server reload test",
    }
    out_report.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return out_report, report


def parse_ids(value):
    try:
        ids = [int(item.strip()) for item in value.split(",") if item.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("IDs must be comma-separated integers") from exc
    if not ids or any(item < 1 for item in ids):
        raise argparse.ArgumentTypeError("at least one positive candidate ID is required")
    return ids


def main():
    parser = argparse.ArgumentParser(description="Review-first ROS occupancy map cleanup")
    sub = parser.add_subparsers(dest="command", required=True)
    analyze_parser = sub.add_parser("analyze")
    analyze_parser.add_argument("map_yaml")
    analyze_parser.add_argument("output_dir")
    analyze_parser.add_argument("--max-cells", type=int, default=6)
    analyze_parser.add_argument("--min-free-ring", type=float, default=0.9)
    apply_parser = sub.add_parser("apply")
    apply_parser.add_argument("manifest")
    apply_parser.add_argument("--approve", required=True, type=parse_ids)
    apply_parser.add_argument("--output-stem")
    args = parser.parse_args()

    if args.command == "analyze":
        path, result = analyze(
            args.map_yaml, args.output_dir, args.max_cells, args.min_free_ring
        )
        print(json.dumps({
            "manifest": str(path),
            "candidate_count": len(result["candidates"]),
            "status": result["status"],
        }, ensure_ascii=False))
    else:
        path, result = apply_review(args.manifest, args.approve, args.output_stem)
        print(json.dumps({"report": str(path), **result}, ensure_ascii=False))


if __name__ == "__main__":
    main()

