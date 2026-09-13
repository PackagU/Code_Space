#!/usr/bin/env python3
"""Offline review/apply test with a synthetic trinary map."""

import importlib.util
import tempfile
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "scripts" / "map_cleanup_review.py"


def main():
    spec = importlib.util.spec_from_file_location("map_cleanup_review", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with tempfile.TemporaryDirectory() as temp:
        base = Path(temp)
        pixels = np.full((12, 14), 254, dtype=np.uint8)
        pixels[0, :] = 205
        pixels[:, 0] = 205
        pixels[5:8, 6:10] = 0  # real 12-cell structure; over candidate size
        pixels[2, 3] = 0       # candidate 1
        pixels[9, 12] = 0      # candidate 2
        image = base / "f1_test.pgm"
        assert cv2.imwrite(str(image), pixels)
        metadata = {
            "image": image.name,
            "mode": "trinary",
            "resolution": 0.05,
            "origin": [1.0, 2.0, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            # 205 -> occupancy probability 0.196078; 0.196 keeps it unknown.
            "free_thresh": 0.196,
        }
        map_yaml = base / "f1_test.yaml"
        map_yaml.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")
        review_dir = base / "review"
        manifest_path, manifest = module.analyze(map_yaml, review_dir)
        assert len(manifest["candidates"]) == 2
        assert manifest["counts"]["unknown"] == 25
        assert module.sha256(image) == manifest["input_image_sha256"]
        assert manifest["input_image_sha256"] == manifest["raw_image_sha256"]

        report_path, report = module.apply_review(manifest_path, [1])
        assert report["removed_cells"] == 1
        assert report["unknown_before"] == report["unknown_after"] == 25
        clean_yaml = Path(report["output_yaml"])
        _, _, clean_meta, clean, *_ = module.load_map(clean_yaml)
        assert clean_meta["resolution"] == metadata["resolution"]
        first = manifest["candidates"][0]["pixels_rc"][0]
        second = manifest["candidates"][1]["pixels_rc"][0]
        assert clean[first[0], first[1]] == 254
        assert clean[second[0], second[1]] == 0
        assert pixels[5:8, 6:10].sum() == 0
        assert report_path.exists()

        try:
            module.apply_review(manifest_path, [999], "bad")
        except ValueError:
            pass
        else:
            raise AssertionError("unknown candidate ID accepted")
    print("PASS map cleanup review: raw preserved, explicit IDs, unknown/metadata invariant")


if __name__ == "__main__":
    main()
