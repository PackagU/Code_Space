#!/usr/bin/env python3
"""P03 무동작/라이다 전용/DDS 배포 계약의 정적 회귀 검사."""

from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SAFE = ROOT / "docker/compose/docker-compose.jetson.safe.yml"
LIDAR = ROOT / "docker/compose/docker-compose.jetson.lidar.yml"
MAPPING = ROOT / "docker/compose/docker-compose.jetson.mapping.yml"
PREFLIGHT = ROOT / "scripts/jetson_preflight.py"
DDS_PROBE = ROOT / "scripts/dds_contract_probe.py"
DDS_TEST = ROOT / "scripts/test_jetson_dds_contract.sh"
RVIZ = ROOT / "scripts/run_rviz_jetson.sh"
UDP_XML = ROOT / "scripts/fastdds_udp_only.xml"


def require(condition: bool, message: str) -> None:
    if not condition:
        print(f"FAIL: {message}", file=sys.stderr)
        raise SystemExit(1)


def main() -> None:
    for path in (SAFE, LIDAR, MAPPING, PREFLIGHT, DDS_PROBE, DDS_TEST, RVIZ, UDP_XML):
        require(path.is_file(), f"missing {path.relative_to(ROOT)}")

    safe = SAFE.read_text(encoding="utf-8")
    lidar = LIDAR.read_text(encoding="utf-8")
    for target in ("rplidar", "opencr", "arm_servo", "motor_nano"):
        require(f"/dev/null:/dev/{target}" in safe, f"safe overlay exposes {target}")
    require("PACKAGU_DEPLOYMENT_MODE: safe" in safe, "safe mode marker missing")
    require("${RPLIDAR_DEVICE:-/dev/rplidar}:/dev/rplidar" in lidar, "lidar mapping missing")
    for target in ("opencr", "arm_servo", "motor_nano"):
        require(f"/dev/null:/dev/{target}" in lidar, f"lidar overlay exposes {target}")
    require("PACKAGU_DEPLOYMENT_MODE: lidar" in lidar, "lidar mode marker missing")

    mapping = MAPPING.read_text(encoding="utf-8")
    require("${RPLIDAR_DEVICE:-/dev/rplidar}:/dev/rplidar" in mapping, "mapping LiDAR missing")
    require("${OPENCR_DEVICE:-/dev/opencr}:/dev/opencr" in mapping, "mapping OpenCR missing")
    for target in ("arm_servo", "motor_nano"):
        require(f"/dev/null:/dev/{target}" in mapping, f"mapping overlay exposes {target}")
    require("PACKAGU_DEPLOYMENT_MODE: mapping_navigation_base_only" in mapping,
            "mapping mode marker missing")

    preflight = PREFLIGHT.read_text(encoding="utf-8")
    for token in ("IpcMode", "ROS_DOMAIN_ID", "clock_delta_sec", "duplicate node names", "slam_toolbox and amcl"):
        require(token in preflight, f"preflight check missing: {token}")

    rviz = RVIZ.read_text(encoding="utf-8")
    for token in ("packagu.donor_id", "PACKAGU_RVIZ_REQUIRED_TOPIC", "humble-jetson-p02"):
        require(token in rviz, f"RViz guard missing: {token}")

    root = ET.parse(UDP_XML).getroot()
    text = " ".join(item.strip() for item in root.itertext() if item.strip())
    require("UDPv4" in text, "UDP transport missing")
    require("false" in text.lower(), "builtin/SHM disable missing")

    for path in (PREFLIGHT, DDS_PROBE):
        compile(path.read_text(encoding="utf-8"), str(path), "exec")
    for path in (DDS_TEST, RVIZ):
        result = subprocess.run(("bash", "-n", str(path)), capture_output=True, text=True)
        require(result.returncode == 0, f"shell syntax error: {path.relative_to(ROOT)}")

    print("Jetson safe profile contract passed")


if __name__ == "__main__":
    main()
