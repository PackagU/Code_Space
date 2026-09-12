#!/usr/bin/env python3
"""udev 규칙/설치 스크립트 계약 검사."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
RULES = ROOT / "scripts/udev/99-packagu-devices.rules"
INSTALL = ROOT / "scripts/udev/install_udev_rules.sh"


def require(condition, message):
    if not condition:
        print(f"FAIL: {message}", file=sys.stderr)
        sys.exit(1)


def main():
    require(RULES.exists(), "missing udev rules file")
    src = RULES.read_text(encoding="utf-8")
    for needle in ['SYMLINK+="rplidar"', 'SYMLINK+="opencr"', "motor_nano", "arm_servo"]:
        require(needle in src, f"rules missing: {needle}")
    active = [l for l in src.splitlines() if l.strip() and not l.strip().startswith("#")]
    require(all('SYMLINK+="' in l for l in active), "active rule lines must define SYMLINK")
    require(not any("____" in l for l in active), "placeholder VID/PID must stay commented")

    require(INSTALL.exists(), "missing install script")
    rc = subprocess.run(["bash", "-n", str(INSTALL)], capture_output=True).returncode
    require(rc == 0, "install script syntax error")
    install_src = INSTALL.read_text(encoding="utf-8")
    for needle in ["udevadm control --reload-rules", "udevadm trigger", "/etc/udev/rules.d/"]:
        require(needle in install_src, f"install script missing: {needle}")

    print("udev contract passed")


if __name__ == "__main__":
    main()
