#!/usr/bin/env python3
"""Static safety/compatibility contract for the unflashed OpenCR sketch."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "src/drive_pkg/firmware/opencr/opencr_drive_bridge.ino"


def main():
    source = FIRMWARE.read_text(encoding="utf-8")
    for needle in (
        "DXL_SERIAL Serial3", "DXL_DIR_PIN = 84", "LEFT_ID = 1", "RIGHT_ID = 2",
        "DXL_BAUDRATE = 1000000", "HOST_BAUDRATE = 115200",
        'sscanf(line, "V %f %f %c"', "COMMAND_TIMEOUT_MS = 500",
        "stopWheels();", 'HOST_SERIAL.print("F ")', "getPresentVelocity",
        "MAX_ABS_RPM = 30.0", "HELLO opencr 0.2-minimal",
    ):
        assert needle in source, f"firmware missing {needle!r}"
    for forbidden in ("case 'w'", "case 'a'", "case 's'", "case 'd'"):
        assert forbidden not in source, f"manual command path must be absent: {forbidden}"
    print("OpenCR firmware static contract passed; compile/upload remains unverified")


if __name__ == "__main__":
    main()
