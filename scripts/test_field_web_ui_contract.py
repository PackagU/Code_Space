#!/usr/bin/env python3
"""Static security, safety, and UI contract for the field browser console."""

import ast
from html.parser import HTMLParser
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BACKEND = ROOT / "src/slam_pkg/slam_pkg/field_web_ui.py"
HTML = ROOT / "src/slam_pkg/web/index.html"
JS = ROOT / "src/slam_pkg/web/app.js"
CSS = ROOT / "src/slam_pkg/web/styles.css"
START = ROOT / "scripts/start_field_web_ui.sh"


class Parser(HTMLParser):
    pass


def main():
    for path in (BACKEND, HTML, JS, CSS, START):
        assert path.is_file(), path
    backend = BACKEND.read_text(encoding="utf-8")
    ast.parse(backend, filename=str(BACKEND))
    html = HTML.read_text(encoding="utf-8")
    parser = Parser(); parser.feed(html); parser.close()
    js = JS.read_text(encoding="utf-8")
    css = CSS.read_text(encoding="utf-8")
    start = START.read_text(encoding="utf-8")

    for token in (
        "매핑 시작", "저장하고 끝내기", "네비게이션", "mapCanvas",
        'data-drive="forward"', 'data-drive="backward"', "소프트 정지",
    ):
        assert token in html, token
    for endpoint in (
        "/api/status", "/api/map", "/api/maps", "/api/cmd", "/api/stop",
        "/api/mapping/start", "/api/mapping/stop", "/api/navigation/start",
        "/api/navigation/initial-pose", "/api/navigation/goal",
    ):
        assert endpoint in backend or endpoint in js, endpoint
    for safety in (
        'bind_host", "127.0.0.1"', "X-Packagu-Token", "secrets.compare_digest",
        "command_deadman_sec", "self.publish_zero()", "frame-ancestors 'none'",
    ):
        assert safety in backend, safety
    assert 'ssh -N -L ${PORT}:127.0.0.1:${PORT}' in start
    assert "arm" not in start.lower() or "not started" in start.lower()
    assert css.count("{") == css.count("}"), "unbalanced CSS braces"
    print("field web UI static contract passed")


if __name__ == "__main__":
    main()
