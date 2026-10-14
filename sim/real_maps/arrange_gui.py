#!/usr/bin/env python3
"""Host-side window layout for --gui runs (observation only): Gazebo on top, RViz and the arm camera below,
all in the left half of the work area. Only windows titled "(on <container hostname>)" are touched.

  python3 sim/real_maps/arrange_gui.py [--container packagu_sim_e2e] [--wait 300]
Needs the host's python3-gi with Wnck 3.0 (GNOME); exits quietly when it is unavailable.
"""
import argparse
import subprocess
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--container', default='packagu_sim_e2e')
    parser.add_argument('--wait', type=float, default=300.0, help='seconds to wait for the windows')
    args = parser.parse_args()
    try:
        import gi
        gi.require_version('Wnck', '3.0')
        from gi.repository import GLib, Wnck
    except (ImportError, ValueError):
        return
    tag, both_since = None, None
    end = time.monotonic()+args.wait
    found = {}
    while time.monotonic() < end:
        if tag is None:
            out = subprocess.run(['docker', 'inspect', '-f', '{{.Config.Hostname}}', args.container],
                                 capture_output=True, text=True)
            tag = f'(on {out.stdout.strip()})' if out.returncode == 0 and out.stdout.strip() else None
        if tag:
            screen = Wnck.Screen.get_default()
            pump(GLib)                          # Wnck learns about new windows/titles only through events
            screen.force_update()
            for w in screen.get_windows():
                name = w.get_name() or ''
                if tag not in name:
                    continue
                key = 'gazebo' if name.startswith('Gazebo') else 'rviz' if 'RViz' in name else \
                    'camera' if 'ImageView' in name else None
                if key:
                    found[key] = w
            if len(found) == 3:
                break
            if 'gazebo' in found and 'rviz' in found:
                both_since = both_since or time.monotonic()
                if time.monotonic()-both_since > 40:   # runs without the arm camera
                    break
        time.sleep(2)
    if 'gazebo' not in found:
        return
    screen = Wnck.Screen.get_default()
    workspace = screen.get_active_workspace()
    x0, y0, width, height = 0, 0, screen.get_width(), screen.get_height()
    if workspace is not None:
        # Leave the dock/top bar: start where a maximized window starts.
        gx, gy, _, _ = found['gazebo'].get_geometry()
        x0, y0 = max(0, gx), max(0, gy)
        width, height = screen.get_width()-x0, screen.get_height()-y0
    half, top = width//2, int(height*.58)
    rviz_w = int(half*.58) if 'camera' in found else half
    layout = {'gazebo': (x0, y0, half, top), 'rviz': (x0, y0+top, rviz_w, height-top),
              'camera': (x0+rviz_w, y0+top, half-rviz_w, height-top)}
    mask = (Wnck.WindowMoveResizeMask.X | Wnck.WindowMoveResizeMask.Y | Wnck.WindowMoveResizeMask.WIDTH |
            Wnck.WindowMoveResizeMask.HEIGHT)
    for key, window in found.items():
        if window.is_maximized():
            window.unmaximize()
            pump(GLib, .5)
        window.set_geometry(Wnck.WindowGravity.STATIC, mask, *layout[key])
    pump(GLib, .5)


def pump(glib, seconds=0.0):
    end = time.monotonic()+seconds
    context = glib.MainContext.default()
    while True:
        while context.pending():
            context.iteration(False)
        if time.monotonic() >= end:
            return
        time.sleep(.05)


if __name__ == '__main__':
    main()
