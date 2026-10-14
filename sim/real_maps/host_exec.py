"""Host trampoline: re-run a sim/real_maps script inside the isolated simulation container.

Import this BEFORE any ROS module. On the host it brings the isolated container up
(sim_container.sh up) and re-executes the same script there; inside the container it is a no-op.
"""
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def ensure_container(script, x11=False):
    if os.environ.get('PACKAGU_SIM_ISOLATED') == '1':
        return
    import shlex
    os.environ['PACKAGU_SIM_CMD'] = ' '.join(shlex.quote(a) for a in ['python3', *sys.argv])
    # X11 socket also for --arm-sim: the arm-tip camera renders through the host display (no Xvfb in the image).
    gui = ['--gui'] if x11 or os.environ.get('SIM_GUI') == '1' or '--gui' in sys.argv or '--arm-sim' in sys.argv else []
    subprocess.run(['bash', str(HERE/'sim_container.sh'), 'up', *gui], check=True)
    if os.environ.get('SIM_GUI') == '1' or '--gui' in sys.argv:
        # Host-side window layout once the GUI windows appear (observation only; detached, never blocks the run).
        subprocess.Popen([sys.executable, str(HERE/'arrange_gui.py'), '--container',
                          os.environ.get('PACKAGU_SIM_CONTAINER', 'packagu_sim_e2e'), '--wait', '900'],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    os.execvp('bash', ['bash', str(HERE/'sim_container.sh'), 'exec', 'python3', '-u',
                       f'sim/real_maps/{Path(script).name}', *sys.argv[1:]])
