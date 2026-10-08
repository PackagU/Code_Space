"""Host trampoline: re-run a sim/real_maps script inside the isolated simulation container.

Import this BEFORE any ROS module. On the host it brings the isolated container up
(sim_container.sh up) and re-executes the same script there; inside the container it is a no-op.
"""
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def ensure_container(script):
    if os.environ.get('PACKAGU_SIM_ISOLATED') == '1':
        return
    import shlex
    os.environ['PACKAGU_SIM_CMD'] = ' '.join(shlex.quote(a) for a in ['python3', *sys.argv])
    gui = ['--gui'] if os.environ.get('SIM_GUI') == '1' else []
    subprocess.run(['bash', str(HERE/'sim_container.sh'), 'up', *gui], check=True)
    os.execvp('bash', ['bash', str(HERE/'sim_container.sh'), 'exec', 'python3', '-u',
                       f'sim/real_maps/{Path(script).name}', *sys.argv[1:]])
