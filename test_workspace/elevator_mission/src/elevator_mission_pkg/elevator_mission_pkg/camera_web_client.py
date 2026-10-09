"""The mission reads the same recognized button and taught pose shown in the website."""
import json
import math
import time
from urllib.request import Request, urlopen


class CameraWebClient:
    def __init__(self, url):
        self.url = url.rstrip('/')

    def request(self, path, data=None):
        request = Request(self.url + path, data=None if data is None else json.dumps(data).encode(),
                          headers={} if data is None else {'Content-Type': 'application/json'})
        with urlopen(request, timeout=.3) as response:
            return json.loads(response.read(65536))

    def select(self, label, target_floor):
        result = self.request('/api/target', {'label': label, 'floor_target': target_floor})
        if not isinstance(result, dict) or result.get('ok') is not True:
            raise ValueError('camera website rejected button target')

    def pose(self, label, since):
        state = self.request('/api/state')
        if not isinstance(state, dict):
            return None
        stamp, ready = state.get('captured_at'), state.get('ready')
        if (state.get('error') or state.get('target') != label or not state.get('token')
                or type(stamp) not in (int, float) or not math.isfinite(stamp)
                or not since <= stamp <= time.time() or time.time() - stamp > .8
                or not isinstance(ready, dict) or ready.get('label') != label):
            return None
        pose = ready.get('press')
        if not isinstance(pose, dict) or set(pose) != {'000', '001', '002', '003'}:
            return None
        if any(type(value) is not int or not 900 <= value <= 2600 for value in pose.values()):
            return None
        return dict(pose)
