"""Local, revision-scoped memory of confirmed button presses, in each floor's map frame."""
import json
import math
from pathlib import Path


class PressMemory:
    def __init__(self, path, revision):
        self.path, self.revision = Path(path), str(revision)
        self.entries = {}
        try:
            data = json.loads(self.path.read_text())
            if data.get('version') == 1 and data.get('revision') == self.revision:
                for key, entry in data.get('entries', {}).items():
                    if self.valid(entry) and key == self.key(entry['floor'], entry['kind'], entry['label']):
                        self.entries[key] = entry
        except (OSError, ValueError, TypeError, AttributeError):
            pass  # no usable memory: observe the button afresh

    @staticmethod
    def key(floor, kind, label):
        return f'{floor}/{kind}/{label}'

    @staticmethod
    def valid(entry):
        try:
            return (entry['floor'] in ('F1', 'F2') and entry['kind'] in ('hall', 'car')
                    and isinstance(entry['label'], str) and bool(entry['label'])
                    and entry['frame'] == 'map' and entry['source'] == 'arm_camera'
                    and all(len(entry[k]) == 3 and all(math.isfinite(v) for v in entry[k])
                            for k in ('button_xyz', 'press_pose')))
        except (KeyError, TypeError, ValueError):
            return False

    def lookup(self, floor, kind, label):
        return self.entries.get(self.key(floor, kind, str(label)))

    def commit(self, entry):
        if not self.valid(entry):
            raise ValueError('invalid confirmed button memory')
        self.entries[self.key(entry['floor'], entry['kind'], entry['label'])] = entry
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps({'version': 1, 'revision': self.revision,
                                        'entries': self.entries}, ensure_ascii=False, indent=2)+'\n')
        temporary.replace(self.path)
