#!/usr/bin/env python3
"""Failed GUI teardown stops before pausing, preserves scene, and falls back to cleanup if pause fails."""
from tempfile import TemporaryDirectory
from pathlib import Path
from unittest.mock import Mock
from sim_stack import Stack

for preserve, ack in [(True,True),(True,False),(False,True)]:
 with TemporaryDirectory() as directory:
  stack=object.__new__(Stack)
  stack.dir=Path(directory)
  stack.result={}
  stack.node=Mock(samples=[],amcl_hist=[],warnings=[])
  stack.recorder=None
  stack.attached=False
  stack.processes=Mock()
  sequence=[]
  stack.stop=lambda: sequence.append('stop')
  def pause(service):
   assert service=='/pause_physics'
   sequence.append('pause')
   return ack
  stack.node.call_empty.side_effect=pause
  stack.finish(preserve_scene=preserve)
  assert sequence==(['stop','pause'] if preserve else ['stop'])
  retained=preserve and ack
  assert (stack.dir/'SCENE_PRESERVED').exists()==retained
  assert stack.processes.cleanup.call_count==(0 if retained else 1)
  stack.node.destroy_node.assert_called_once()
print('failure scene: stop before pause / preserve / pause-failure cleanup / normal cleanup PASS')
