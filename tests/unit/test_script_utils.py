# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for script_utils module."""

from absl.testing import absltest

from src.utils import workpiece_pose
from src.utils.script_utils import load_python_script
from src.utils.workpiece_pose import (
  update_detected_workpiece_pose,
)


class ScriptUtilsTest(absltest.TestCase):
  def test_load_python_script_from_function(self):
    code = load_python_script(update_detected_workpiece_pose)
    self.assertIsInstance(code, str)
    self.assertIn("def update_detected_workpiece_pose", code)
    self.assertIn("update_detected_workpiece_pose(context, params)", code)
    self.assertIn("world = context.object_world", code)

  def test_load_python_script_from_module(self):
    code = load_python_script(
      workpiece_pose,
      function_name="update_detected_workpiece_pose",
    )
    self.assertIsInstance(code, str)
    self.assertIn("def update_detected_workpiece_pose", code)
    self.assertIn("update_detected_workpiece_pose(context, params)", code)

  def test_load_python_script_from_module_string(self):
    code = load_python_script(
      "src.utils.workpiece_pose",
      function_name="update_detected_workpiece_pose",
    )
    self.assertIsInstance(code, str)
    self.assertIn("def update_detected_workpiece_pose", code)
    self.assertIn("update_detected_workpiece_pose(context, params)", code)

  def test_load_python_script_without_call(self):
    code = load_python_script(
      workpiece_pose,
      call_args=None,
    )
    self.assertIsInstance(code, str)
    self.assertIn("def update_detected_workpiece_pose", code)
    self.assertNotIn("update_detected_workpiece_pose(context, params)", code)


if __name__ == "__main__":
  absltest.main()
