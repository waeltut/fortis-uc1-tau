"""Geometry integration test, run by colcon against the compiled extension."""
import pytest
from _clearance import ClearanceChecker


def robot():
    return '''<robot name="clearance_test">
      <link name="base"/>
      <link name="left_shape"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
      <link name="right_shape"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
      <joint name="left_fixed" type="fixed"><parent link="base"/><child link="left_shape"/></joint>
      <joint name="right_slide" type="prismatic">
        <parent link="base"/><child link="right_shape"/><axis xyz="1 0 0"/>
        <limit lower="0" upper="1" effort="1" velocity="1"/>
      </joint></robot>'''


def test_surface_gap_and_reused_checker():
    checker = ClearanceChecker(robot())
    # Centre separation minus two 10 cm radii gives the surface clearance.
    for separation, expected in ((0.26, True), (0.24, False), (0.19, False), (0.4, True)):
        assert checker.has_clearance({'right_slide': separation}, 0.05) == expected


def test_missing_state_and_invalid_margin():
    checker = ClearanceChecker(robot())
    with pytest.raises(RuntimeError, match='Missing clearance joint state'):
        checker.has_clearance({}, 0.05)
    with pytest.raises(ValueError):
        checker.has_clearance({'right_slide': 0.4}, float('nan'))
