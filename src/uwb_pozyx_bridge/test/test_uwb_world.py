"""Unit tests for the rendering of the Gazebo world.

The world file of the package holds the anchor positions twice: in the
``<anchor>`` elements of the UWB plugin and in the ``<include>`` poses of
the visible anchor models.  Both are rewritten from the layout, so that
the simulation and the ESP32 tag describe the same rectangle.
"""

import re

import pytest

from uwb_pozyx_bridge.uwb_layout import AnchorLayout
from uwb_pozyx_bridge.uwb_world import render_world


_TEMPLATE = """<sdf version="1.9">
  <world name="uwb_runway">
    <plugin filename="uwb_gz_plugin" name="uwb_gz_plugin::systems::UwbSystem">
      <anchor name="uwb_anchor0"><pos>0 0 2.0</pos></anchor>
      <anchor name="uwb_anchor1"><pos>0 10 2.0</pos></anchor>
      <anchor name="uwb_anchor2"><pos>10 10 2.0</pos></anchor>
      <anchor name="uwb_anchor3"><pos>10 0 2.0</pos></anchor>
    </plugin>
    <include>
      <uri>model://uwb_anchor</uri>
      <name>uwb_anchor0</name>
      <pose>0 0 2.0 0 0 0</pose>
    </include>
    <include>
      <uri>model://uwb_anchor</uri>
      <name>uwb_anchor1</name>
      <pose>0 10 2.0 0 0 0</pose>
    </include>
    <include>
      <uri>model://uwb_anchor</uri>
      <name>uwb_anchor2</name>
      <pose>10 10 2.0 0 0 0</pose>
    </include>
    <include>
      <uri>model://uwb_anchor</uri>
      <name>uwb_anchor3</name>
      <pose>10 0 2.0 0 0 0</pose>
    </include>
  </world>
</sdf>
"""


def _anchor_positions(rendered: str) -> list[tuple[float, float, float]]:
    """Read the plugin's anchor positions back from the rendered world."""
    positions = []
    for match in re.finditer(
        r'<anchor\s+name="uwb_anchor\d+"\s*>\s*<pos>([^<]*)</pos>', rendered
    ):
        east, north, up = (float(v) for v in match.group(1).split())
        positions.append((east, north, up))
    return positions


def _include_positions(rendered: str) -> list[tuple[float, float, float]]:
    """Read the anchor model poses back from the rendered world."""
    positions = []
    for match in re.finditer(
        r"<uri>model://uwb_anchor</uri>\s*"
        r"<name>\s*uwb_anchor\d+\s*</name>\s*"
        r"<pose>([^<]*)</pose>",
        rendered,
    ):
        east, north, up = (float(v) for v in match.group(1).split()[:3])
        positions.append((east, north, up))
    return positions


def test_the_default_layout_leaves_the_world_unchanged():
    """A 10 m x 10 m layout renders the positions of the template."""
    layout = AnchorLayout(size_x_m=10.0, size_y_m=10.0, plane_height_m=2.0)
    rendered = render_world(_TEMPLATE, layout)

    assert _anchor_positions(rendered) == [
        (0.0, 0.0, 2.0),
        (0.0, 10.0, 2.0),
        (10.0, 10.0, 2.0),
        (10.0, 0.0, 2.0),
    ]
    assert _include_positions(rendered) == _anchor_positions(rendered)


def test_a_rectangular_layout_is_rendered_in_both_places():
    """The plugin and the visible models get the same rectangle."""
    layout = AnchorLayout(size_x_m=14.0, size_y_m=7.0, plane_height_m=2.0)
    rendered = render_world(_TEMPLATE, layout)

    expected = [
        (0.0, 0.0, 2.0),
        (0.0, 14.0, 2.0),
        (7.0, 14.0, 2.0),
        (7.0, 0.0, 2.0),
    ]
    assert _anchor_positions(rendered) == expected
    assert _include_positions(rendered) == expected


def test_the_rotation_is_applied_to_the_world_positions():
    """The world is rendered with the anchors where they really are."""
    layout = AnchorLayout(
        size_x_m=10.0, size_y_m=10.0, alpha_deg=90.0, plane_height_m=2.0
    )
    rendered = render_world(_TEMPLATE, layout)

    positions = _anchor_positions(rendered)
    # a0 -> a1 points East, a0 -> a3 points South.
    assert positions[1][0] == pytest.approx(10.0, abs=1e-4)
    assert positions[1][1] == pytest.approx(0.0, abs=1e-4)
    assert positions[3][0] == pytest.approx(0.0, abs=1e-4)
    assert positions[3][1] == pytest.approx(-10.0, abs=1e-4)


def test_the_rest_of_the_world_is_preserved():
    """Only the anchor positions are rewritten."""
    layout = AnchorLayout(size_x_m=14.0, size_y_m=7.0)
    rendered = render_world(_TEMPLATE, layout)

    assert '<world name="uwb_runway">' in rendered
    assert "model://uwb_anchor" in rendered
