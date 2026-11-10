"""Generation of the Gazebo world for a given anchor layout.

The UWB world (``uwb_runway.sdf``) carries the anchor positions twice:

* in the ``<anchor>`` elements of the UWB plugin, which is what the ranging
  model is built from,
* in the ``<include>`` poses of the visible anchor models.

Both have to agree with the layout that the solver assumes, so they are
rewritten together from the same :class:`~uwb_pozyx_bridge.uwb_layout.AnchorLayout`
that the odometry node uses.  This keeps the simulation and the ESP32 tag
in sync for any rectangle, not only for the square of the static world file.

The world file of the package stays the reference for everything that is
*not* the layout (the runway, the lights, the sensors): this module only
substitutes the anchor positions.
"""

from __future__ import annotations

import re

from .uwb_layout import AnchorLayout

__all__ = ["render_world", "write_world"]


#: Matches the ``<pos>`` of an ``<anchor>`` element of the UWB plugin.
_ANCHOR_POS_RE = re.compile(
    r'(<anchor\s+name="uwb_anchor\d+"\s*>\s*<pos>)([^<]*)(</pos>)',
    re.IGNORECASE,
)

#: Matches a whole ``<include>`` block of an anchor model.
_ANCHOR_INCLUDE_RE = re.compile(
    r"(<include>\s*"
    r"<uri>model://uwb_anchor</uri>\s*"
    r"<name>\s*(uwb_anchor\d+)\s*</name>\s*"
    r"<pose>)([^<]*)(</pose>)",
    re.IGNORECASE,
)


def _format_pos(east_m: float, north_m: float, up_m: float) -> str:
    """Format a position, as the ``<pos>`` element of the plugin wants it."""
    return f"{east_m:.4f} {north_m:.4f} {up_m:.4f}"


def _format_pose(east_m: float, north_m: float, up_m: float) -> str:
    """Format a Gazebo ENU pose, with the zero rotation of the anchors."""
    return f"{east_m:.4f} {north_m:.4f} {up_m:.4f} 0 0 0"


def render_world(template: str, layout: AnchorLayout) -> str:
    """Render the world file for a layout.

    The anchor positions are taken from the layout and written both into the
    plugin's ``<anchor>`` elements and into the ``<include>`` poses of the
    visible anchor models.

    Args:
        template: The content of the world file template.
        layout: The layout to render.

    Returns:
        The rendered world file.
    """
    world_positions = layout.anchor_world_enu()
    by_name = {
        f"uwb_anchor{index}": position
        for index, position in enumerate(world_positions)
    }

    def _replace_anchor_pos(match: re.Match) -> str:
        name = re.search(r'name="(uwb_anchor\d+)"', match.group(1), re.IGNORECASE)
        if name is None:
            return match.group(0)
        position = by_name.get(name.group(1).lower())
        if position is None:
            return match.group(0)
        return match.group(1) + _format_pos(*position) + match.group(3)

    def _replace_include_pose(match: re.Match) -> str:
        position = by_name.get(match.group(2).lower())
        if position is None:
            return match.group(0)
        return match.group(1) + _format_pose(*position) + match.group(4)

    rendered = _ANCHOR_POS_RE.sub(_replace_anchor_pos, template)
    rendered = _ANCHOR_INCLUDE_RE.sub(_replace_include_pose, rendered)
    return rendered


def write_world(path: str, template: str, layout: AnchorLayout) -> str:
    """Render the world file for a layout and write it to disk.

    Args:
        path: Where to write the world file.
        template: The content of the world file template.
        layout: The layout to render.

    Returns:
        The path of the written file.
    """
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(render_world(template, layout))
    return path
