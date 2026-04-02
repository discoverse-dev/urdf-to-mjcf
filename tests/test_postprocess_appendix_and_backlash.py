"""Tests for appendix and backlash postprocess helpers."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from robot2mjcf.postprocess.add_appendix import (
    add_appendix,
    add_filtered_contact_constraints,
    add_filtered_equality_constraints,
    add_filtered_sensor_constraints,
    find_all_bodies,
    find_all_joints,
    find_all_sites,
    merge_elements,
    validate_contact_constraints,
    validate_equality_constraints,
    validate_sensor_constraints,
)
from robot2mjcf.postprocess.add_backlash import (
    add_backlash,
    add_backlash_default,
    add_backlash_joints,
    find_parent_body,
)


def write_text(path: Path, content: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)
    return path


def test_find_all_named_elements_collects_nested_names() -> None:
    worldbody = ET.fromstring(
        """
        <worldbody>
          <body name="base">
            <joint name="joint_a" />
            <site name="site_a" />
            <body name="arm">
              <joint name="joint_b" />
              <site name="site_b" />
            </body>
          </body>
        </worldbody>
        """
    )

    assert find_all_joints(worldbody) == {"joint_a", "joint_b"}
    assert find_all_bodies(worldbody) == {"base", "arm"}
    assert find_all_sites(worldbody) == {"site_a", "site_b"}


def test_validate_constraint_helpers_accept_only_existing_references() -> None:
    equality = ET.fromstring(
        """
        <equality>
          <joint joint1="joint_a" joint2="joint_b" />
          <joint joint1="joint_a" joint2="missing_joint" />
        </equality>
        """
    )
    contact = ET.fromstring(
        """
        <contact>
          <exclude body1="base" body2="arm" />
          <exclude body1="base" body2="missing_body" />
        </contact>
        """
    )
    sensor = ET.fromstring(
        """
        <sensor>
          <jointpos name="joint_sensor" joint="joint_a" />
          <framepos name="site_sensor" site="site_a" />
          <jointvel name="invalid_joint_sensor" joint="missing_joint" />
          <touch name="invalid_site_sensor" site="missing_site" />
        </sensor>
        """
    )

    assert validate_equality_constraints(equality, {"joint_a", "joint_b"})
    assert validate_contact_constraints(contact, {"base", "arm"})
    assert validate_sensor_constraints(sensor, {"joint_a"}, {"site_a"})


def test_add_filtered_constraints_only_appends_valid_entries() -> None:
    root = ET.fromstring("<mujoco />")
    equality = ET.fromstring(
        """
        <equality>
          <joint joint1="joint_a" joint2="joint_b" />
          <joint joint1="joint_a" joint2="missing_joint" />
        </equality>
        """
    )
    contact = ET.fromstring(
        """
        <contact>
          <exclude body1="base" body2="arm" />
          <exclude body1="base" body2="missing_body" />
        </contact>
        """
    )
    sensor = ET.fromstring(
        """
        <sensor>
          <jointpos name="joint_sensor" joint="joint_a" />
          <framepos name="site_sensor" site="site_a" />
          <jointvel name="invalid_joint_sensor" joint="missing_joint" />
          <touch name="invalid_site_sensor" site="missing_site" />
        </sensor>
        """
    )

    add_filtered_equality_constraints(root, equality, {"joint_a", "joint_b"})
    add_filtered_contact_constraints(root, contact, {"base", "arm"})
    add_filtered_sensor_constraints(root, sensor, {"joint_a"}, {"site_a"})

    assert len(root.findall("./equality/joint")) == 1
    assert len(root.findall("./contact/exclude")) == 1
    assert len(root.findall("./sensor/*")) == 2


def test_merge_elements_appends_new_and_updates_existing_attributes() -> None:
    root = ET.fromstring("<mujoco><option integrator='Euler' /></mujoco>")

    merge_elements(root, ET.fromstring("<visual><global offwidth='640' /></visual>"))
    merge_elements(root, ET.fromstring("<option integrator='implicitfast'><flag warmstart='enable' /></option>"))

    visual = root.find("visual")
    option = root.find("option")

    assert visual is not None
    assert visual.find("global") is not None
    assert option is not None
    assert option.attrib["integrator"] == "implicitfast"
    assert option.find("flag") is not None


def test_add_appendix_merges_generic_elements_and_filters_invalid_constraints(tmp_path) -> None:
    mjcf_path = write_text(
        tmp_path / "model.xml",
        """
        <mujoco>
          <worldbody>
            <body name="base">
              <joint name="joint_a" type="hinge" />
              <site name="site_a" />
              <body name="arm">
                <joint name="joint_b" type="hinge" />
              </body>
            </body>
          </worldbody>
        </mujoco>
        """.strip(),
    )
    appendix_path = write_text(
        tmp_path / "appendix.xml",
        """
        <option timestep="0.002" />
        <equality>
          <joint joint1="joint_a" joint2="joint_b" />
          <joint joint1="joint_a" joint2="missing_joint" />
        </equality>
        <contact>
          <exclude body1="base" body2="arm" />
          <exclude body1="base" body2="missing_body" />
        </contact>
        <sensor>
          <jointpos name="joint_sensor" joint="joint_a" />
          <touch name="touch_sensor" site="site_a" />
          <jointvel name="invalid_joint_sensor" joint="missing_joint" />
        </sensor>
        """.strip(),
    )

    add_appendix(mjcf_path, appendix_path)

    root = ET.parse(mjcf_path).getroot()
    option = root.find("./option")

    assert option is not None
    assert option.attrib["timestep"] == "0.002"
    assert len(root.findall("./equality/joint")) == 1
    assert len(root.findall("./contact/exclude")) == 1
    assert len(root.findall("./sensor/*")) == 2


def test_add_backlash_default_and_joints_extend_hinges() -> None:
    root = ET.fromstring(
        """
        <mujoco>
          <worldbody>
            <body name="base">
              <joint name="hinge_a" type="hinge" axis="0 1 0" pos="0 0 1" quat="1 0 0 0" />
              <joint name="hinge_skip" type="hinge" class="backlash" />
              <joint type="hinge" />
            </body>
          </worldbody>
        </mujoco>
        """
    )

    add_backlash_default(root, 0.2, 0.05)
    add_backlash_joints(root)

    default_joint = root.find("./default/default[@class='backlash']/joint")
    backlash_joint = root.find(".//joint[@name='hinge_a_backlash']")

    assert default_joint is not None
    assert default_joint.attrib["range"] == "-0.2 0.2"
    assert default_joint.attrib["damping"] == "0.05"
    assert backlash_joint is not None
    assert backlash_joint.attrib["class"] == "backlash"
    assert backlash_joint.attrib["axis"] == "0 1 0"
    assert backlash_joint.attrib["pos"] == "0 0 1"
    assert backlash_joint.attrib["quat"] == "1 0 0 0"
    assert root.find(".//joint[@name='hinge_skip_backlash']") is None


def test_find_parent_body_and_add_backlash_persist_to_disk(tmp_path) -> None:
    mjcf_path = write_text(
        tmp_path / "model.xml",
        """
        <mujoco>
          <worldbody>
            <body name="base">
              <joint name="hinge_a" type="hinge" />
            </body>
          </worldbody>
        </mujoco>
        """.strip(),
    )
    root = ET.parse(mjcf_path).getroot()
    joint = root.find(".//joint")

    assert joint is not None
    parent_body = find_parent_body(joint, root)
    assert parent_body is not None
    assert parent_body.attrib["name"] == "base"

    add_backlash(mjcf_path, 0.1, 0.02)

    saved_root = ET.parse(mjcf_path).getroot()
    saved_default_joint = saved_root.find("./default/default[@class='backlash']/joint")

    assert saved_root.find(".//joint[@name='hinge_a_backlash']") is not None
    assert saved_default_joint is not None
    assert saved_default_joint.attrib["damping"] == "0.02"
