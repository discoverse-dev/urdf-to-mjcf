"""Basic import and CLI smoke tests."""

import subprocess
import sys

import urdf2mjcf


def test_version() -> None:
    assert urdf2mjcf.__version__
    assert isinstance(urdf2mjcf.__version__, str)


def test_run_exported() -> None:
    assert callable(urdf2mjcf.run)


def test_cli_help() -> None:
    result = subprocess.run(
        [sys.executable, "-m", "urdf2mjcf.convert", "--help"],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0
    assert "Convert a URDF file to an MJCF file" in result.stdout
