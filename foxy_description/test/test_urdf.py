import subprocess
from pathlib import Path
import pytest
from ament_index_python.packages import get_package_share_directory

@pytest.fixture
def get_xacro_path():
    return Path(get_package_share_directory("foxy_description"), "urdf", "foxy.urdf.xacro")


def test_urdf(get_xacro_path):
    cmd = f"check_urdf <(xacro {get_xacro_path})"

    check_urdf = subprocess.run(["bash", "-c", cmd],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                text=True)

    if check_urdf.returncode != 0:
        pytest.fail(f"check_urdf failed with exit code {check_urdf.returncode}. {check_urdf.stderr}")
