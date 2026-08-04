"""Device naming helpers for Foxy launch files and ROS nodes."""

import os
import re
import socket
from typing import Optional


def get_device_hostname() -> str:
    """Return the operating system hostname.

    Returns:
        The local device hostname, or ``"host"`` if the hostname is empty.
    """
    return socket.gethostname().strip() or "host"


def sanitize_ros_namespace(value: str) -> str:
    """Convert a value into a valid single-token ROS 2 namespace.

    Examples:
        ``foxy-01``       -> ``foxy_01``
        ``My Foxy.local`` -> ``my_foxy_local``
        ``123-robot``     -> ``host_123_robot``
    """
    namespace = value.strip().lower()
    namespace = namespace.replace("-", "_")

    # Replace dots, spaces, and unsupported characters.
    namespace = re.sub(r"[^a-z0-9_]", "_", namespace)

    # Collapse repeated underscores and remove edge underscores.
    namespace = re.sub(r"_+", "_", namespace).strip("_")

    if not namespace:
        namespace = "host"

    # A ROS name token must not start with a number.
    if namespace[0].isdigit():
        namespace = f"host_{namespace}"

    return namespace


def get_robot_hostname(
    override: Optional[str] = None,
    environment_variable: str = "FOXY_NAME",
) -> str:
    """Return the namespace for the current Foxy device.

    Resolution order:

    1. Explicit ``override`` argument
    2. Value of ``FOXY_NAME``
    3. Current device hostname

    The selected value is always converted into a valid ROS namespace.
    """
    name = override or os.environ.get(environment_variable)

    if not name:
        name = get_device_hostname()

    return sanitize_ros_namespace(name)
