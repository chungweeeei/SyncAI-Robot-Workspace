from dataclasses import dataclass

import psutil
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# The root filesystem. Inside the robot container this is the overlayfs, so the
# figures describe the host's docker storage partition rather than any
# robot-specific volume — which is still the number that matters, because that
# partition is what fills up and takes the stack down with it.
DISK_MOUNT_POINT = "/"

MONITOR_PERIOD_SEC = 1.0

BYTES_PER_GIB = 1024**3


@dataclass
class MemoryUsage:
    total: int
    used: int
    available: int
    percent: float


@dataclass
class DiskUsage:
    total: int
    used: int
    free: int
    percent: float


class MonitorManager:
    """Reports host memory and disk usage to stdout once a second.

    Reporting only — nothing here throttles or shuts anything down. The readout
    lands on stdout rather than a topic because the byobu pipe-pane tap is what
    makes this stack's console output persistent (ROS_LOG_DIR is a tmpfs), so a
    log line is already a durable, rotated record under
    log/stack/<robot_id>/sys_manager/.
    """

    def __init__(self, node: Node):
        self._node = node
        self._logger = node.get_logger()

        self.init_timer()

    def init_timer(self):

        self._monitor_timer = self._node.create_timer(
            timer_period_sec=MONITOR_PERIOD_SEC,
            callback=self._log_resource_usage,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def get_memory_usage(self) -> MemoryUsage:
        # psutil reads /proc/meminfo, which Docker does NOT namespace: inside the
        # robot container this reports host-wide memory, not the container's
        # cgroup limit. For a robot that owns its whole compute board that is the
        # useful number, but it is not the container's own footprint.
        memory = psutil.virtual_memory()
        return MemoryUsage(
            total=memory.total,
            used=memory.used,
            available=memory.available,
            percent=memory.percent,
        )

    def get_disk_usage(self) -> DiskUsage:
        disk = psutil.disk_usage(DISK_MOUNT_POINT)
        return DiskUsage(
            total=disk.total,
            used=disk.used,
            free=disk.free,
            percent=disk.percent,
        )

    def _log_resource_usage(self):
        # An exception escaping a timer callback takes the callback out of the
        # executor for good, so a transient read failure must not propagate —
        # losing one sample is fine, losing the timer is not.
        try:
            memory = self.get_memory_usage()
            disk = self.get_disk_usage()
        except Exception as err:
            self._logger.error(
                f"[MonitorManager][_log_resource_usage] Failed to read resource usage: {str(err)}"
            )
            return

        # `total - available`, not psutil's `used`: psutil derives `percent` from
        # `available`, while `used` excludes buffers/cache — printing `used`
        # next to `percent` yields two numbers that visibly disagree (e.g.
        # "9.0GiB/122.8GiB (8.4%)" when 9.0/122.8 is 7.4%).
        memory_used = self._format_bytes(memory.total - memory.available)
        memory_total = self._format_bytes(memory.total)
        disk_used = self._format_bytes(disk.used)
        disk_total = self._format_bytes(disk.total)

        self._logger.info(
            f"[MonitorManager][_log_resource_usage] "
            f"memory {memory_used}/{memory_total} ({memory.percent:.1f}%), "
            f"disk {DISK_MOUNT_POINT} {disk_used}/{disk_total} ({disk.percent:.1f}%)"
        )

    @staticmethod
    def _format_bytes(num_bytes: int) -> str:
        return f"{num_bytes / BYTES_PER_GIB:.1f}GiB"


def init_monitor_manager(node: Node) -> MonitorManager:
    node.get_logger().info(
        "[MonitorManager][init_monitor_manager] Initializing Resource Monitoring module"
    )
    return MonitorManager(node=node)
