"""Topic tools for the ROS 2 MCP server."""

import threading
import time

from fastmcp import FastMCP
from mcp.types import ToolAnnotations

from rclpy import qos
from rclpy.duration import Duration
from rclpy.node import Node

from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message


def register_topic_tools(mcp: FastMCP, node: Node) -> None:
    """Register all topic-related tools."""

    @mcp.tool(
        description="Get list of all available ROS topics.\nExample:\nget_topics()",
        annotations=ToolAnnotations(title="Get Topics", readOnlyHint=True),
    )
    def get_topics() -> dict:
        """
        Fetch the available topics from the live ROS 2 node graph.

        Returns
        -------
        dict
            Contains two lists: 'topics' and 'types'.
        """
        names_and_types = node.get_topic_names_and_types()

        return {
            "topics": [name for name, _ in names_and_types],
            "types": [types for _, types in names_and_types],
        }

    @mcp.tool(
        description="Get the message type for a specific topic.\n"
        "Example:\nget_topic_type('/cmd_vel')",
        annotations=ToolAnnotations(title="Get Topic Type", readOnlyHint=True),
    )
    def get_topic_type(topic: str) -> dict:
        """
        Get the message type(s) for a specific topic.

        Args:
            topic (str): The topic name (e.g., '/cmd_vel').

        Returns:
            dict: Contains the 'topic' and its 'types' (a list, since a
                topic may be published with more than one type), or an
                'error' message if the topic doesn't exist.
        """
        if not topic or not topic.strip():
            return {"error": "Topic name cannot be empty"}

        names_and_types = dict(node.get_topic_names_and_types())

        if topic not in names_and_types:
            return {"error": f"Topic '{topic}' not found."}

        return {
            "topic": topic,
            "types": names_and_types[topic],
        }

    @mcp.tool(
        description="Get detailed info (type, publishers, subscribers) for a "
        "specific topic.\nExample:\nget_topic_details('/cmd_vel')",
        annotations=ToolAnnotations(title="Get Topic Details", readOnlyHint=True),
    )
    def get_topic_details(topic: str) -> dict:
        """
        Get detailed information about a specific topic including its
        type, publishers, and subscribers.

        Args:
            topic (str): The topic name (e.g., '/cmd_vel').

        Returns:
            dict: Contains the topic 'types', publisher/subscriber counts,
                and per-endpoint details (node, type, QoS), or an 'error'
                message if the topic doesn't exist.
        """
        if not topic or not topic.strip():
            return {"error": "Topic name cannot be empty"}

        names_and_types = dict(node.get_topic_names_and_types())

        if topic not in names_and_types:
            return {"error": f"Topic '{topic}' not found."}

        def _endpoint(info) -> dict:
            ns = info.node_namespace
            full_name = f"/{info.node_name}" if ns == "/" else f"{ns}/{info.node_name}"
            qos = info.qos_profile
            return {
                "node": full_name,
                "topic_type": info.topic_type,
                "reliability": qos.reliability.name,
                "durability": qos.durability.name,
            }

        publishers = [
            _endpoint(info) for info in node.get_publishers_info_by_topic(topic)
        ]
        subscribers = [
            _endpoint(info) for info in node.get_subscriptions_info_by_topic(topic)
        ]

        return {
            "topic": topic,
            "types": names_and_types[topic],
            "publisher_count": len(publishers),
            "subscriber_count": len(subscribers),
            "publishers": publishers,
            "subscribers": subscribers,
        }

    @mcp.tool(
        description=(
            "Get the complete structure/definition of a message type.\n"
            "Example:\n"
            "get_message_details('geometry_msgs/Twist')"
        ),
        annotations=ToolAnnotations(
            title="Get Message Details",
            readOnlyHint=True,
        ),
    )
    def get_message_details(message_type: str) -> dict:
        """
        Get the complete structure/definition of a message type.

        Args:
            message_type (str): The message type (e.g., 'geometry_msgs/Twist')

        Returns:
            dict: Contains the message structure with field names and types.
                Nested (non-primitive) field types are expanded under
                'nested_types'. Returns an 'error' message if the message
                type doesn't exist.
        """
        if not message_type or not message_type.strip():
            return {"error": "Message type cannot be empty"}

        def _base_type(field_type: str) -> str:
            """Strip sequence/array decorations to the underlying type."""
            t = field_type
            if t.startswith("sequence<") and t.endswith(">"):
                t = t[len("sequence<") : -1].split(",")[0].strip()
            if "[" in t:
                t = t[: t.index("[")]
            return t

        try:
            msg_class = get_message(message_type.strip())
        except (ValueError, AttributeError, ModuleNotFoundError, ImportError):
            return {"error": f"Message type '{message_type}' not found."}

        fields = msg_class.get_fields_and_field_types()

        # Recursively expand any nested message-typed fields, guarding
        # against cycles via `visited`.
        nested_types: dict = {}
        pending = [_base_type(t) for t in fields.values() if "/" in _base_type(t)]
        visited: set = set()
        while pending:
            type_name = pending.pop()
            if type_name in visited:
                continue
            visited.add(type_name)
            try:
                sub_fields = get_message(type_name).get_fields_and_field_types()
            except (ValueError, AttributeError, ModuleNotFoundError, ImportError):
                continue
            nested_types[type_name] = sub_fields
            pending.extend(
                _base_type(t) for t in sub_fields.values() if "/" in _base_type(t)
            )

        return {
            "message_type": message_type.strip(),
            "fields": fields,
            "nested_types": nested_types,
        }

    @mcp.tool(
        description=(
            "Publish a single message to a ROS topic.\n"
            "Example:\n"
            "publish_once(topic='/cmd_vel', msg_type='geometry_msgs/msg/TwistStamped', msg={'linear': {'x': 1.0}})"
        ),
        annotations=ToolAnnotations(
            title="Publish Once",
            destructiveHint=True,
        ),
    )
    def publish_once(topic: str = "", msg_type: str = "", msg: dict = None) -> dict:
        """
        Publish a single message to a ROS topic directly from this node.

        Args:
            topic (str): ROS topic name (e.g., "/cmd_vel").
            msg_type (str): ROS message type (e.g., "geometry_msgs/msg/Twist").
            msg (dict): Message payload as a dictionary. Fields omitted keep
                their default values.

        Returns:
            dict:
                - {"success": True, ...} with the subscriber count reached.
                - {"error": "<error message>"} on invalid input or failure.
        """
        msg = msg or {}

        if not topic or not topic.strip():
            return {"error": "Topic name cannot be empty"}

        if not msg_type or not msg_type.strip():
            return {"error": "Message type cannot be empty"}

        try:
            msg_class = get_message(msg_type.strip())
        except (ValueError, AttributeError, ModuleNotFoundError, ImportError):
            return {"error": f"Message type '{msg_type}' not found."}

        topic = topic.strip()
        msg_type = msg_type.strip()

        try:
            msg_instance = msg_class()
            set_message_fields(msg_instance, msg)
        except Exception as exc:  # noqa: BLE001 - surface any field error to caller
            return {"error": f"Failed to populate message: {exc}"}

        # Create a publisher just for this call and tear it down afterwards so
        # nothing accumulates on the node across calls.
        publisher = node.create_publisher(
            msg_type=msg_class,
            topic=topic,
            qos_profile=qos.QoSProfile(
                depth=5,
                reliability=qos.ReliabilityPolicy.RELIABLE,
                durability=qos.DurabilityPolicy.VOLATILE,
                history=qos.HistoryPolicy.KEEP_LAST,
            ),
        )
        try:
            # A freshly created publisher may not have discovered subscribers
            # yet; wait briefly (bounded) for a match so the message isn't
            # dropped.
            deadline = time.monotonic() + 1.0
            while (
                publisher.get_subscription_count() == 0 and time.monotonic() < deadline
            ):
                time.sleep(0.05)

            subscriber_count = publisher.get_subscription_count()
            publisher.publish(msg=msg_instance)

            # Ensure the sample is actually delivered before destroying the
            # publisher: wait_for_all_acked covers reliable subscribers, and a
            # short settle covers best-effort ones (nothing to ack).
            publisher.wait_for_all_acked(Duration(seconds=3.0))
            time.sleep(0.1)
        except Exception as exc:  # noqa: BLE001 - report publish failure to caller
            return {"error": f"Failed to publish: {exc}"}
        finally:
            node.destroy_publisher(publisher)

        return {
            "success": True,
            "topic": topic,
            "msg_type": msg_type,
            "subscriber_count": subscriber_count,
        }

    @mcp.tool(
        description=(
            "Subscribe to a ROS topic and return the first message received.\n"
            "Example:\n"
            "subscribe_once(topic='/cmd_vel', msg_type='geometry_msgs/msg/TwistStamped')\n"
            "subscribe_once(topic='/slow_topic', msg_type='my_package/SlowMsg', timeout=10.0)  # Use longer timeout for slow topics\n"
            "subscribe_once(topic='/high_rate_topic', msg_type='sensor_msgs/Image', timeout=5.0, queue_length=5, throttle_rate_ms=100)  # Control message buffering and rate\n"
            "subscribe_once(topic='/camera/image_raw', msg_type='sensor_msgs/Image', expects_image='true')  # Hint that this is an image for faster processing\n"
            "subscribe_once(topic='/point_cloud', msg_type='sensor_msgs/PointCloud2', expects_image='false')  # Skip image detection for non-image data"
        ),
        annotations=ToolAnnotations(
            title="Subscribe Once",
            readOnlyHint=True,
        ),
    )
    def subscribe_once(
        topic: str = "",
        msg_type: str = "",
        expects_image: str = "auto",
        timeout: float = None,  # type: ignore[assignment]  # See issue #140
        queue_length: int = None,  # type: ignore[assignment]  # See issue #140
        throttle_rate_ms: int = None,  # type: ignore[assignment]  # See issue #140
    ):
        """
        Subscribe to a given ROS topic via rosbridge and return the first message received.

        Args:
            topic (str): The ROS topic name (e.g., "/cmd_vel", "/joint_states").
            msg_type (str): The ROS message type (e.g., "geometry_msgs/Twist").
            timeout (float): Timeout in seconds. If None, uses ws_manager.default_timeout.
            queue_length (int): How many messages to buffer before dropping old ones. Must be ≥ 1. Default is 1.
            throttle_rate_ms (int): Minimum interval between messages in milliseconds. Must be ≥ 0. Default is 0 (no throttling).
            expects_image (str): Hint about whether to expect image data.
                - "true": prioritize image parsing (use for sensor_msgs/Image topics)
                - "false": skip image detection for faster processing (use for non-image topics)
                - "auto": auto-detect based on message fields (default)

        Returns:
            dict:
                - {"msg": <parsed ROS message>} if successful
                - {"error": "<error message>"} if subscription or timeout fails
        """
        topic = topic.strip()
        msg_type = msg_type.strip()

        if not topic:
            return {"error": "Topic name cannot be empty"}

        if not msg_type:
            return {"error": "Message type cannot be empty"}

        if expects_image not in ("true", "false", "auto"):
            return {"error": "expects_image must be 'true', 'false' or 'auto'"}

        timeout = 5.0 if timeout is None else timeout
        if timeout <= 0:
            return {"error": "timeout must be positive"}

        queue_length = 1 if queue_length is None else queue_length
        if queue_length < 1:
            return {"error": "queue_length must be >= 1"}

        if throttle_rate_ms is not None and throttle_rate_ms < 0:
            return {"error": "throttle_rate_ms must be >= 0"}

        try:
            msg_class = get_message(msg_type)
        except (ValueError, AttributeError, ModuleNotFoundError, ImportError):
            return {"error": f"Message type '{msg_type}' not found."}

        # The callback runs on the executor thread (the node spins on the main
        # thread); it hands the first message to this thread via an Event.
        received: dict = {}
        event = threading.Event()

        def _callback(message) -> None:
            if not event.is_set():
                received["msg"] = message
                event.set()

        # TODO: map topics (e.g. /map, nav_msgs/OccupancyGrid) need their own
        #       QoS profile — typically TRANSIENT_LOCAL durability + KEEP_LAST(1)
        #       so a late subscriber still receives the latched map. The current
        #       hard-coded VOLATILE profile below will miss it.
        subscription = node.create_subscription(
            msg_type=msg_class,
            topic=topic,
            callback=_callback,
            qos_profile=qos.QoSProfile(
                depth=queue_length,
                reliability=qos.ReliabilityPolicy.RELIABLE,
                durability=qos.DurabilityPolicy.VOLATILE,
                history=qos.HistoryPolicy.KEEP_LAST,
            ),
        )
        try:
            got_message = event.wait(timeout)
        finally:
            node.destroy_subscription(subscription)

        if not got_message:
            return {
                "error": f"Timed out after {timeout}s waiting for a message "
                f"on '{topic}'."
            }

        # TODO: image messages need dedicated post-processing here — when
        #       expects_image is "true" (or auto-detected), decode/encode the
        #       image payload (e.g. sensor_msgs/Image, CompressedImage) instead
        #       of returning the raw byte buffer via message_to_ordereddict.
        return {"msg": message_to_ordereddict(received["msg"])}
