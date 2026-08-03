"""Service tools for the ROS 2 MCP server."""

import threading

from fastmcp import FastMCP
from mcp.types import ToolAnnotations

from rclpy.node import Node

from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message, get_service


def register_service_tools(mcp: FastMCP, node: Node) -> None:
    """Register all service-related tools."""

    @mcp.tool(
        description="Get list of all available ROS services.\nExample:\nget_services()",
        annotations=ToolAnnotations(
            title="Get Services",
            readOnlyHint=True,
        ),
    )
    def get_services() -> dict:
        """
        Fetch the available services from the live ROS 2 node graph.

        Returns:
            dict: Contains two lists, 'services' and 'types'.
        """
        names_and_types = node.get_service_names_and_types()

        return {
            "services": [name for name, _ in names_and_types],
            "types": [types for _, types in names_and_types],
        }
    
    @mcp.tool(
        description=(
            "Get the service type for a specific service.\nExample:\nget_service_type('/rosapi/topics')"
        ),
        annotations=ToolAnnotations(
            title="Get Service Type",
            readOnlyHint=True,
        ),
    )
    def get_service_type(service: str) -> dict:
        """
        Get the service type for a specific service.

        Args:
            service (str): The service name (e.g., '/rosapi/topics')

        Returns:
            dict: Contains the service type,
                or an error message if service doesn't exist.
        """
        if not service or not service.strip():
            return {"error": "Service name cannot be empty"}

        names_and_types = dict(node.get_service_names_and_types())

        if service not in names_and_types:
            return {"error": f"Service '{service}' not found."}

        return {
            "service": service,
            "types": names_and_types[service],
        }
    
    @mcp.tool(
        description=(
            "Get complete service details including request/response structures and provider nodes.\n"
            "Example:\n"
            "get_service_details('/rosapi/topics')"
        ),
        annotations=ToolAnnotations(
            title="Get Service Details",
            readOnlyHint=True,
        ),
    )
    def get_service_details(service: str) -> dict:
        """
        Get complete service details including request/response structures and provider nodes.

        Args:
            service (str): The service name (e.g., '/rosapi/topics')

        Returns:
            dict: Contains complete service definition with request and response structures,
                provider nodes, and provider count.
        """
        if not service or not service.strip():
            return {"error": "Service name cannot be empty"}

        names_and_types = dict(node.get_service_names_and_types())

        if service not in names_and_types:
            return {"error": f"Service '{service}' not found."}

        types = names_and_types[service]

        try:
            srv_class = get_service(types[0])
        except (ValueError, AttributeError, ModuleNotFoundError, ImportError, IndexError):
            return {"error": f"Service type '{types[0] if types else ''}' not found."}

        def _base_type(field_type: str) -> str:
            """Strip sequence/array decorations to the underlying type."""
            t = field_type
            if t.startswith("sequence<") and t.endswith(">"):
                t = t[len("sequence<"):-1].split(",")[0].strip()
            if "[" in t:
                t = t[:t.index("[")]
            return t

        def _structure(msg_class) -> dict:
            """Expand a message class into its fields plus any nested types."""
            fields = msg_class.get_fields_and_field_types()
            nested_types: dict = {}
            pending = [
                _base_type(t) for t in fields.values() if "/" in _base_type(t)
            ]
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
            return {"fields": fields, "nested_types": nested_types}

        # Find nodes that offer this service (i.e. run a server for it).
        providers = []
        for node_name, ns in node.get_node_names_and_namespaces():
            offered = dict(
                node.get_service_names_and_types_by_node(node_name, ns)
            )
            if service in offered:
                full_name = (
                    f"/{node_name}" if ns == "/" else f"{ns}/{node_name}"
                )
                providers.append(full_name)

        return {
            "service": service,
            "types": types,
            "request": _structure(srv_class.Request),
            "response": _structure(srv_class.Response),
            "provider_count": len(providers),
            "providers": providers,
        }
    
    @mcp.tool(
        description=(
            "Call a ROS service with specified request data.\n"
            "Example:\n"
            "call_service('/rosapi/topics', 'rosapi/Topics', {})\n"
            "call_service('/slow_service', 'my_package/SlowService', {}, timeout=10.0)  # Specify timeout only for slow services\n"
            "\n"
            "IMPORTANT: Field names in the request dict should match the field names shown by get_service_details(), "
            "which are already formatted for rosbridge (without leading underscores). "
            "For example, use {'topic': '/image'} not {'_topic': '/image'}."
        ),
        annotations=ToolAnnotations(
            title="Call Service",
            destructiveHint=True,
        ),
    )
    def call_service(
        service_name: str,
        service_type: str,
        request: dict,
        timeout: float = None,
    ) -> dict:
        """
        Call a ROS service with specified request data.

        Args:
            service_name (str): The service name (e.g., '/rosapi/topics')
            service_type (str): The service type (e.g., 'rosapi/Topics')
            request (dict): Service request data as a dictionary
            timeout (float): Timeout in seconds. If None, uses ws_manager.default_timeout.

        Returns:
            dict: Contains the service response or error information.
        """
        request = request or {}
        service_name = service_name.strip()
        service_type = service_type.strip()

        if not service_name:
            return {"error": "Service name cannot be empty"}

        if not service_type:
            return {"error": "Service type cannot be empty"}

        timeout = 5.0 if timeout is None else timeout
        if timeout <= 0:
            return {"error": "timeout must be positive"}

        try:
            srv_class = get_service(service_type)
        except (ValueError, AttributeError, ModuleNotFoundError, ImportError):
            return {"error": f"Service type '{service_type}' not found."}

        try:
            request_instance = srv_class.Request()
            set_message_fields(request_instance, request)
        except Exception as exc:  # noqa: BLE001 - surface any field error to caller
            return {"error": f"Failed to populate request: {exc}"}

        # Create a client just for this call and tear it down afterwards so
        # nothing accumulates on the node across calls.
        client = node.create_client(srv_class, service_name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return {
                    "error": f"Service '{service_name}' not available after "
                             f"{timeout}s."
                }

            # The node spins on the main thread, so the future is completed by
            # that executor; wait for it here via a done callback + Event.
            future = client.call_async(request_instance)
            event = threading.Event()
            future.add_done_callback(lambda _f: event.set())

            if not event.wait(timeout):
                future.cancel()
                return {
                    "error": f"Timed out after {timeout}s waiting for a "
                             f"response from '{service_name}'."
                }

            response = future.result()
        except Exception as exc:  # noqa: BLE001 - report call failure to caller
            return {"error": f"Failed to call service: {exc}"}
        finally:
            node.destroy_client(client)

        return {
            "success": True,
            "service": service_name,
            "service_type": service_type,
            "response": message_to_ordereddict(response),
        }