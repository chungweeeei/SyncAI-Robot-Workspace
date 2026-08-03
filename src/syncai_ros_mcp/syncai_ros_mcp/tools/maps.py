"""Map tools for the ROS 2 MCP server.

Like ``tasks.py`` (and unlike the topic/service tools that introspect the live
ROS 2 graph), these tools are thin HTTP clients for ``syncai_backend``'s map API
(``interfaces/rest/routers/map.py``):

* ``GET    /api/v1/map/info``          -> map raster metadata (resolution/size/origin)
* ``GET    /api/v1/map/image``         -> map raster as a base64 PNG
* ``POST   /api/v1/map/vertices``      -> create one or more map vertices
* ``GET    /api/v1/map/vertices``      -> list vertices (optional map_name/type filter)
* ``GET    /api/v1/map/vertices/{id}`` -> read a single vertex
* ``PUT    /api/v1/map/vertices/{id}`` -> update a vertex
* ``DELETE /api/v1/map/vertices/{id}`` -> delete a vertex

The backend base URL defaults to ``http://localhost:3000`` (the port
``syncai_backend`` binds in ``interfaces/rest/server.py``) and can be overridden
with the ``SYNCAI_BACKEND_BASE_URL`` environment variable.
"""

import base64

from fastmcp import FastMCP
from fastmcp.utilities.types import Image
from mcp.types import ToolAnnotations

from syncai_ros_mcp.tools import _backend


# Semantic vertex roles accepted by the backend (see VertexType in map.py).
_VERTEX_TYPES = ("GENERAL", "ARTIFACT", "CHARGER", "HOME", "WAITING")

# Prefix the backend puts on the base64 PNG (occupancy_grid_to_png_base64).
_PNG_DATA_URI_PREFIX = "data:image/png;base64,"


def register_map_tools(mcp: FastMCP) -> None:
    """Register all map-related tools."""

    @mcp.tool(
        description=(
            "Get the current map raster metadata: resolution, pixel width/height, "
            "and world origin (GET /api/v1/map/info).\n"
            "Example:\nget_map_info()"
        ),
        annotations=ToolAnnotations(title="Get Map Info", readOnlyHint=True),
    )
    def get_map_info() -> dict:
        """
        Get metadata about the current map raster.

        Returns:
            dict: The backend MapInfoResponse ({'resolution', 'width', 'height',
                'origin': {'x', 'y', 'z'}}), or {'error': ...} if the map is not
                available yet / on transport failure.
        """
        return _backend.request("GET", "/api/v1/map/info")

    @mcp.tool(
        description=(
            "Get the current map as a PNG image (GET /api/v1/map/image).\n"
            "Returns an image content block, so the client renders the map "
            "directly. Use get_map_info() for resolution/origin metadata.\n"
            "Example:\nget_map_image()"
        ),
        annotations=ToolAnnotations(title="Get Map Image", readOnlyHint=True),
    )
    def get_map_image():
        """
        Get the current map raster rendered as a PNG.

        Returns:
            Image: the map PNG (rendered by the client) on success, or
                {'error': ...} if the map is not available yet / on failure.
        """
        body = _backend.request("GET", "/api/v1/map/image")
        if "error" in body:
            return body

        data_uri = body.get("image", "")
        if not data_uri.startswith(_PNG_DATA_URI_PREFIX):
            return {"error": "Backend returned an unexpected map image format."}

        try:
            png_bytes = base64.b64decode(data_uri[len(_PNG_DATA_URI_PREFIX):])
        except (ValueError, TypeError) as exc:
            return {"error": f"Failed to decode map image: {exc}"}

        return Image(data=png_bytes, format="png")

    @mcp.tool(
        description=(
            "Create one or more map vertices (POST /api/v1/map/vertices).\n"
            "Each vertex is a dict with these fields:\n"
            "  name (str), type (one of GENERAL/ARTIFACT/CHARGER/HOME/WAITING),\n"
            "  map_name (str), x (float, m), y (float, m), theta (float, degrees).\n"
            "Example:\n"
            "create_map_vertices(vertices=[{'name': 'dock-A', 'type': 'CHARGER', "
            "'map_name': 'factory', 'x': 1.0, 'y': 2.0, 'theta': 90.0}])"
        ),
        annotations=ToolAnnotations(title="Create Map Vertices"),
    )
    def create_map_vertices(vertices: list = None) -> dict:
        """
        Create one or more map vertices on the SyncAI backend.

        Args:
            vertices (list): Non-empty list of vertex dicts, each with 'name',
                'type', 'map_name', 'x', 'y', and 'theta'.

        Returns:
            dict: {'vertices': [MapVertexResponse, ...]} on success, or
                {'error': ...} on validation/transport failure.
        """
        if not vertices:
            return {"error": "vertices cannot be empty; provide at least one vertex"}

        if not isinstance(vertices, list):
            return {"error": "vertices must be a list of vertex objects"}

        for i, vertex in enumerate(vertices):
            if not isinstance(vertex, dict):
                return {"error": f"vertices[{i}] must be an object"}
            vtype = vertex.get("type")
            if vtype not in _VERTEX_TYPES:
                return {
                    "error": f"vertices[{i}].type must be one of "
                    f"{', '.join(_VERTEX_TYPES)} (got {vtype!r})"
                }

        body = _backend.request("POST", "/api/v1/map/vertices", json=vertices)
        if isinstance(body, dict) and "error" in body:
            return body
        return {"vertices": body}

    @mcp.tool(
        description=(
            "List map vertices, optionally filtered by map name and/or type "
            "(GET /api/v1/map/vertices).\n"
            "type, when given, must be one of "
            "GENERAL/ARTIFACT/CHARGER/HOME/WAITING.\n"
            "Example:\n"
            "list_map_vertices()\n"
            "list_map_vertices(map_name='factory', type='CHARGER')"
        ),
        annotations=ToolAnnotations(title="List Map Vertices", readOnlyHint=True),
    )
    def list_map_vertices(map_name: str = None, type: str = None) -> dict:
        """
        List map vertices, optionally filtered.

        Args:
            map_name (str): Only return vertices belonging to this map.
            type (str): Only return vertices of this semantic role.

        Returns:
            dict: {'vertices': [MapVertexResponse, ...]} on success, or
                {'error': ...} on failure.
        """
        if type is not None and type not in _VERTEX_TYPES:
            return {
                "error": f"type must be one of {', '.join(_VERTEX_TYPES)} "
                f"(got {type!r})"
            }

        params = {}
        if map_name:
            params["map_name"] = map_name
        if type:
            params["type"] = type

        body = _backend.request("GET", "/api/v1/map/vertices", params=params)
        if isinstance(body, dict) and "error" in body:
            return body
        return {"vertices": body}

    @mcp.tool(
        description=(
            "Get a single map vertex by its id "
            "(GET /api/v1/map/vertices/{id}).\n"
            "Example:\nget_map_vertex('3fa85f64-5717-4562-b3fc-2c963f66afa6')"
        ),
        annotations=ToolAnnotations(title="Get Map Vertex", readOnlyHint=True),
    )
    def get_map_vertex(vertex_id: str) -> dict:
        """
        Get a single map vertex.

        Args:
            vertex_id (str): The vertex UUID.

        Returns:
            dict: The backend MapVertexResponse on success, or {'error': ...}
                on failure (e.g. not found).
        """
        vertex_id = (vertex_id or "").strip()
        if not vertex_id:
            return {"error": "vertex_id cannot be empty"}

        return _backend.request("GET", f"/api/v1/map/vertices/{vertex_id}")

    @mcp.tool(
        description=(
            "Update fields of an existing map vertex "
            "(PUT /api/v1/map/vertices/{id}).\n"
            "Only the fields you pass are changed. Updatable fields: name (str), "
            "type (GENERAL/ARTIFACT/CHARGER/HOME/WAITING), map_name (str), "
            "x (float, m), y (float, m), theta (float, degrees).\n"
            "Example:\n"
            "update_map_vertex('3fa85f64-...', name='dock-B', theta=180.0)"
        ),
        annotations=ToolAnnotations(title="Update Map Vertex"),
    )
    def update_map_vertex(
        vertex_id: str,
        name: str = None,
        type: str = None,
        map_name: str = None,
        x: float = None,
        y: float = None,
        theta: float = None,
    ) -> dict:
        """
        Update an existing map vertex. Only provided fields are changed.

        Args:
            vertex_id (str): The vertex UUID.
            name (str): New vertex name.
            type (str): New semantic role (one of the VertexType values).
            map_name (str): New owning map name.
            x (float): New world x-coordinate (metres).
            y (float): New world y-coordinate (metres).
            theta (float): New yaw angle in degrees.

        Returns:
            dict: The updated MapVertexResponse on success, or {'error': ...}
                on failure.
        """
        vertex_id = (vertex_id or "").strip()
        if not vertex_id:
            return {"error": "vertex_id cannot be empty"}

        if type is not None and type not in _VERTEX_TYPES:
            return {
                "error": f"type must be one of {', '.join(_VERTEX_TYPES)} "
                f"(got {type!r})"
            }

        # Send only the fields the caller actually provided, so omitted fields
        # keep their current values (the backend uses exclude_unset semantics).
        changes = {
            key: value
            for key, value in (
                ("name", name),
                ("type", type),
                ("map_name", map_name),
                ("x", x),
                ("y", y),
                ("theta", theta),
            )
            if value is not None
        }
        if not changes:
            return {"error": "provide at least one field to update"}

        return _backend.request("PUT", f"/api/v1/map/vertices/{vertex_id}", json=changes)

    @mcp.tool(
        description=(
            "Delete a map vertex by its id "
            "(DELETE /api/v1/map/vertices/{id}).\n"
            "Example:\ndelete_map_vertex('3fa85f64-5717-4562-b3fc-2c963f66afa6')"
        ),
        annotations=ToolAnnotations(title="Delete Map Vertex", destructiveHint=True),
    )
    def delete_map_vertex(vertex_id: str) -> dict:
        """
        Delete a map vertex.

        Args:
            vertex_id (str): The vertex UUID.

        Returns:
            dict: The backend DeleteResponse ({'message': ...}) on success, or
                {'error': ...} on failure.
        """
        vertex_id = (vertex_id or "").strip()
        if not vertex_id:
            return {"error": "vertex_id cannot be empty"}

        return _backend.request("DELETE", f"/api/v1/map/vertices/{vertex_id}")
