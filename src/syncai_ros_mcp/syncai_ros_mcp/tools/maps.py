"""Map tools for the ROS 2 MCP server.

Like ``tasks.py`` (and unlike the topic/service tools that introspect the live
ROS 2 graph), these tools are thin HTTP clients for ``syncai_backend``'s map API
(``interfaces/rest/routers/map.py``). Every map route is nested under the map's
directory name, and the vertex routes are nested under their owning map:

* ``GET    /api/v1/maps``                          -> catalogue of stored maps
* ``GET    /api/v1/maps/{name}``                   -> one map's summary
* ``PATCH  /api/v1/maps/{name}``                   -> rename a map (no tool here yet)
* ``GET    /api/v1/maps/{name}/image``             -> gridmap as raw PNG bytes
* ``GET    /api/v1/maps/{name}/vertices``          -> list vertices (optional type filter)
* ``POST   /api/v1/maps/{name}/vertices``          -> create one or more vertices
* ``GET    /api/v1/maps/{name}/vertices/{id}``     -> read a single vertex
* ``PUT    /api/v1/maps/{name}/vertices/{id}``     -> update a vertex
* ``DELETE /api/v1/maps/{name}/vertices/{id}``     -> delete a vertex

The old flat ``/api/v1/map`` routes these tools were written against no longer
exist on the backend (every call 404'd), and two of their conventions went with
them: the vertex body no longer carries ``map_name`` (the URL owns it, so a body
naming a different map cannot contradict the path it was posted to), and the
image endpoint answers with ``image/png`` bytes rather than a base64 JSON
envelope. Moving a vertex between maps is therefore a delete and a create.

The backend base URL defaults to ``http://localhost:3000`` (the port
``syncai_backend`` binds in ``interfaces/rest/server.py``) and can be overridden
with the ``SYNCAI_BACKEND_BASE_URL`` environment variable.
"""

from fastmcp import FastMCP
from fastmcp.utilities.types import Image
from mcp.types import ToolAnnotations

from syncai_ros_mcp.tools import _backend


# Semantic vertex roles accepted by the backend (see VertexType in map.py).
_VERTEX_TYPES = ("GENERAL", "ARTIFACT", "CHARGER", "HOME", "WAITING")

# Fields the backend's MapVertexRequest accepts. Anything else in a vertex dict
# is rejected up front rather than silently dropped by pydantic, so a caller
# who passes the retired ``map_name`` learns that the URL owns it now.
_VERTEX_FIELDS = ("name", "type", "x", "y", "theta")


def _map_path(map_name: str) -> str:
    """Path prefix for one map's routes, or '' when the name is blank."""
    map_name = (map_name or "").strip()
    return f"/api/v1/maps/{map_name}" if map_name else ""


def _type_error(vtype) -> dict:
    return {
        "error": f"type must be one of {', '.join(_VERTEX_TYPES)} (got {vtype!r})"
    }


def register_map_tools(mcp: FastMCP) -> None:
    """Register all map-related tools."""

    @mcp.tool(
        description=(
            "List every map stored on the robot (GET /api/v1/maps).\n"
            "Each entry is a MapSummaryResponse: name, active (true for the map "
            "the stack was launched with — the one navigation is localizing "
            "against right now), grid (resolution / origin{x, y, yaw} / width / "
            "height, or null when the map has a point cloud but no gridmap yet), "
            "thumbnail, has_pointcloud, grid_converting, size_bytes, modified_at, "
            "vertex_count.\n"
            "Example:\nlist_maps()"
        ),
        annotations=ToolAnnotations(title="List Maps", readOnlyHint=True),
    )
    def list_maps() -> dict:
        """
        List the maps in the backend's map catalogue.

        Returns:
            dict: {'maps': [MapSummaryResponse, ...]} on success, or
                {'error': ...} on transport failure.
        """
        body = _backend.request("GET", "/api/v1/maps")
        if isinstance(body, dict) and "error" in body:
            return body
        return {"maps": body}

    @mcp.tool(
        description=(
            "Get one stored map's summary (GET /api/v1/maps/{name}): name, "
            "active, grid geometry (resolution, origin{x, y, yaw}, width, height "
            "— null until the point cloud has been converted to a gridmap), "
            "thumbnail, has_pointcloud, grid_converting, size_bytes, modified_at, "
            "vertex_count.\n"
            "There is no separate 'currently loaded map' endpoint: the entry "
            "with active=true in list_maps() is the map the stack was launched "
            "with, so call this with that name for the live map's geometry.\n"
            "Example:\nget_map_info('factory')"
        ),
        annotations=ToolAnnotations(title="Get Map Info", readOnlyHint=True),
    )
    def get_map_info(map_name: str) -> dict:
        """
        Get the summary of a single stored map.

        Args:
            map_name (str): The map's directory name (as listed by list_maps).

        Returns:
            dict: The backend MapSummaryResponse on success, or {'error': ...}
                on failure (e.g. 404 for an unknown map).
        """
        path = _map_path(map_name)
        if not path:
            return {"error": "map_name cannot be empty"}

        return _backend.request("GET", path)

    @mcp.tool(
        description=(
            "Get a stored map's gridmap as a PNG image "
            "(GET /api/v1/maps/{name}/image).\n"
            "Returns an image content block, so the client renders the map "
            "directly. Use get_map_info() for resolution/origin metadata. Fails "
            "with 404 when the map has a point cloud but no gridmap yet "
            "(grid is null in list_maps()).\n"
            "Example:\nget_map_image('factory')"
        ),
        annotations=ToolAnnotations(title="Get Map Image", readOnlyHint=True),
    )
    def get_map_image(map_name: str):
        """
        Get a stored map's gridmap rendered as a PNG.

        Args:
            map_name (str): The map's directory name (as listed by list_maps).

        Returns:
            Image: the map PNG (rendered by the client) on success, or
                {'error': ...} if the map has no gridmap / on failure.
        """
        path = _map_path(map_name)
        if not path:
            return {"error": "map_name cannot be empty"}

        # request_bytes, not request: the endpoint answers with image/png bytes
        # and no JSON envelope, so the shared JSON client cannot read it.
        body = _backend.request_bytes("GET", f"{path}/image")
        if isinstance(body, dict):
            return body

        return Image(data=body, format="png")

    @mcp.tool(
        description=(
            "Create one or more vertices on a map "
            "(POST /api/v1/maps/{map_name}/vertices).\n"
            "The map is named by map_name; each vertex is a dict with exactly "
            "these fields:\n"
            "  name (str), type (one of GENERAL/ARTIFACT/CHARGER/HOME/WAITING),\n"
            "  x (float, m), y (float, m), theta (float, degrees).\n"
            "Do NOT put map_name inside a vertex — the owning map comes from "
            "the map_name argument only.\n"
            "Example:\n"
            "create_map_vertices(map_name='factory', vertices=[{'name': 'dock-A', "
            "'type': 'CHARGER', 'x': 1.0, 'y': 2.0, 'theta': 90.0}])"
        ),
        annotations=ToolAnnotations(title="Create Map Vertices"),
    )
    def create_map_vertices(map_name: str, vertices: list = None) -> dict:
        """
        Create one or more vertices on a map.

        Args:
            map_name (str): The owning map's directory name.
            vertices (list): Non-empty list of vertex dicts, each with 'name',
                'type', 'x', 'y', and 'theta' (and nothing else).

        Returns:
            dict: {'vertices': [MapVertexResponse, ...]} on success, or
                {'error': ...} on validation/transport failure.
        """
        path = _map_path(map_name)
        if not path:
            return {"error": "map_name cannot be empty"}

        if not vertices:
            return {"error": "vertices cannot be empty; provide at least one vertex"}

        if not isinstance(vertices, list):
            return {"error": "vertices must be a list of vertex objects"}

        for i, vertex in enumerate(vertices):
            if not isinstance(vertex, dict):
                return {"error": f"vertices[{i}] must be an object"}

            # Reject rather than strip: a map_name that disagrees with the path
            # is a real mistake, and one that agrees is still a field the
            # backend does not accept. Same for any other unknown key.
            unknown = sorted(set(vertex) - set(_VERTEX_FIELDS))
            if unknown:
                return {
                    "error": f"vertices[{i}] has unsupported field(s) "
                    f"{', '.join(unknown)}; the owning map is the map_name "
                    f"argument, and each vertex takes only "
                    f"{', '.join(_VERTEX_FIELDS)}"
                }

            missing = [key for key in _VERTEX_FIELDS if key not in vertex]
            if missing:
                return {
                    "error": f"vertices[{i}] is missing field(s) {', '.join(missing)}"
                }

            vtype = vertex.get("type")
            if vtype not in _VERTEX_TYPES:
                error = _type_error(vtype)
                error["error"] = f"vertices[{i}].{error['error']}"
                return error

        body = _backend.request("POST", f"{path}/vertices", json=vertices)
        if isinstance(body, dict) and "error" in body:
            return body
        return {"vertices": body}

    @mcp.tool(
        description=(
            "List the vertices of a map, optionally filtered by type "
            "(GET /api/v1/maps/{map_name}/vertices).\n"
            "type, when given, must be one of "
            "GENERAL/ARTIFACT/CHARGER/HOME/WAITING. An unknown map is a 404, "
            "not an empty list.\n"
            "Example:\n"
            "list_map_vertices('factory')\n"
            "list_map_vertices('factory', type='CHARGER')"
        ),
        annotations=ToolAnnotations(title="List Map Vertices", readOnlyHint=True),
    )
    def list_map_vertices(map_name: str, type: str = None) -> dict:
        """
        List a map's vertices, optionally filtered by semantic role.

        Args:
            map_name (str): The owning map's directory name.
            type (str): Only return vertices of this semantic role.

        Returns:
            dict: {'vertices': [MapVertexResponse, ...]} on success, or
                {'error': ...} on failure.
        """
        path = _map_path(map_name)
        if not path:
            return {"error": "map_name cannot be empty"}

        if type is not None and type not in _VERTEX_TYPES:
            return _type_error(type)

        params = {"type": type} if type else None

        body = _backend.request("GET", f"{path}/vertices", params=params)
        if isinstance(body, dict) and "error" in body:
            return body
        return {"vertices": body}

    @mcp.tool(
        description=(
            "Get a single vertex of a map by its id "
            "(GET /api/v1/maps/{map_name}/vertices/{id}).\n"
            "A vertex that exists but belongs to a different map is a 404 from "
            "this map's URL.\n"
            "Example:\n"
            "get_map_vertex('factory', '3fa85f64-5717-4562-b3fc-2c963f66afa6')"
        ),
        annotations=ToolAnnotations(title="Get Map Vertex", readOnlyHint=True),
    )
    def get_map_vertex(map_name: str, vertex_id: str) -> dict:
        """
        Get a single map vertex.

        Args:
            map_name (str): The owning map's directory name.
            vertex_id (str): The vertex UUID.

        Returns:
            dict: The backend MapVertexResponse on success, or {'error': ...}
                on failure (e.g. not found).
        """
        path = _map_path(map_name)
        if not path:
            return {"error": "map_name cannot be empty"}

        vertex_id = (vertex_id or "").strip()
        if not vertex_id:
            return {"error": "vertex_id cannot be empty"}

        return _backend.request("GET", f"{path}/vertices/{vertex_id}")

    @mcp.tool(
        description=(
            "Update fields of an existing map vertex "
            "(PUT /api/v1/maps/{map_name}/vertices/{id}).\n"
            "Only the fields you pass are changed. Updatable fields: name (str), "
            "type (GENERAL/ARTIFACT/CHARGER/HOME/WAITING), x (float, m), "
            "y (float, m), theta (float, degrees). A vertex cannot be moved to "
            "another map here — delete it and create it on the other map.\n"
            "Example:\n"
            "update_map_vertex('factory', '3fa85f64-...', name='dock-B', theta=180.0)"
        ),
        annotations=ToolAnnotations(title="Update Map Vertex"),
    )
    def update_map_vertex(
        map_name: str,
        vertex_id: str,
        name: str = None,
        type: str = None,
        x: float = None,
        y: float = None,
        theta: float = None,
    ) -> dict:
        """
        Update an existing map vertex. Only provided fields are changed.

        Args:
            map_name (str): The owning map's directory name.
            vertex_id (str): The vertex UUID.
            name (str): New vertex name.
            type (str): New semantic role (one of the VertexType values).
            x (float): New world x-coordinate (metres).
            y (float): New world y-coordinate (metres).
            theta (float): New yaw angle in degrees.

        Returns:
            dict: The updated MapVertexResponse on success, or {'error': ...}
                on failure.
        """
        path = _map_path(map_name)
        if not path:
            return {"error": "map_name cannot be empty"}

        vertex_id = (vertex_id or "").strip()
        if not vertex_id:
            return {"error": "vertex_id cannot be empty"}

        if type is not None and type not in _VERTEX_TYPES:
            return _type_error(type)

        # Send only the fields the caller actually provided, so omitted fields
        # keep their current values (the backend uses exclude_unset semantics).
        changes = {
            key: value
            for key, value in (
                ("name", name),
                ("type", type),
                ("x", x),
                ("y", y),
                ("theta", theta),
            )
            if value is not None
        }
        if not changes:
            return {"error": "provide at least one field to update"}

        return _backend.request("PUT", f"{path}/vertices/{vertex_id}", json=changes)

    @mcp.tool(
        description=(
            "Delete a vertex of a map by its id "
            "(DELETE /api/v1/maps/{map_name}/vertices/{id}).\n"
            "Example:\n"
            "delete_map_vertex('factory', '3fa85f64-5717-4562-b3fc-2c963f66afa6')"
        ),
        annotations=ToolAnnotations(title="Delete Map Vertex", destructiveHint=True),
    )
    def delete_map_vertex(map_name: str, vertex_id: str) -> dict:
        """
        Delete a map vertex.

        Args:
            map_name (str): The owning map's directory name.
            vertex_id (str): The vertex UUID.

        Returns:
            dict: The backend DeleteResponse ({'message': ...}) on success, or
                {'error': ...} on failure.
        """
        path = _map_path(map_name)
        if not path:
            return {"error": "map_name cannot be empty"}

        vertex_id = (vertex_id or "").strip()
        if not vertex_id:
            return {"error": "vertex_id cannot be empty"}

        return _backend.request("DELETE", f"{path}/vertices/{vertex_id}")
