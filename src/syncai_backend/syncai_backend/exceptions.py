class NotFoundError(Exception):
    pass


class UnauthorizedError(Exception):
    pass


class BadRequestError(Exception):
    pass


class InternalServerError(Exception):
    pass


class RobotGatewayError(Exception):
    """Raised when the robot gateway fails to talk to the robot
    (gRPC / WebSocket / Unix socket) or when the response cannot be parsed.

    Callers should treat this as an upstream failure (HTTP 502 Bad Gateway).
    """


class RepoInternalError(Exception):
    pass
