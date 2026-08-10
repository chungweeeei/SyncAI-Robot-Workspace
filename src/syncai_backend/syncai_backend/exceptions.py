class NotFoundError(Exception):
    pass


class UnauthorizedError(Exception):
    pass


class BadRequestError(Exception):
    pass


class ConflictError(Exception):
    pass


# Maps to 502 Bad Gateway. Named for what it means — a downstream dependency
# (Temporal, a ROS service) failed us — not for the status family; the old
# name, InternalServerError, read as "this maps to 500" and it never did.
class UpstreamError(Exception):
    pass
