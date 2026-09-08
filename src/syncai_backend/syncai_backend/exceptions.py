class NotFoundError(Exception):
    pass


class UnauthorizedError(Exception):
    pass


class BadRequestError(Exception):
    pass


class ConflictError(Exception):
    """A request that contradicts current state (HTTP 409).

    ``code`` is an optional stable, machine-readable discriminator for callers
    that must tell two 409s on the same route apart — the re-convert endpoint
    raises both "a conversion is already running" and "this grid holds hand
    edits", and the frontend confirms-and-retries only the latter. Matching on
    the human-readable detail instead would couple the UI to prose that exists
    to be reworded. Optional so every pre-existing raise stays valid.
    """

    def __init__(self, message: str, code: "str | None" = None):
        super().__init__(message)
        self.code = code


# Maps to 502 Bad Gateway. Named for what it means — a downstream dependency
# (Temporal, a ROS service) failed us — not for the status family; the old
# name, InternalServerError, read as "this maps to 500" and it never did.
class UpstreamError(Exception):
    pass
