// Shared plumbing for the backend REST clients in this directory.

/** FastAPI reports errors as {detail: string} (or a 422 validation array). */
export async function errorDetail(res: Response): Promise<string> {
  try {
    const body = await res.json();
    if (typeof body?.detail === "string") return body.detail;
    if (body?.detail) return JSON.stringify(body.detail);
  } catch {
    /* non-JSON body (proxy error page); fall back to the status line */
  }
  return `${res.status} ${res.statusText}`;
}

/**
 * A JSON round trip against the backend, throwing the backend's own sentence.
 *
 * The domain-exception handlers answer with `{detail}`, and those strings are
 * written for an operator to read ("Map vertex <id> was not found in 'dp2f'.").
 * Every caller here renders `error.message` verbatim, so unwrapping `detail` is
 * what puts the actionable half on screen instead of a status code.
 *
 * `Content-Type` is sent unconditionally rather than only when there is a body:
 * FastAPI ignores it on a GET/DELETE, and making it conditional is one more
 * branch for a header that costs nothing.
 *
 * Pass `T = void` for an endpoint with no body (204, or a DELETE whose envelope
 * nothing reads) — the response is not parsed in that case.
 */
export async function requestJson<T>(
  url: string,
  init?: RequestInit & { parse?: boolean },
): Promise<T> {
  const { parse = true, ...rest } = init ?? {};
  const res = await fetch(url, {
    ...rest,
    headers: { "Content-Type": "application/json", ...rest.headers },
  });

  if (!res.ok) throw new Error(await errorDetail(res));

  return (parse ? await res.json() : undefined) as T;
}
