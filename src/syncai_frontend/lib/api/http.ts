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
