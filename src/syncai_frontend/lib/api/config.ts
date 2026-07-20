// Backend base URLs. The FastAPI backend listens on :3000; the frontend is
// served from :3001 (or dev :3000-of-next), so the browser must target the
// backend host explicitly.
//
// Resolution order:
//   1. NEXT_PUBLIC_API_BASE / NEXT_PUBLIC_WS_BASE (explicit override)
//   2. the page's own hostname on port 3000 (works when the dashboard is
//      opened at http://<robot-ip>:3001 on the LAN)
//   3. http://localhost:3000 (SSR / build-time fallback)

const BACKEND_PORT = 3000;

function resolveHttpBase(): string {
  const override = process.env.NEXT_PUBLIC_API_BASE;
  if (override) return override.replace(/\/$/, "");
  if (typeof window !== "undefined") {
    return `${window.location.protocol}//${window.location.hostname}:${BACKEND_PORT}`;
  }
  return `http://localhost:${BACKEND_PORT}`;
}

function resolveWsBase(): string {
  const override = process.env.NEXT_PUBLIC_WS_BASE;
  if (override) return override.replace(/\/$/, "");
  const http = resolveHttpBase();
  return http.replace(/^http/, "ws");
}

export function apiUrl(path: string): string {
  return `${resolveHttpBase()}${path.startsWith("/") ? path : `/${path}`}`;
}

export function wsUrl(path: string): string {
  return `${resolveWsBase()}${path.startsWith("/") ? path : `/${path}`}`;
}
