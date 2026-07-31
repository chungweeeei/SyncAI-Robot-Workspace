import { apiUrl, wsUrl } from "@/lib/api/config";
import type { PointCloudFrame, StreamStatus } from "@/lib/types/pointcloud";

/**
 * Decode a binary point-cloud frame: little-endian uint32 count + count*3
 * float32 xyz. The 4-byte header keeps the float payload 4-byte aligned so it
 * can be viewed without copying.
 */
export function decodePointCloud(buffer: ArrayBuffer): PointCloudFrame {
  const count = new DataView(buffer).getUint32(0, true);
  const positions = new Float32Array(buffer, 4, count * 3);
  return { count, positions };
}

export interface PointCloudStreamHandlers {
  onFrame: (frame: PointCloudFrame) => void;
  onStatus?: (status: StreamStatus) => void;
}

export interface PointCloudStream {
  close: () => void;
}

/**
 * Connect to the live body_cloud WebSocket and invoke ``onFrame`` for each
 * decoded frame. Reconnects automatically with a fixed backoff until closed.
 */
export function createPointCloudStream(
  handlers: PointCloudStreamHandlers,
  path = "/api/v1/robot/pointcloud/stream",
): PointCloudStream {
  let ws: WebSocket | null = null;
  let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  let closed = false;

  const connect = () => {
    if (closed) return;
    handlers.onStatus?.("connecting");
    ws = new WebSocket(wsUrl(path));
    ws.binaryType = "arraybuffer";

    ws.onopen = () => handlers.onStatus?.("open");
    ws.onmessage = (ev) => {
      if (ev.data instanceof ArrayBuffer) {
        handlers.onFrame(decodePointCloud(ev.data));
      }
    };
    ws.onerror = () => handlers.onStatus?.("error");
    ws.onclose = () => {
      handlers.onStatus?.("closed");
      if (!closed) {
        reconnectTimer = setTimeout(connect, 2000);
      }
    };
  };

  connect();

  return {
    close: () => {
      closed = true;
      if (reconnectTimer) clearTimeout(reconnectTimer);
      ws?.close();
    },
  };
}

/**
 * Fetch one stored map's cloud once.
 *
 * Read from that map's saved `map.pcd` and voxel-downsampled server-side with
 * the same numbers the live body_cloud path uses, so the two overlay. It used
 * to come from the localizer/map_cloud topic and therefore needed no name —
 * which also meant it could only ever answer for the loaded map. Callers pass
 * the active map's name now (see useActiveMap).
 *
 * Only called when the user enables the "map cloud" layer, so weak clients
 * never pay for the 100k+ point download.
 */
export async function fetchMapPointCloud(
  name: string,
  opts: { signal?: AbortSignal } = {},
): Promise<PointCloudFrame> {
  const path = `/api/v1/maps/${encodeURIComponent(name)}/pointcloud`;
  const res = await fetch(apiUrl(path), { signal: opts.signal });
  if (!res.ok) {
    throw new Error(`map pointcloud fetch failed: ${res.status}`);
  }
  return decodePointCloud(await res.arrayBuffer());
}
