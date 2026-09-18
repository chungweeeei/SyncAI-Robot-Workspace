// How a bag's four numbers are read. Shared by the recorder panel and the
// catalogue list so the same quantity never appears in two spellings on one
// screen — the reason these are here rather than local to each component the
// way map-card.tsx keeps its own.

/**
 * `4:12`, or `1:04:12` once a run passes the hour.
 *
 * Clock notation rather than "252 s": this is a duration a person compares
 * against how long they remember driving, and the hour field appears only when
 * there is one so short runs are not padded with a zero that means nothing.
 */
export function formatDuration(seconds: number): string {
  const total = Math.max(0, Math.floor(seconds));
  const s = total % 60;
  const m = Math.floor(total / 60) % 60;
  const h = Math.floor(total / 3600);

  const mm = h > 0 ? String(m).padStart(2, "0") : String(m);
  return `${h > 0 ? `${h}:` : ""}${mm}:${String(s).padStart(2, "0")}`;
}

/**
 * KiB / MiB / GiB, and the tier matters here in a way it does not for maps.
 *
 * A map is always tens of megabytes, so map-card.tsx can print MiB and stop. A
 * bag spans four orders of magnitude — a `robot_state` capture is kilobytes, a
 * lidar run is gigabytes — and the first seconds of any recording are spent
 * below a megabyte, where a fixed MiB tier reads `0 MiB` and looks broken at
 * exactly the moment the operator is checking that it started.
 */
export function formatSize(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`;
  const kib = bytes / 1024;
  if (kib < 1024) return `${kib.toFixed(0)} KiB`;
  const mib = kib / 1024;
  if (mib < 1024) return `${mib.toFixed(mib < 10 ? 1 : 0)} MiB`;
  return `${(mib / 1024).toFixed(1)} GiB`;
}

/**
 * `2026-09-18 09:22`, sliced out of the ISO string rather than run through
 * toLocaleString — the same hydration-safety trade map-card.tsx records: this
 * is a client component Next still prerenders, and a server/browser timezone
 * difference would be a mismatch. UTC for everyone is the honest answer, and it
 * is also what the bag directory's own timestamps are in.
 */
export function formatTimestamp(iso: string): string {
  return iso.slice(0, 16).replace("T", " ");
}

/**
 * `1 585` — thin spaces every three digits.
 *
 * A message count is read for its order of magnitude ("did it record anything
 * at all"), and seven unbroken digits in a tabular readout do not give that up
 * at a glance. `Intl.NumberFormat` is avoided deliberately: its grouping
 * depends on the runtime's locale, which is the prerender hazard above again.
 */
export function formatCount(count: number): string {
  return String(count).replace(/\B(?=(\d{3})+(?!\d))/g, " ");
}
