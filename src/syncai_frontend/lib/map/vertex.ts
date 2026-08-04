// The vertex-type vocabulary, shared by the canvas that draws vertices and the
// panel that edits them.
//
// It lives here rather than in either consumer because the two must not drift:
// the glyph on the marker is the only thing telling an operator what a dot on
// the map *is*, and it has to be the same letter the panel's type selector shows.

import type { VertexType } from "@/lib/types/map";

export interface VertexTypeSpec {
  value: VertexType;
  /** Panel label. Condensed caps in the UI, so short. */
  label: string;
  /**
   * One character, drawn beside the marker.
   *
   * Deliberately not a colour per type. `tone` in components/console/instrument
   * is an EFIS semantic — live is a measured value, warn is a fault — and
   * spending five signal hues on five vertex roles would make a WAITING stop
   * render in the same red the console uses for MAINTENANCE. Every stored vertex
   * is drawn in one marker hue; the commanded hue is reserved for the one the
   * operator is placing, exactly as it is for the brush ring.
   */
  glyph: string;
  /** One line in the panel, so the roles are not guessed from the name. */
  hint: string;
}

/** Order is the panel's order; GENERAL first because it is the default. */
export const VERTEX_TYPES: readonly VertexTypeSpec[] = [
  { value: "GENERAL", label: "Gen", glyph: "G", hint: "A plain navigation stop." },
  { value: "ARTIFACT", label: "Art", glyph: "A", hint: "An IoT / conveyor station." },
  { value: "CHARGER", label: "Chg", glyph: "C", hint: "A charging dock." },
  { value: "HOME", label: "Home", glyph: "H", hint: "The idle / park base." },
  { value: "WAITING", label: "Wait", glyph: "W", hint: "A hold spot for queueing." },
];

export const DEFAULT_VERTEX_TYPE: VertexType = "GENERAL";

const GLYPHS: Record<VertexType, string> = VERTEX_TYPES.reduce(
  (all, spec) => ({ ...all, [spec.value]: spec.glyph }),
  {} as Record<VertexType, string>,
);

export function vertexGlyph(type: VertexType): string {
  // The fallback covers a row written before a type was retired from the enum;
  // the list endpoint returns whatever the column holds.
  return GLYPHS[type] ?? "?";
}
