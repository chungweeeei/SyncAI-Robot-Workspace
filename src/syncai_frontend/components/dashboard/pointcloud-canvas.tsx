"use client";

import * as React from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { MeshoptDecoder } from "three/examples/jsm/libs/meshopt_decoder.module.js";
import { useTheme } from "next-themes";

import { cn } from "@/lib/utils";
import { G23_JOINTS } from "@/lib/robot/g23-joints";
import { vertexGlyph } from "@/lib/map/vertex";
import type { MapVertex } from "@/lib/types/map";
import type { MapMetadata, PlanarPose, RobotPose } from "@/lib/types/robot";
import type { PointCloudFrame } from "@/lib/types/pointcloud";
import type { StreamStatus } from "@/lib/types/stream";
import {
  createPointCloudStream,
  fetchMapPointCloud,
} from "@/lib/ros/pointcloud-stream";

// Upper bound on live points held in the GPU buffer. The backend caps frames
// (default 30k) well below this; the slack absorbs config changes without a
// reallocation.
const MAX_LIVE_POINTS = 200_000;

// Fixed height band (metres, map frame) used to colour points by z. A fixed
// range keeps colours stable frame-to-frame instead of flickering with the
// per-frame min/max.
const Z_MIN = -0.5;
const Z_MAX = 3.0;

// The robot itself is deliberately absent here: it renders in its own material
// so the machine looks like the same machine in either theme.
interface Theme {
  background: number;
  ground: number;
  groundOpacity: number;
  /** Solid colour for the static map cloud, kept distinct from the
   *  height-coloured body cloud. White on the dark background; a dark grey on
   *  the near-white light background so it stays visible in both themes. */
  mapCloud: number;
  /** Committed / being-dragged goal marker. `signal-cmd` cyan from globals.css:
   *  a goal is a commanded value, and it is the same cyan in the goal readback
   *  and on the Send button. */
  goal: number;
  goalDraft: number;
  /** Staged initial-pose marker. `signal-caution` amber, matching its control:
   *  it asserts where the robot *is*, not where it should go, and the two must
   *  never be misread for each other on the floor. */
  initialPose: number;
  initialPoseDraft: number;
  /**
   * Stored map vertices. One hue for all five types, deliberately not a signal
   * colour: see lib/map/vertex.ts on why the type is a glyph.
   *
   * Both themes get a *dark* hue, which is where this parts company with the
   * gridmap editor's `PALETTES.vertex` (components/maps/grid-canvas.tsx) — the
   * two agree on the family, not on the value. The editor can flip to a light
   * marker in night mode because it draws its own halo behind every mark; here
   * the marker lies on a ground plane textured with the occupancy grid, which is
   * white free space in *either* theme (lib/map/render.ts blits the bytes
   * literally). The editor's dark-theme hue put a light mark on that white
   * floor, which is how a stop became something you had to go looking for.
   *
   * The dark value is still mid-toned rather than ink, because a map that has
   * not been converted to a gridmap has no ground plane at all and the mark
   * falls back onto the near-black background.
   */
  vertex: number;
  /**
   * The vertex under the pointer. The commanded hue, because that stop is one
   * double-click away from becoming the commanded pose — and because the gridmap
   * editor already lights its selected vertex in `palette.cmd`.
   */
  vertexHover: number;
}

// Scene colours track the console surfaces so the viewport reads as a recessed
// well in the panel rather than a pasted-in canvas. Keep these in sync with the
// --background / --elevated / --signal-cmd values in globals.css.
const THEMES: Record<"light" | "dark", Theme> = {
  light: {
    background: 0xe9eef2,
    ground: 0xffffff,
    groundOpacity: 0.85,
    mapCloud: 0x54646f,
    goal: 0x0a6d94,
    goalDraft: 0x4aa6c6,
    initialPose: 0x93600e,
    initialPoseDraft: 0xc08c33,
    vertex: 0x173845,
    vertexHover: 0x0a6d94,
  },
  dark: {
    background: 0x0b1014,
    ground: 0x1b252d,
    groundOpacity: 0.6,
    mapCloud: 0xa7b6c1,
    goal: 0x45c8f0,
    goalDraft: 0x8adcf7,
    initialPose: 0xf0b23c,
    initialPoseDraft: 0xf6cd7e,
    vertex: 0x2b5f77,
    vertexHover: 0x45c8f0,
  },
};

// Point sizes (metres — PointsMaterial keeps sizeAttenuation on, so these are
// world units that shrink with distance, not screen pixels). The live body
// cloud is drawn small and fine: at ~2.5k points per scan, sprites large enough
// to see individually also merge into blobs that hide the structure of what the
// lidar actually saw. The map cloud stays coarser — it is decimated to a 0.3 m
// voxel, so drawing it finer than that only makes it look sparse.
const LIVE_POINT_SIZE = 0.05;
const MAP_POINT_SIZE = 0.14;

// G23 model baked from description/G23.urdf by scripts/urdf2glb.py. It keeps
// the ROS convention (Z-up, +x forward, metres) rather than glTF's nominal
// +Y-up, which is exactly what the Z-up world below expects — so it needs no
// correction rotation. See that script's docstring.
const ROBOT_MODEL_URL = "/models/g23.glb";

// Height of base_link above the ground with the legs at the rest pose the GLB
// is baked in: 0.41012 m of link offsets down to FL_FOOT, plus the 22 mm foot
// collision sphere the URDF uses as the contact point. The pose feed reports a
// planar pose (lio_bridge projects to x/y/yaw, so z is ~0), so without this the
// robot renders buried to its knees.
const ROBOT_BASE_HEIGHT_M = 0.43212;

// Time constant (seconds) of the easing applied to the reported pose. The
// dashboard's feed is now the ~20 Hz telemetry WebSocket, so the filter's job
// shrank from hiding a 1 Hz snapshot cadence (tau 0.25 then) to bridging the
// 50 ms gaps between frames — 0.1 s does that while cutting the lag the old
// value would now just waste. It is deliberately a filter rather than a
// replay buffer: smoothing costs a fraction of a second of lag but never
// renders a pose the robot has already left behind, which a buffer would.
// /model-preview still ticks its fake pose at 500 ms, where the robot eases
// between updates a touch more stiffly than before — acceptable for a dev
// tool.
const POSE_EASE_TAU_S = 0.1;

/** Pose the robot is actually drawn at, eased toward the reported one. */
interface SmoothPose {
  x: number;
  y: number;
  z: number;
  /**
   * Radians, and deliberately *not* wrapped to [-π, π]: each new target is
   * unwrapped against this value so easing always takes the short way round.
   */
  yaw: number;
}

/**
 * Load the robot model once per page load.
 *
 * The scene-setup effect below tears down and rebuilds the renderer whenever
 * the map or the theme changes, and refetching plus reparsing a ~600 kB GLB on
 * every theme toggle is pure waste. Caching the promise at module scope is the
 * same move the body_cloud WebSocket already makes for the same reason.
 *
 * The GLB carries geometry only — no materials, and no vertex normals (STL has
 * none to carry over, and generating them would inflate the asset for a
 * mechanical part that reads fine flat-shaded). Left alone, glTF's default
 * material is fully metallic and would render pure black in this deliberately
 * unlit scene, so every mesh gets our own lit material here.
 */
let robotModelPromise: Promise<THREE.Object3D> | null = null;

function loadRobotModel(): Promise<THREE.Object3D> {
  if (!robotModelPromise) {
    const loader = new GLTFLoader();
    // scripts/urdf2glb.py runs gltfpack -cc, whose output declares
    // EXT_meshopt_compression. (KHR_mesh_quantization needs no registration.)
    loader.setMeshoptDecoder(MeshoptDecoder);
    robotModelPromise = loader.loadAsync(ROBOT_MODEL_URL).then((gltf) => {
      const material = new THREE.MeshStandardMaterial({
        color: 0xb0b4ba,
        metalness: 0.1,
        roughness: 0.75,
        // No NORMAL attribute in the asset, so shade off face derivatives.
        flatShading: true,
      });
      gltf.scene.traverse((obj) => {
        const mesh = obj as THREE.Mesh;
        if (mesh.isMesh) mesh.material = material;
      });
      return gltf.scene;
    });
  }
  return robotModelPromise;
}

/**
 * Joint names seen in a `joints` prop that G23_JOINTS does not know. Warned
 * once each (module scope, like the model cache): a name mismatch between the
 * telemetry source and the URDF would otherwise fail silently as a leg that
 * never moves — and kJointNames in syncai_driver_manager.cpp still carries a
 * TODO about its ordering, so a mismatch is a live possibility.
 */
const warnedUnknownJoints = new Set<string>();

/** Map a height to an RGB colour (blue = low, red = high) via an HSL sweep. */
function heightColor(z: number, out: THREE.Color): THREE.Color {
  const t = Math.min(1, Math.max(0, (z - Z_MIN) / (Z_MAX - Z_MIN)));
  // hue 240deg (blue) -> 0deg (red)
  return out.setHSL(((1 - t) * 240) / 360, 0.9, 0.55);
}

// Pose marker (goal / initial pose), in metres. The marker sits on the ground in
// a perspective view, so it has to be a real object of roughly robot size or it
// stops reading as a place on the floor — a screen-space arrow of fixed pixel
// length would grow into the horizon. Lifted off z=0 to keep it out of a
// z-fight with the ground.
const MARKER_RING_INNER_M = 0.26;
const MARKER_RING_OUTER_M = 0.34;
const MARKER_SHAFT_LEN_M = 0.5;
const MARKER_SHAFT_RADIUS_M = 0.035;
const MARKER_HEAD_LEN_M = 0.26;
const MARKER_HEAD_RADIUS_M = 0.1;
const MARKER_Z_M = 0.05;

/** Drag distance (CSS px) below which the heading is not taken from the drag. */
const HEADING_DEADZONE_PX = 8;

// Height (metres) the robot model floats at while it is being carried on the
// pointer, before a press plants it on the floor. Purely an affordance: held and
// placed have to look different, or a carried robot reads as a pose that is
// already set. Small enough that the shadowless model still lines up with the
// marker ring below it, which stays on the floor throughout.
const CARRY_LIFT_M = 0.35;

/**
 * The z=0 map plane the pointer is projected onto to pick a goal.
 *
 * Deliberately the mathematical plane rather than a raycast against the ground
 * *mesh*: the mesh only spans the occupancy grid (and does not exist at all
 * without a 2D map), while goal mode has to work anywhere the operator can see
 * floor. Bounds are then checked separately against the map extent.
 */
const GROUND_PLANE = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);

/**
 * Ring + arrow marking a pose on the floor, built pointing down +x so the
 * group's rotation.z is the heading. Unlit (MeshBasicMaterial) like everything
 * else in the scene except the robot itself, so it keeps its colour whichever
 * way it faces.
 *
 * One material for all three meshes, which is also what lets `setMarkerColor`
 * recolour a marker in place — the draft marker changes hue with the pick mode
 * and must not force a scene rebuild to do it.
 */
function createPoseMarker(color: number, opacity: number): THREE.Group {
  const material = new THREE.MeshBasicMaterial({
    color,
    transparent: opacity < 1,
    opacity,
    side: THREE.DoubleSide,
  });

  const group = new THREE.Group();

  // RingGeometry is already in the XY plane, i.e. flat on this z-up world.
  group.add(
    new THREE.Mesh(
      new THREE.RingGeometry(MARKER_RING_INNER_M, MARKER_RING_OUTER_M, 32),
      material,
    ),
  );

  // Cylinder / cone run along +y by default; -90deg about z aims them down +x.
  const shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(
      MARKER_SHAFT_RADIUS_M,
      MARKER_SHAFT_RADIUS_M,
      MARKER_SHAFT_LEN_M,
      12,
    ),
    material,
  );
  shaft.rotation.z = -Math.PI / 2;
  shaft.position.x = MARKER_RING_OUTER_M + MARKER_SHAFT_LEN_M / 2;
  group.add(shaft);

  const head = new THREE.Mesh(
    new THREE.ConeGeometry(MARKER_HEAD_RADIUS_M, MARKER_HEAD_LEN_M, 16),
    material,
  );
  head.rotation.z = -Math.PI / 2;
  head.position.x =
    MARKER_RING_OUTER_M + MARKER_SHAFT_LEN_M + MARKER_HEAD_LEN_M / 2;
  group.add(head);

  group.visible = false;
  return group;
}

/** Move a marker to a pose (theta in degrees), or hide it when there is none. */
function placePoseMarker(marker: THREE.Group, pose: PlanarPose | null) {
  marker.visible = pose !== null;
  if (!pose) return;
  marker.position.set(pose.x, pose.y, MARKER_Z_M);
  marker.rotation.z = (pose.theta * Math.PI) / 180;
}

/** Recolour a marker built by `createPoseMarker` (shared material). */
function setMarkerColor(marker: THREE.Group, color: number) {
  const mesh = marker.children[0] as THREE.Mesh;
  (mesh.material as THREE.MeshBasicMaterial).color.setHex(color);
}

/*
 * Vertex markers, in metres.
 *
 * Deliberately *not* the ring-and-arrow of createPoseMarker at a smaller scale.
 * A map carries a dozen or more stored stops and only ever one goal, and a dozen
 * arrows crossing each other at floor level is noise the operator has to read
 * past to find the marker they are acting on. A stop is instead a flat target on
 * the floor — a translucent disc, a crisp ring, and a chevron for the heading —
 * so the whole layer reads as ground marking rather than as instruments.
 *
 * The chevron is what carries the heading, and it is short: a stop's heading is
 * worth knowing but never worth as much screen as the goal's, which is drawn as
 * a full arrow because it is the pose being commanded right now.
 *
 * The mark sits lower than MARKER_Z_M so a goal placed on a stop draws over it
 * rather than z-fighting with it.
 */
const VERTEX_DISC_RADIUS_M = 0.2;
// A 5 cm band rather than 3: the ring is what locates the stop from across a
// warehouse, and a hairline ring is the first thing to alias away at distance.
const VERTEX_RING_INNER_M = 0.2;
const VERTEX_RING_OUTER_M = 0.25;
const VERTEX_CHEVRON_BASE_M = 0.31;
const VERTEX_CHEVRON_TIP_M = 0.46;
const VERTEX_CHEVRON_HALF_M = 0.1;
const VERTEX_Z_M = 0.02;

/*
 * The floor mark alone disappears the moment the camera drops toward the
 * horizon — which is most of the time, because the useful views of a robot are
 * from behind and low. A thin stem and a small badge above it give every stop a
 * vertical presence that survives a grazing camera, the same way a pin does on a
 * street map, without adding anything at floor level.
 *
 * The badge carries the type glyph and nothing else. The name lives in the
 * dialog a double-click opens: a caption per stop is the one thing that turns
 * this layer back into clutter, and the operator only needs a name at the moment
 * they are about to act on one.
 */
const VERTEX_STEM_HEIGHT_M = 0.6;
const VERTEX_STEM_RADIUS_M = 0.009;
const VERTEX_BADGE_SIZE_M = 0.22;
/** Badge scale-up on hover — the marker under the pointer has to answer back. */
const VERTEX_BADGE_HOVER_SCALE = 1.3;
/** Texture resolution of a badge, not its drawn size. */
const VERTEX_BADGE_TEXTURE_PX = 128;
/**
 * Radius of the invisible disc that catches the pointer. Comfortably wider than
 * the mark itself, for the same reason the gridmap editor's VERTEX_HIT_RADIUS is
 * wider than its dot: a stop is a point, and a point is hard to hit.
 */
const VERTEX_HIT_RADIUS_M = 0.42;

/** Materials for one vertex hue, shared by every marker drawn in it. */
interface VertexMaterials {
  /** The disc: present, but never competing with the cloud drawn over it. */
  fill: THREE.MeshBasicMaterial;
  /** Ring and chevron — the part that has to stay crisp at distance. */
  line: THREE.MeshBasicMaterial;
  stem: THREE.MeshBasicMaterial;
}

function createVertexMaterials(color: number): VertexMaterials {
  const make = (opacity: number) =>
    new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity,
      side: THREE.DoubleSide,
      // Ground marking, not geometry: writing depth would let one stop's
      // translucent disc erase the cloud behind the next one.
      depthWrite: false,
    });
  // The ring and chevron are drawn flat out: they are the mark. Only the disc
  // stays translucent, and even that is now solid enough to read as a target
  // through the live cloud rather than as a smudge under it.
  return { fill: make(0.35), line: make(1), stem: make(0.7) };
}

/** `0x2b5f77` → `"#2b5f77"`, for the canvas the badge is drawn on. */
function cssHex(color: number): string {
  return `#${color.toString(16).padStart(6, "0")}`;
}

/**
 * The type badge: a filled disc in the marker hue with the glyph knocked out in
 * near-white, plus a pale rim.
 *
 * The hue is baked in rather than left to `SpriteMaterial.color` tinting a white
 * texture. Tinting was cheaper — one texture served both the resting and hover
 * hues — but it forces the glyph to be a *darkened* version of the disc it sits
 * on, and once the disc hue went dark enough to be findable, dark-on-dark is
 * what that leaves. A light glyph on a solid dark disc is the contrast that
 * makes the badge readable at a glance, and it is the same figure/ground the
 * console's own chips use.
 *
 * Two textures per glyph then, resting and hover — ten in the worst case, all
 * 128 px, all built once per layer.
 */
function createBadgeTexture(glyph: string, fill: number): THREE.CanvasTexture {
  const size = VERTEX_BADGE_TEXTURE_PX;
  const canvas = document.createElement("canvas");
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext("2d");
  if (!ctx) throw new Error("2D canvas context unavailable");

  const centre = size / 2;
  ctx.fillStyle = cssHex(fill);
  ctx.beginPath();
  ctx.arc(centre, centre, centre - size * 0.08, 0, Math.PI * 2);
  ctx.fill();
  // A pale rim, so a dark badge keeps its edge against the near-black
  // background of a map with no gridmap under it.
  ctx.strokeStyle = "rgba(255, 255, 255, 0.55)";
  ctx.lineWidth = size * 0.05;
  ctx.stroke();

  ctx.fillStyle = "#f2f7fa";
  ctx.font = `700 ${size * 0.5}px ui-monospace, SFMono-Regular, Menlo, monospace`;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  // +2% down: the monospace cap sits high in the em box, and a badge whose
  // letter is off-centre is the kind of thing you see without being able to
  // name it.
  ctx.fillText(glyph, centre, centre + size * 0.02);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

/** Everything one vertex owns, so hover can recolour it without a rebuild. */
interface VertexHandle {
  fill: THREE.Mesh[];
  line: THREE.Mesh[];
  stem: THREE.Mesh[];
  badge: THREE.Sprite;
  /** Resting and hover faces of this stop's badge, swapped by `paint`. */
  badgeTextures: { base: THREE.CanvasTexture; hover: THREE.CanvasTexture };
  /** The mark itself, hidden as a whole while the stop is being re-placed. */
  marker: THREE.Group;
}

export interface VertexLayer {
  group: THREE.Group;
  /** What a pointer ray is tested against. */
  pickables: THREE.Object3D[];
  /** Resolve a `userData.vertexId` from a hit back to its row. */
  byId: Map<string, MapVertex>;
  /** Light up one marker, or none. Cheap enough to call per pointer move. */
  setHovered: (id: string | null) => void;
  /** Take one marker off the map while its pose is in the operator's hands. */
  setMoving: (id: string | null) => void;
  dispose: () => void;
}

/**
 * The whole stored-vertex layer as one group, plus the disposer for everything
 * it allocated.
 *
 * Built wholesale and thrown away on any change rather than diffed: the list
 * comes from a fetch-once-per-mount hook, so "any change" means a map swap or a
 * theme toggle, not a stream. Geometry, materials and the badge textures are all
 * shared across the layer; a vertex owns only its meshes, its sprite material
 * (which carries the tint hover changes) and its transforms.
 */
function createVertexLayer(
  vertices: MapVertex[],
  theme: Theme,
): VertexLayer {
  const group = new THREE.Group();
  const base = createVertexMaterials(theme.vertex);
  const hover = createVertexMaterials(theme.vertexHover);

  const discGeom = new THREE.CircleGeometry(VERTEX_DISC_RADIUS_M, 32);
  const ringGeom = new THREE.RingGeometry(
    VERTEX_RING_INNER_M,
    VERTEX_RING_OUTER_M,
    32,
  );
  // Both the chevron and the ring are already in the XY plane, i.e. flat on
  // this z-up world, and the chevron already points down +x — so the marker
  // group's rotation.z is the heading, exactly as in createPoseMarker.
  const chevron = new THREE.Shape();
  chevron.moveTo(VERTEX_CHEVRON_TIP_M, 0);
  chevron.lineTo(VERTEX_CHEVRON_BASE_M, VERTEX_CHEVRON_HALF_M);
  chevron.lineTo(VERTEX_CHEVRON_BASE_M, -VERTEX_CHEVRON_HALF_M);
  chevron.closePath();
  const chevronGeom = new THREE.ShapeGeometry(chevron);
  const stemGeom = new THREE.CylinderGeometry(
    VERTEX_STEM_RADIUS_M,
    VERTEX_STEM_RADIUS_M,
    VERTEX_STEM_HEIGHT_M,
    6,
  );
  const hitGeom = new THREE.CircleGeometry(VERTEX_HIT_RADIUS_M, 12);
  // The hit discs are `visible = false` (set per mesh below) and still picked:
  // three's Raycaster tests layers, never visibility, so a flagged-off mesh
  // costs no draw call and keeps catching rays. That is the whole job here.
  const hitMaterial = new THREE.MeshBasicMaterial({
    depthWrite: false,
    side: THREE.DoubleSide,
  });

  // Keyed by glyph *and* face, so five types cost at most ten textures however
  // many stops the map has.
  const badgeTextures = new Map<string, THREE.CanvasTexture>();
  const badgeTexture = (glyph: string, hovered: boolean) => {
    const key = `${hovered ? "h" : "b"}:${glyph}`;
    let texture = badgeTextures.get(key);
    if (!texture) {
      texture = createBadgeTexture(
        glyph,
        hovered ? theme.vertexHover : theme.vertex,
      );
      badgeTextures.set(key, texture);
    }
    return texture;
  };
  const handles = new Map<string, VertexHandle>();
  const pickables: THREE.Object3D[] = [];
  const byId = new Map<string, MapVertex>();

  for (const vertex of vertices) {
    byId.set(vertex.id, vertex);

    const marker = new THREE.Group();
    marker.position.set(vertex.x, vertex.y, VERTEX_Z_M);
    marker.rotation.z = (vertex.theta * Math.PI) / 180;

    const disc = new THREE.Mesh(discGeom, base.fill);
    const ring = new THREE.Mesh(ringGeom, base.line);
    const head = new THREE.Mesh(chevronGeom, base.line);

    const stem = new THREE.Mesh(stemGeom, base.stem);
    // The cylinder runs along +y by default; +90deg about x stands it up.
    stem.rotation.x = Math.PI / 2;
    stem.position.z = VERTEX_STEM_HEIGHT_M / 2;

    const hit = new THREE.Mesh(hitGeom, hitMaterial);
    hit.userData.vertexId = vertex.id;
    hit.visible = false;

    marker.add(disc, ring, head, stem, hit);
    group.add(marker);

    const glyph = vertexGlyph(vertex.type);
    const faces = {
      base: badgeTexture(glyph, false),
      hover: badgeTexture(glyph, true),
    };
    const badge = new THREE.Sprite(
      new THREE.SpriteMaterial({
        map: faces.base,
        transparent: true,
        depthWrite: false,
      }),
    );
    badge.scale.setScalar(VERTEX_BADGE_SIZE_M);
    // Hung off the layer rather than the marker: a sprite ignores rotation, so
    // parenting it under the heading transform would only hide that fact.
    // Lifted by a third of its own height so it caps the stem instead of
    // swallowing the top of it.
    badge.position.set(
      vertex.x,
      vertex.y,
      VERTEX_Z_M + VERTEX_STEM_HEIGHT_M + VERTEX_BADGE_SIZE_M / 3,
    );
    badge.userData.vertexId = vertex.id;
    group.add(badge);

    pickables.push(hit, badge);
    handles.set(vertex.id, {
      fill: [disc],
      line: [ring, head],
      stem: [stem],
      badge,
      badgeTextures: faces,
      marker,
    });
  }

  let hovered: string | null = null;

  const paint = (id: string | null, on: boolean) => {
    const handle = id ? handles.get(id) : undefined;
    if (!handle) return;
    const set = on ? hover : base;
    for (const mesh of handle.fill) mesh.material = set.fill;
    for (const mesh of handle.line) mesh.material = set.line;
    for (const mesh of handle.stem) mesh.material = set.stem;
    const material = handle.badge.material as THREE.SpriteMaterial;
    material.map = on ? handle.badgeTextures.hover : handle.badgeTextures.base;
    // Swapping the map is a program change, not a uniform change.
    material.needsUpdate = true;
    handle.badge.scale.setScalar(
      on ? VERTEX_BADGE_SIZE_M * VERTEX_BADGE_HOVER_SCALE : VERTEX_BADGE_SIZE_M,
    );
  };

  const setHovered = (id: string | null) => {
    // An id this layer does not know is treated as none: the caller's idea of
    // what is hovered outlives a rebuild, and a map swap can retire the stop it
    // names. Resolving it here (rather than trusting the id) is what keeps a
    // rebuilt layer from carrying a highlight nothing can clear.
    const next = id && handles.has(id) ? id : null;
    if (next === hovered) return;
    paint(hovered, false);
    paint(next, true);
    hovered = next;
  };

  let moving: string | null = null;

  const setMoving = (id: string | null) => {
    const next = id && handles.has(id) ? id : null;
    if (next === moving) return;
    for (const candidate of [moving, next]) {
      const handle = candidate ? handles.get(candidate) : undefined;
      if (!handle) continue;
      // Group visibility covers the whole mark; the badge is hung off the layer
      // rather than the marker, so it has to be told separately.
      const shown = candidate !== next;
      handle.marker.visible = shown;
      handle.badge.visible = shown;
    }
    moving = next;
  };

  const dispose = () => {
    for (const geometry of [
      discGeom,
      ringGeom,
      chevronGeom,
      stemGeom,
      hitGeom,
    ]) {
      geometry.dispose();
    }
    for (const set of [base, hover]) {
      set.fill.dispose();
      set.line.dispose();
      set.stem.dispose();
    }
    hitMaterial.dispose();
    // Textures are shared between the badges wearing the same glyph, so they are
    // freed here rather than per badge — and they have to be freed here at all:
    // the scene teardown's traverse reaches geometries and materials, never the
    // texture a material points at.
    for (const texture of badgeTextures.values()) texture.dispose();
    for (const handle of handles.values()) handle.badge.material.dispose();
  };

  return { group, pickables, byId, setHovered, setMoving, dispose };
}

/**
 * Wire the OrbitControls buttons for a camera mode.
 *  - "move":  left-drag pans (moves the view), right-drag orbits.
 *  - "focus": left-drag orbits around the locked target; panning is disabled
 *    so the target stays pinned to the robot.
 * Middle button always dollies (zoom).
 *
 * An armed pick mode overrides the left button entirely: a left-drag then has to
 * produce a pose, not move the camera. Mapping it to null (OrbitControls falls
 * through to its no-action default) rather than disabling the controls outright
 * keeps right-drag orbit and wheel zoom live, so the operator can still look
 * around while placing a pose.
 */
function applyCameraMode(
  controls: OrbitControls,
  mode: "move" | "focus",
  picking: boolean,
) {
  const orbit = mode === "focus";
  controls.enablePan = !orbit;
  controls.mouseButtons = {
    LEFT: picking ? null : orbit ? THREE.MOUSE.ROTATE : THREE.MOUSE.PAN,
    MIDDLE: THREE.MOUSE.DOLLY,
    RIGHT: THREE.MOUSE.ROTATE,
  };
}

// Fallback world framing when no 2D map is available (e.g. a raw body_cloud
// render test with no map_server running). Points arrive near the LIO odom
// origin, so a modest span centred on the origin frames them sensibly.
const DEFAULT_SPAN_M = 20;

/**
 * How far south of the target an overhead camera is parked, as a fraction of its
 * height.
 *
 * A camera placed *exactly* above its target in a z-up world has its up vector
 * parallel to its view direction, which is undefined for both the projection and
 * OrbitControls' spherical maths — the view snaps to an arbitrary azimuth and
 * the first orbit drag flips it. About a degree off vertical costs nothing that
 * reads as tilt and pins map +y to the top of the screen, which is the
 * orientation the gridmap editor and every site plan use.
 */
const TOP_DOWN_TILT = 0.02;

/** Slack around the map extent when framing it from overhead. */
const TOP_DOWN_MARGIN = 1.08;

/** Height at which a perspective camera frames a `widthM` x `heightM` rectangle. */
function overheadDistance(
  camera: THREE.PerspectiveCamera,
  widthM: number,
  heightM: number,
): number {
  const halfFov = (camera.fov * Math.PI) / 360;
  // The vertical fov is the fixed one; the horizontal follows from the aspect,
  // so a wide, short map is framed by its width and a tall one by its height.
  const forHeight = heightM / 2 / Math.tan(halfFov);
  const forWidth = widthM / 2 / (Math.tan(halfFov) * camera.aspect);
  return Math.max(forHeight, forWidth) * TOP_DOWN_MARGIN;
}

/**
 * The kinds of pose a drag on the viewport can produce.
 *
 * "vertex" re-places a stop that already exists, which is why the canvas needs
 * `movingVertex` alongside the mode: unlike the other two, the drag is *about*
 * a row, and the marker for it has to come off the map while it is in hand.
 */
export type PickMode = "goal" | "initial-pose" | "vertex";

interface PointCloudCanvasProps {
  /** 2D map metadata; when omitted the cloud renders with no ground plane. */
  meta?: MapMetadata;
  /**
   * Ground-plane texture URL — GET /api/v1/maps/{name}/image, absolute, from
   * apiUrl(). A plain URL rather than the base64 data URI the removed
   * /api/v1/map/image returned: TextureLoader takes either, and a real URL is
   * the one the browser can cache and revalidate against the endpoint's ETag.
   */
  mapImageUrl?: string;
  /**
   * Name of the map to load the static cloud for; required for showMapCloud to
   * do anything, since that cloud is now read per map from its saved map.pcd.
   */
  mapName?: string;
  pose?: RobotPose;
  /**
   * Joint angles in radians, keyed by URDF joint name (the vocabulary
   * MotorState.name / the telemetry stream uses, e.g. "FL_Knee_joint").
   * Applied to the GLB relative to its baked zero configuration. Omitted or
   * missing joints simply keep their last angle.
   */
  joints?: Record<string, number>;
  /** When true, also fetch and render the static localizer map cloud. */
  showMapCloud?: boolean;
  /**
   * The active map's stored vertices, drawn flat on the ground with their
   * headings and names. Read-only here: placing and editing them belongs to the
   * gridmap editor, and the dashboard's two ground gestures are already spoken
   * for by the pick modes below.
   */
  vertices?: MapVertex[];
  /** Hide the vertex layer without unmounting the canvas. Defaults to true. */
  showVertices?: boolean;
  /**
   * Fired when a stored vertex is double-clicked. Double, not single: a single
   * click on the viewport is already how the camera is driven, and a stop is a
   * place the robot will drive to — the gesture that proposes that has to be one
   * the operator cannot make by brushing the map.
   *
   * What it means is "the operator asked about this stop", not "go there": the
   * canvas never dispatches a task, it hands the row up and the view asks.
   */
  onVertexActivate?: (vertex: MapVertex) => void;
  /**
   * The vertex a `"vertex"` pick is re-placing. Its stored marker is hidden for
   * the duration — the draft under the pointer is the same stop, and drawing it
   * twice would leave the operator unsure which one they are about to save — and
   * its heading seeds the draft, so releasing without a drag keeps the heading it
   * already had instead of snapping to 0°.
   */
  movingVertex?: MapVertex | null;
  /**
   * Camera interaction mode. "move" = free navigation, left-drag pans the
   * scene. "focus" = the camera locks onto the robot (target follows its pose
   * and stays centred), left-drag orbits around it. Defaults to "move".
   */
  cameraMode?: "move" | "focus";
  /**
   * Bumped by the view's top-down button to swing the camera overhead. A nonce
   * rather than a callback the view holds, the same shape the gridmap editor's
   * Fit action uses (`fitNonce` in components/maps/grid-canvas.tsx): the camera
   * lives in here, and handing out a setter would give the view a second way to
   * reach it. 0 means "never pressed", so a fresh mount keeps its default view.
   *
   * It is an action, not a mode — nothing stops the next drag from orbiting
   * straight back out of it, which is why it is a button rather than a third
   * option beside Move / Focus.
   */
  topDownNonce?: number;
  /**
   * Open the live body_cloud WebSocket. Defaults to true; the model-preview
   * route turns it off so a machine with no backend running doesn't sit in the
   * stream's 2 s reconnect loop, logging a failed socket forever.
   */
  liveStream?: boolean;
  /** Committed goal, drawn on the ground until the caller clears it. */
  goal?: PlanarPose | null;
  /**
   * The initial pose the operator last placed, drawn in the caution hue until
   * cleared. It outlives the publication on purpose — it is the reference the
   * reported pose is read against while the localizer's ICP converges — but only
   * by seconds: useInitialPose expires a published estimate on its own, and this
   * marker goes with it.
   */
  initialPose?: PlanarPose | null;
  /**
   * What a press-drag-release on the ground currently produces, or null for
   * none (RViz style either way): the press point is projected onto the z=0 map
   * plane and the drag direction gives the heading. Left-button camera motion is
   * suspended while a mode is armed.
   *
   * In "initial-pose" mode the robot model is picked up as soon as the mode is
   * armed: it floats under the bare pointer until a press plants it, so the
   * preview is the machine itself rather than only the draft arrow.
   *
   * One mode at a time — the gesture is identical for both, so the only thing
   * telling the operator (and this canvas) which pose they are placing is which
   * mode is armed. The view owns that choice.
   */
  pickMode?: PickMode | null;
  /** Fired once on release with the dragged pose, for whichever mode is armed. */
  onPickCommit?: (pose: PlanarPose) => void;
  onStatus?: (status: StreamStatus) => void;
  className?: string;
}

export function PointCloudCanvas({
  meta,
  mapImageUrl,
  mapName,
  pose,
  joints,
  showMapCloud,
  vertices,
  showVertices = true,
  onVertexActivate,
  movingVertex = null,
  cameraMode = "move",
  topDownNonce = 0,
  liveStream = true,
  goal = null,
  initialPose = null,
  pickMode = null,
  onPickCommit,
  onStatus,
  className,
}: PointCloudCanvasProps) {
  const containerRef = React.useRef<HTMLDivElement>(null);
  const { resolvedTheme } = useTheme();

  // Pose being carried / dragged right now. Kept in state (not a ref) so the
  // marker effect below runs on every pointer move; the scene itself is
  // untouched, so this costs a marker transform per frame, not a rebuild.
  const [draft, setDraft] = React.useState<PlanarPose | null>(null);

  /**
   * True while the draft is being *carried* — following the bare pointer with no
   * button down — as opposed to planted by a press and being turned by a drag.
   *
   * Only initial-pose mode carries. Arming it takes the robot out of the pose
   * feed's hands and puts it in the pointer's, so the operator sees the machine
   * itself track the cursor and can read the fit against the cloud before
   * committing to a spot; a press then pins that spot and the drag aims it.
   */
  const [carrying, setCarrying] = React.useState(false);

  // All mutable three.js objects live here so the pose / map-cloud effects can
  // reach into the scene without tearing it down.
  const sceneRef = React.useRef<{
    renderer: THREE.WebGLRenderer;
    scene: THREE.Scene;
    camera: THREE.PerspectiveCamera;
    controls: OrbitControls;
    liveGeom: THREE.BufferGeometry;
    mapPoints: THREE.Points | null;
    /**
     * The map extent this scene was built around, in metres, or null when there
     * is no 2D map. Kept here rather than read off the `meta` prop so the
     * top-down effect can frame the map without listing `meta` as a dependency —
     * which would re-frame the camera every time a map loads.
     */
    mapFrame: { cx: number; cy: number; widthM: number; heightM: number } | null;
    goalMarker: THREE.Group;
    initialPoseMarker: THREE.Group;
    draftMarker: THREE.Group;
    dispose: () => void;
  } | null>(null);

  // Keep the latest onStatus without forcing the setup effect to re-run.
  const onStatusRef = React.useRef(onStatus);
  React.useEffect(() => {
    onStatusRef.current = onStatus;
  }, [onStatus]);

  // Camera mode + pose easing state, read by the render loop and by effects
  // that must not re-run on every change without listing them as deps.
  //
  // Both pose refs live at component scope rather than inside the scene-setup
  // effect so a map load or theme toggle rebuilds the renderer without
  // restarting the animation from the world origin.
  const cameraModeRef = React.useRef(cameraMode);
  const pickModeRef = React.useRef(pickMode);
  const targetPoseRef = React.useRef<SmoothPose | null>(null);
  const renderedPoseRef = React.useRef<SmoothPose | null>(null);

  /**
   * Where the robot model is drawn *instead of* the reported pose, while an
   * initial-pose drag is in flight. Null the rest of the time.
   *
   * A ref rather than state because the render loop reads it every frame; the
   * pointer handlers already hold the same drag in `draft` for the marker.
   */
  const posePreviewRef = React.useRef<SmoothPose | null>(null);

  // The stored-vertex layer and the marker currently lit under the pointer.
  // `hoverIdRef` is the scene's copy and drives the highlight at pointer rate;
  // the state alongside it exists only so the cursor can become a pointer, which
  // is a render — hence two, rather than one read at two speeds.
  const vertexLayerRef = React.useRef<VertexLayer | null>(null);
  const hoverIdRef = React.useRef<string | null>(null);
  /** Mirrors `movingVertex` for the layer effect, which rebuilds around it. */
  const movingIdRef = React.useRef<string | null>(null);
  const [hoveredVertex, setHoveredVertex] = React.useState<MapVertex | null>(
    null,
  );

  // Joint articulation. The GLB keeps URDF link names as node names, so each
  // joint resolves to the child-link Object3D it rotates. Nodes belong to the
  // module-cached model instance, so the resolved map stays valid across scene
  // rebuilds — resolve once, on first model attach. `latestJointsRef` buffers
  // the newest joints so angles that arrive while the model is still loading
  // are applied as soon as it lands (mirrors targetPoseRef for the body pose).
  const jointNodesRef = React.useRef<Map<
    string,
    { node: THREE.Object3D; axis: THREE.Vector3 }
  > | null>(null);
  const latestJointsRef = React.useRef<Record<string, number> | undefined>(
    undefined,
  );

  const applyJoints = React.useCallback(() => {
    const nodes = jointNodesRef.current;
    const target = latestJointsRef.current;
    if (!nodes || !target) return;
    for (const [name, q] of Object.entries(target)) {
      const joint = nodes.get(name);
      if (!joint) {
        if (!warnedUnknownJoints.has(name)) {
          warnedUnknownJoints.add(name);
          console.warn(`unknown joint "${name}" — not in G23_JOINTS / the GLB`);
        }
        continue;
      }
      // The GLB is baked at the URDF zero configuration with identity joint
      // rotations (no rpy on any joint origin), so q is absolute: overwrite
      // the quaternion, leave the baked position (the joint origin) alone.
      joint.node.quaternion.setFromAxisAngle(joint.axis, q);
    }
  }, []);

  // ---- Scene setup (rebuilds on map / theme change) --------------------
  React.useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    const theme = THEMES[resolvedTheme === "dark" ? "dark" : "light"];
    const widthM = meta ? meta.width * meta.resolution : DEFAULT_SPAN_M;
    const heightM = meta ? meta.height * meta.resolution : DEFAULT_SPAN_M;
    const centerX = meta ? meta.origin[0] + widthM / 2 : 0;
    const centerY = meta ? meta.origin[1] + heightM / 2 : 0;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(theme.background);

    // z-up world so ROS map coordinates (x, y, z) map straight through.
    const camera = new THREE.PerspectiveCamera(60, 1, 0.1, 2000);
    camera.up.set(0, 0, 1);
    const span = Math.max(widthM, heightM);
    camera.position.set(centerX, centerY - span * 0.6, span * 0.8);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    container.appendChild(renderer.domElement);
    renderer.domElement.style.width = "100%";
    renderer.domElement.style.height = "100%";
    renderer.domElement.style.display = "block";

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(centerX, centerY, 0);
    controls.enableDamping = true;
    // Left-button behaviour tracks the camera mode; the dedicated mode effect
    // keeps this in sync, but seed it here so a scene rebuild preserves it.
    applyCameraMode(
      controls,
      cameraModeRef.current,
      pickModeRef.current !== null,
    );
    controls.update();

    // Ground plane textured with the 2D occupancy grid for spatial context.
    // Only drawn when a 2D map is available; a raw cloud test skips it.
    if (meta) {
      const ground = new THREE.Mesh(
        new THREE.PlaneGeometry(widthM, heightM),
        new THREE.MeshBasicMaterial({
          color: theme.ground,
          transparent: true,
          opacity: theme.groundOpacity,
          side: THREE.DoubleSide,
        }),
      );
      ground.position.set(centerX, centerY, 0);
      scene.add(ground);

      if (mapImageUrl) {
        new THREE.TextureLoader().load(mapImageUrl, (texture) => {
          texture.colorSpace = THREE.SRGBColorSpace;
          const mat = ground.material as THREE.MeshBasicMaterial;
          mat.map = texture;
          mat.color.set(0xffffff);
          mat.needsUpdate = true;
        });
      }
    }

    // Live body_cloud: preallocated dynamic buffers, drawn up to drawRange.
    const liveGeom = new THREE.BufferGeometry();
    const positions = new Float32Array(MAX_LIVE_POINTS * 3);
    const colors = new Float32Array(MAX_LIVE_POINTS * 3);
    liveGeom.setAttribute(
      "position",
      new THREE.BufferAttribute(positions, 3).setUsage(THREE.DynamicDrawUsage),
    );
    liveGeom.setAttribute(
      "color",
      new THREE.BufferAttribute(colors, 3).setUsage(THREE.DynamicDrawUsage),
    );
    liveGeom.setDrawRange(0, 0);
    const livePoints = new THREE.Points(
      liveGeom,
      new THREE.PointsMaterial({ size: LIVE_POINT_SIZE, vertexColors: true }),
    );
    livePoints.frustumCulled = false;
    scene.add(livePoints);

    // The robot is the only lit object in the scene — the ground, both clouds
    // and the old marker are all unlit — so these lights exist solely to give
    // it readable form. The hemisphere fill keeps its underside off pure black
    // and the directional key rakes across the body from the default camera
    // side, which is what makes the leg geometry legible.
    scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 2.0));
    const keyLight = new THREE.DirectionalLight(0xffffff, 1.5);
    keyLight.position.set(-1, -1.5, 3);
    scene.add(keyLight);

    // Pose markers: the committed goal, the staged initial pose, and the
    // lighter one that follows the drag (recoloured per pick mode). All start
    // hidden; the marker effect below places them and re-runs after a scene
    // rebuild, so a rebuild mid-drag does not lose them.
    const goalMarker = createPoseMarker(theme.goal, 1);
    const initialPoseMarker = createPoseMarker(theme.initialPose, 1);
    const draftMarker = createPoseMarker(theme.goalDraft, 0.55);
    scene.add(goalMarker);
    scene.add(initialPoseMarker);
    scene.add(draftMarker);

    const robotGroup = new THREE.Group();
    // Hidden until the first pose arrives (a raw cloud test has no /robot/state).
    robotGroup.visible = false;
    scene.add(robotGroup);

    // Attach the shared model once it resolves. `cancelled` guards a scene
    // rebuild that lands mid-load: Object3D.add() reparents, so a stale
    // callback would steal the model out of the group the new scene just built.
    let cancelled = false;
    loadRobotModel()
      .then((model) => {
        if (cancelled) return;
        model.position.z = ROBOT_BASE_HEIGHT_M;
        robotGroup.add(model);
        // Resolve joint -> child-link nodes once; the map survives scene
        // rebuilds because the nodes belong to the cached model.
        if (!jointNodesRef.current) {
          const nodes = new Map<
            string,
            { node: THREE.Object3D; axis: THREE.Vector3 }
          >();
          for (const [name, spec] of G23_JOINTS) {
            const node = model.getObjectByName(spec.childLink);
            if (node) {
              nodes.set(name, { node, axis: spec.axis });
            } else {
              console.warn(
                `joint "${name}": link node "${spec.childLink}" missing from GLB`,
              );
            }
          }
          jointNodesRef.current = nodes;
        }
        applyJoints();
      })
      .catch((err) => console.error("robot model failed to load", err));

    const resize = () => {
      const rect = container.getBoundingClientRect();
      if (rect.width === 0 || rect.height === 0) return;
      renderer.setSize(rect.width, rect.height, false);
      camera.aspect = rect.width / rect.height;
      camera.updateProjectionMatrix();
    };
    resize();
    const observer = new ResizeObserver(resize);
    observer.observe(container);

    /**
     * Advance the drawn pose toward the reported one by one frame.
     *
     * The exponential factor is derived from dt rather than applied per frame,
     * so a 120 Hz display converges at the same wall-clock rate as a 60 Hz one
     * (and a tab restored after minutes in the background lands on a large dt,
     * i.e. snaps — which is the right answer, the robot really has moved).
     */
    const stepPose = (dt: number) => {
      const target = targetPoseRef.current;
      let cur = renderedPoseRef.current;

      if (target) {
        if (cur) {
          const a = 1 - Math.exp(-dt / POSE_EASE_TAU_S);
          const dx = (target.x - cur.x) * a;
          const dy = (target.y - cur.y) * a;
          const dz = (target.z - cur.z) * a;
          cur.x += dx;
          cur.y += dy;
          cur.z += dz;
          cur.yaw += (target.yaw - cur.yaw) * a;

          // Focus mode: shift the camera by the same delta so the viewing offset
          // (and any orbit the user set) survives while the robot moves. This
          // used to run per pose update, which made the 1 Hz jump doubly
          // obvious — the whole view lurched, not just the robot.
          if (cameraModeRef.current === "focus") {
            camera.position.x += dx;
            camera.position.y += dy;
            camera.position.z += dz;
          }
        } else {
          // First fix: snap, so the robot does not fly in from the world origin.
          cur = renderedPoseRef.current = { ...target };
        }
      }

      // Focus mode tracks the *reported* pose even mid-drag: the camera follows
      // the machine, not the estimate being placed. Easing above also keeps
      // running through a drag, so releasing it hands the model back to a
      // current pose rather than to wherever the robot was when the drag began.
      if (cur && cameraModeRef.current === "focus") {
        controls.target.set(cur.x, cur.y, cur.z);
      }

      // An initial-pose drag draws the robot at the dragged pose instead — no
      // easing, so the model tracks the pointer 1:1 the way a dragged object
      // has to. Until either exists there is nothing to draw (a raw cloud test
      // with no pose feed).
      const drawn = posePreviewRef.current ?? cur;
      if (!drawn) return;
      robotGroup.position.set(drawn.x, drawn.y, drawn.z);
      robotGroup.rotation.z = drawn.yaw;
      robotGroup.visible = true;
    };

    let raf = 0;
    let prevFrameMs = performance.now();
    const animate = () => {
      const now = performance.now();
      const dt = (now - prevFrameMs) / 1000;
      prevFrameMs = now;

      stepPose(dt);
      controls.update();
      renderer.render(scene, camera);
      raf = requestAnimationFrame(animate);
    };
    raf = requestAnimationFrame(animate);

    const dispose = () => {
      cancelled = true;
      cancelAnimationFrame(raf);
      observer.disconnect();
      controls.dispose();
      renderer.dispose();
      // Detach the robot before the sweep below. The model is cached across
      // scene rebuilds (theme / map changes), so letting the traverse dispose
      // its geometry would leave every later mount with an empty group.
      robotGroup.clear();
      scene.traverse((obj) => {
        const mesh = obj as THREE.Mesh;
        if (mesh.geometry) mesh.geometry.dispose();
        const mat = mesh.material as THREE.Material | THREE.Material[];
        if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
        else if (mat) mat.dispose();
      });
      if (renderer.domElement.parentNode === container) {
        container.removeChild(renderer.domElement);
      }
    };

    sceneRef.current = {
      renderer,
      scene,
      camera,
      controls,
      liveGeom,
      mapPoints: null,
      mapFrame: meta ? { cx: centerX, cy: centerY, widthM, heightM } : null,
      goalMarker,
      initialPoseMarker,
      draftMarker,
      dispose,
    };

    return () => {
      dispose();
      sceneRef.current = null;
    };
  }, [meta, mapImageUrl, resolvedTheme, applyJoints]);

  // ---- Live body_cloud stream (independent of scene rebuilds) ----------
  // The WebSocket is opened once on mount and closed on unmount. Each frame is
  // written into whatever live geometry the scene-setup effect currently owns
  // (via sceneRef), so map loads and theme changes rebuild the scene without
  // tearing down the socket. Previously the stream lived in the setup effect,
  // so an async map load closed the still-connecting WS and logged
  // "WebSocket is closed before the connection is established".
  React.useEffect(() => {
    if (!liveStream) return;

    const color = new THREE.Color();
    const applyFrame = (frame: PointCloudFrame) => {
      const ctx = sceneRef.current;
      if (!ctx) return;
      const posAttr = ctx.liveGeom.getAttribute(
        "position",
      ) as THREE.BufferAttribute;
      const colAttr = ctx.liveGeom.getAttribute(
        "color",
      ) as THREE.BufferAttribute;
      const n = Math.min(frame.count, MAX_LIVE_POINTS);
      const src = frame.positions;
      const dstPos = posAttr.array as Float32Array;
      const dstCol = colAttr.array as Float32Array;
      for (let i = 0; i < n; i++) {
        const j = i * 3;
        dstPos[j] = src[j];
        dstPos[j + 1] = src[j + 1];
        dstPos[j + 2] = src[j + 2];
        heightColor(src[j + 2], color);
        dstCol[j] = color.r;
        dstCol[j + 1] = color.g;
        dstCol[j + 2] = color.b;
      }
      posAttr.needsUpdate = true;
      colAttr.needsUpdate = true;
      ctx.liveGeom.setDrawRange(0, n);
    };

    const stream = createPointCloudStream({
      onFrame: applyFrame,
      onStatus: (s) => onStatusRef.current?.(s),
    });
    return () => stream.close();
  }, [liveStream]);

  // ---- Pose updates (no scene rebuild) ---------------------------------
  // This only records where the robot should be; the render loop above walks
  // the drawn pose toward it. Nothing here touches the scene, so it is safe to
  // run while a rebuild is in flight.
  React.useEffect(() => {
    if (!pose) return;
    const yaw = (pose.theta * Math.PI) / 180;
    const cur = renderedPoseRef.current;
    targetPoseRef.current = {
      x: pose.x,
      y: pose.y,
      z: pose.z,
      // Unwrap against the drawn yaw so the ease takes the short way round:
      // 359deg -> 1deg has to be +2deg, not a -358deg spin in place.
      yaw: cur
        ? yaw + Math.round((cur.yaw - yaw) / (2 * Math.PI)) * 2 * Math.PI
        : yaw,
    };
  }, [pose]);

  // ---- Joint updates (no scene rebuild) ---------------------------------
  // Applied directly, no easing: the telemetry feed runs at ~20 Hz, fast
  // enough that legs read as continuous — unlike the 1 Hz body pose above.
  React.useEffect(() => {
    latestJointsRef.current = joints;
    applyJoints();
  }, [joints, applyJoints]);

  // ---- Camera mode (move / focus) --------------------------------------
  // Re-runs on a scene rebuild (meta / theme) too, so the mode survives a
  // renderer teardown.
  React.useEffect(() => {
    cameraModeRef.current = cameraMode;
    const ctx = sceneRef.current;
    if (!ctx) return;
    applyCameraMode(ctx.controls, cameraMode, pickModeRef.current !== null);
    // Entering focus: frame the robot from a fixed offset behind and above it.
    if (cameraMode === "focus" && renderedPoseRef.current) {
      const p = renderedPoseRef.current;
      ctx.controls.target.set(p.x, p.y, p.z);
      ctx.camera.position.set(p.x, p.y - 8, p.z + 6);
    }
    ctx.controls.update();
  }, [cameraMode, meta, mapImageUrl, resolvedTheme]);

  // ---- Top-down view (one-shot) ----------------------------------------
  // Deliberately not a dep of the camera-mode effect above and deliberately not
  // re-run on a scene rebuild: this answers a button press, and a map load or a
  // theme toggle re-framing the camera would throw away a view the operator set
  // by hand. The nonce is the only thing that triggers it.
  React.useEffect(() => {
    if (!topDownNonce) return;
    const ctx = sceneRef.current;
    if (!ctx) return;
    const { camera, controls, mapFrame } = ctx;

    // Focus mode's target is the robot and the render loop keeps writing it, so
    // there the only choice left is the height — and the current one is the
    // operator's own zoom, which a "look from above" should not discard.
    // Otherwise a map to frame beats wherever the view had been panned to:
    // the whole point of going overhead is to see the site, not the corner of
    // it that happened to be under the camera.
    const framing = cameraModeRef.current !== "focus" ? mapFrame : null;
    if (framing) controls.target.set(framing.cx, framing.cy, 0);

    const height = framing
      ? overheadDistance(camera, framing.widthM, framing.heightM)
      : camera.position.distanceTo(controls.target);

    camera.position.set(
      controls.target.x,
      // Not exactly overhead — see TOP_DOWN_TILT.
      controls.target.y - height * TOP_DOWN_TILT,
      controls.target.z + height,
    );
    controls.update();
  }, [topDownNonce]);

  // ---- Pick mode: hand the left button over ----------------------------
  // Separate from the effect above (which also re-frames the camera when focus
  // mode is entered — arming a pick must not jolt the view). `cameraMode` is a
  // dep so the button mapping is re-asserted after that effect rewrites it.
  React.useEffect(() => {
    pickModeRef.current = pickMode;
    const ctx = sceneRef.current;
    if (!ctx) return;
    applyCameraMode(ctx.controls, cameraModeRef.current, pickMode !== null);
  }, [pickMode, cameraMode, meta, mapImageUrl, resolvedTheme]);

  // ---- Pose markers (no scene rebuild) ---------------------------------
  React.useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    const theme = THEMES[resolvedTheme === "dark" ? "dark" : "light"];

    placePoseMarker(ctx.goalMarker, goal);
    placePoseMarker(ctx.initialPoseMarker, initialPose);

    // The draft belongs to whichever mode is armed, in that mode's hue — the
    // arrow has to say which pose is being placed while it is being placed, not
    // once it lands in a readback panel. Leaving the mode mid-drag must not
    // strand a marker on the map, hence the null.
    setMarkerColor(
      ctx.draftMarker,
      pickMode === "initial-pose" ? theme.initialPoseDraft : theme.goalDraft,
    );
    placePoseMarker(ctx.draftMarker, pickMode ? draft : null);
  }, [goal, initialPose, draft, pickMode, meta, mapImageUrl, resolvedTheme]);

  // ---- Initial-pose pick: the robot itself is the preview ----------------
  // An initial pose asserts where the machine *is*, so the machine is what
  // moves: arming the mode takes the robot off the reported pose and hands it to
  // the pointer, a press plants it, the drag turns it to face where it faces.
  // That makes the gesture self-describing — what stands on the floor at release
  // is exactly what gets published — which is why this flow needs no confirm.
  //
  // A goal pick deliberately does not do this. A goal is somewhere to go, not a
  // claim about the present, and moving the robot to preview one would state
  // something false; the arrow marker is the whole of that preview.
  React.useEffect(() => {
    if (pickMode !== "initial-pose" || !draft) {
      posePreviewRef.current = null;
      return;
    }
    posePreviewRef.current = {
      x: draft.x,
      y: draft.y,
      // Whatever height the robot is already drawn at — the pose feed is planar
      // (lio_bridge reports z ~ 0), so this keeps it on the same floor rather
      // than guessing a new one — plus the lift while it is still in hand.
      z: (renderedPoseRef.current?.z ?? 0) + (carrying ? CARRY_LIFT_M : 0),
      yaw: (draft.theta * Math.PI) / 180,
    };
  }, [draft, carrying, pickMode]);

  // ---- Stored map vertices (toggle) ------------------------------------
  // The layer is reached into after it is built — hover recolours a marker on
  // every pointer move — so it is held in a ref rather than being rebuilt.
  // `hoverIdRef` mirrors the id the layer is currently showing, so a rebuild
  // (map swap, theme toggle) can restore the highlight, and so a pointer move
  // that stays on the same marker costs one comparison and no React render.
  //
  // The layer itself is rebuilt rather than diffed, so it needs no slot in
  // sceneRef: the group is created here and this effect's own cleanup takes it
  // back out. The scene-rebuild deps (meta / mapImageUrl / theme) are listed for
  // the same reason the marker effect lists them — a rebuild drops the old
  // scene, and without them the layer would never be re-added to the new one.
  React.useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx || !showVertices || !vertices?.length) return;

    const theme = THEMES[resolvedTheme === "dark" ? "dark" : "light"];
    const layer = createVertexLayer(vertices, theme);
    ctx.scene.add(layer.group);
    vertexLayerRef.current = layer;
    // A theme toggle under a resting pointer must not drop the highlight, and a
    // list patched by a re-place rebuilds this layer mid-gesture: both ids
    // survive the rebuild even though the objects wearing them do not.
    layer.setHovered(hoverIdRef.current);
    layer.setMoving(movingIdRef.current);

    return () => {
      // `ctx.scene` may already be the discarded scene by the time this runs
      // (the setup effect's cleanup goes first); removing from it is harmless
      // either way, and the dispose is what actually matters — the teardown
      // traverse frees geometries and materials but never the badge textures.
      ctx.scene.remove(layer.group);
      layer.dispose();
      vertexLayerRef.current = null;
    };
  }, [vertices, showVertices, meta, mapImageUrl, resolvedTheme]);

  // A stop in the operator's hands comes off the map. Its own effect rather than
  // a dep of the one above: arming a re-place must not rebuild the layer, and
  // the build effect re-applies this from the ref after a rebuild anyway.
  React.useEffect(() => {
    movingIdRef.current = movingVertex?.id ?? null;
    vertexLayerRef.current?.setMoving(movingIdRef.current);
  }, [movingVertex]);

  // ---- Optional static map cloud (toggle) ------------------------------
  React.useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;

    if (!showMapCloud || !mapName) {
      if (ctx.mapPoints) {
        ctx.scene.remove(ctx.mapPoints);
        ctx.mapPoints.geometry.dispose();
        (ctx.mapPoints.material as THREE.Material).dispose();
        ctx.mapPoints = null;
      }
      return;
    }

    const theme = THEMES[resolvedTheme === "dark" ? "dark" : "light"];
    const abort = new AbortController();
    fetchMapPointCloud(mapName, { signal: abort.signal })
      .then((frame) => {
        if (abort.signal.aborted || !sceneRef.current) return;
        const geom = new THREE.BufferGeometry();
        geom.setAttribute(
          "position",
          new THREE.BufferAttribute(frame.positions, 3),
        );
        // Solid map-cloud colour (white on dark, dark-grey on light) keeps it
        // visually distinct from the height-coloured live body cloud.
        const points = new THREE.Points(
          geom,
          new THREE.PointsMaterial({
            size: MAP_POINT_SIZE,
            color: theme.mapCloud,
            opacity: 0.6,
            transparent: true,
          }),
        );
        points.frustumCulled = false;
        sceneRef.current.mapPoints = points;
        sceneRef.current.scene.add(points);
      })
      .catch((err) => {
        if (!abort.signal.aborted) console.error(err);
      });

    return () => abort.abort();
  }, [showMapCloud, mapName, resolvedTheme]);

  // ---- Pose picking -----------------------------------------------------
  // The anchor is the ground point the press landed on, kept alongside the raw
  // pointer position so the heading deadzone can be measured in screen pixels:
  // a metric deadzone would be enormous at the far end of a perspective view
  // and vanishingly small up close.
  const anchorRef = React.useRef<{
    wx: number;
    wy: number;
    cx: number;
    cy: number;
  } | null>(null);
  // Lazily constructed: this component re-renders at the telemetry rate, and a
  // useRef initialiser argument is evaluated (then thrown away) every render.
  const raycasterRef = React.useRef<THREE.Raycaster | null>(null);

  /** Aim the shared raycaster through a pointer position, or null off-canvas. */
  const castFromPointer = (event: { clientX: number; clientY: number }) => {
    const ctx = sceneRef.current;
    if (!ctx) return null;
    const rect = ctx.renderer.domElement.getBoundingClientRect();
    if (rect.width === 0 || rect.height === 0) return null;

    const raycaster = (raycasterRef.current ??= new THREE.Raycaster());
    raycaster.setFromCamera(
      new THREE.Vector2(
        ((event.clientX - rect.left) / rect.width) * 2 - 1,
        -((event.clientY - rect.top) / rect.height) * 2 + 1,
      ),
      ctx.camera,
    );
    return raycaster;
  };

  /** Project a pointer position onto the z=0 map plane. */
  const pickGround = (event: React.PointerEvent) => {
    const raycaster = castFromPointer(event);
    if (!raycaster) return null;
    const hit = raycaster.ray.intersectPlane(GROUND_PLANE, new THREE.Vector3());
    // Misses when the ray runs parallel to the floor or points at the sky.
    return hit ? { wx: hit.x, wy: hit.y } : null;
  };

  /**
   * The stored vertex under a pointer position, or null.
   *
   * A real raycast against the layer, not a distance test against projected
   * screen positions: the markers stand on the floor of a perspective view, so
   * "near the pointer" only means anything after the projection the raycaster is
   * already doing. Hidden layer, no hits — the ref is null.
   */
  const pickVertex = (event: { clientX: number; clientY: number }) => {
    const layer = vertexLayerRef.current;
    if (!layer) return null;
    const raycaster = castFromPointer(event);
    if (!raycaster) return null;
    const hit = raycaster.intersectObjects(layer.pickables, false)[0];
    const id = hit?.object.userData.vertexId as string | undefined;
    return id ? (layer.byId.get(id) ?? null) : null;
  };

  /** Light the marker under the pointer, or clear the highlight. */
  const hoverVertex = (vertex: MapVertex | null) => {
    const id = vertex?.id ?? null;
    // Told every time, not only on a change: setHovered is self-deduping, and
    // going through it unconditionally is what re-lights a marker whose layer
    // was rebuilt while the pointer sat still on it.
    vertexLayerRef.current?.setHovered(id);
    if (id === hoverIdRef.current) return;
    hoverIdRef.current = id;
    setHoveredVertex(vertex);
  };

  /**
   * The planner can only plan inside the occupancy grid, and the localizer can
   * only match against the map it loaded, so a press that lands on floor beyond
   * the map must not become a pose. With no 2D map loaded there is nothing to
   * bound against, so any ground point is accepted.
   */
  const insideMap = (wx: number, wy: number) => {
    if (!meta) return true;
    const [x0, y0] = meta.origin;
    return (
      wx >= x0 &&
      wx <= x0 + meta.width * meta.resolution &&
      wy >= y0 &&
      wy <= y0 + meta.height * meta.resolution
    );
  };

  /**
   * The heading a draft starts at, before any drag aims it.
   *
   * A re-place starts from the stop's own heading rather than the robot's: the
   * gesture is about where that stop is, and a plain click to nudge it half a
   * metre must not silently spin it to face wherever the machine happens to be
   * pointing. The other two modes keep the robot's heading, which is the sane
   * default when the pose being placed is the robot's own.
   */
  const seedTheta = () =>
    draft?.theta ??
    (pickMode === "vertex" ? movingVertex?.theta : pose?.theta) ??
    0;

  const handlePointerDown = (event: React.PointerEvent) => {
    if (!pickMode || event.button !== 0) return;
    const hit = pickGround(event);
    if (!hit || !insideMap(hit.wx, hit.wy)) return;
    anchorRef.current = { ...hit, cx: event.clientX, cy: event.clientY };
    // Capture on the canvas, not on this container: OrbitControls captures the
    // same pointer on the canvas itself, and capturing further up the tree
    // would steal it and strand OrbitControls' pointerup handler.
    sceneRef.current?.renderer.domElement.setPointerCapture(event.pointerId);
    // The press plants what was being carried: same spot, on the floor now.
    setCarrying(false);
    // Until the pointer moves, keep the heading it was carried at (the seed, on
    // the first press) so a plain click still yields a sane pose instead of
    // snapping to 0deg.
    setDraft({ x: hit.wx, y: hit.wy, theta: seedTheta() });
  };

  const handlePointerMove = (event: React.PointerEvent) => {
    if (!pickMode) {
      // Hover-testing the vertex layer is the *only* thing a bare pointer move
      // does here. Skipped with a button down: that is a camera drag, and
      // lighting up markers the view is sweeping past says the pointer is over
      // something when it is really just orbiting.
      if (event.buttons === 0) hoverVertex(pickVertex(event));
      return;
    }
    // A pose is being placed, so nothing on the map is a target — and leaving a
    // marker lit under a crosshair would offer a second meaning for a gesture
    // that already has one.
    hoverVertex(null);

    const anchor = anchorRef.current;

    // No press yet: carry the pose under the pointer, keeping the heading. Goal
    // mode alone does not — an arrow trailing the cursor with nothing committed
    // would read as a goal that is already set. The other two are *moving*
    // something that exists (the robot, or a stored stop), so seeing it follow
    // the pointer before the press is the whole point. A cursor over ground
    // outside the map carries nothing, for the same reason a press there does
    // not plant: it could not be published.
    if (!anchor) {
      if (pickMode === "goal") return;
      const hit = pickGround(event);
      if (!hit || !insideMap(hit.wx, hit.wy)) {
        setDraft(null);
        return;
      }
      setCarrying(true);
      setDraft({ x: hit.wx, y: hit.wy, theta: seedTheta() });
      return;
    }

    // Planted: the position is fixed at the anchor and the drag only aims.
    // Keep whatever heading the draft already has inside the deadzone.
    let theta = seedTheta();
    const dragPx = Math.hypot(
      event.clientX - anchor.cx,
      event.clientY - anchor.cy,
    );
    if (dragPx >= HEADING_DEADZONE_PX) {
      // World-space angle from the anchor to wherever the drag now points at
      // the floor. This cannot come from the screen delta: the camera may be
      // looking at the map from any azimuth (or from below), so screen-right is
      // not world +x.
      const hit = pickGround(event);
      if (hit) {
        const dx = hit.wx - anchor.wx;
        const dy = hit.wy - anchor.wy;
        if (dx !== 0 || dy !== 0) theta = (Math.atan2(dy, dx) * 180) / Math.PI;
      }
    }
    setDraft({ x: anchor.wx, y: anchor.wy, theta });
  };

  const handlePointerUp = (event: React.PointerEvent) => {
    if (!anchorRef.current) return;
    anchorRef.current = null;
    const canvas = sceneRef.current?.renderer.domElement;
    if (canvas?.hasPointerCapture(event.pointerId)) {
      canvas.releasePointerCapture(event.pointerId);
    }
    if (draft && pickMode) onPickCommit?.(draft);
    setDraft(null);
    setCarrying(false);
  };

  // The pointer leaving the viewport takes the carried robot with it, rather
  // than stranding it wherever it was last seen — the operator moving onto the
  // control panel has not chosen that spot. A press in flight is unaffected:
  // the canvas holds the pointer capture, so the drag survives.
  //
  // This is also what clears the draft on disarm, and why no effect watches
  // pickMode to do it: the disarm button is off-canvas, so reaching it means
  // passing through here first. Both the marker and preview effects gate on
  // pickMode anyway, so a draft left in state is never drawn.
  const handlePointerLeave = () => {
    hoverVertex(null);
    if (anchorRef.current) return;
    setDraft(null);
    setCarrying(false);
  };

  /**
   * Double-click a stored vertex to ask about it. Ignored while a pick mode is
   * armed — the two presses of the double-click have already staged and
   * committed a pose by the time this fires, and a dialog on top of that would
   * be asking about the wrong thing.
   */
  const handleDoubleClick = (event: React.MouseEvent) => {
    if (pickMode || !onVertexActivate) return;
    const vertex = pickVertex(event);
    if (vertex) onVertexActivate(vertex);
  };

  return (
    <div
      ref={containerRef}
      className={cn(
        "relative h-full w-full overflow-hidden",
        pickMode && "cursor-crosshair touch-none",
        // Nothing else on this canvas is clickable, so the cursor is the only
        // thing that says a marker is.
        !pickMode && hoveredVertex && "cursor-pointer",
        className,
      )}
      onPointerDown={handlePointerDown}
      onPointerMove={handlePointerMove}
      onPointerUp={handlePointerUp}
      onPointerCancel={handlePointerUp}
      onPointerLeave={handlePointerLeave}
      onDoubleClick={handleDoubleClick}
    />
  );
}
