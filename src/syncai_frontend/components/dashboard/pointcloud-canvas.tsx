"use client";

import * as React from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { MeshoptDecoder } from "three/examples/jsm/libs/meshopt_decoder.module.js";
import { useTheme } from "next-themes";

import { cn } from "@/lib/utils";
import { G23_JOINTS } from "@/lib/robot/g23-joints";
import type { GoalPose } from "@/lib/api/task";
import type { MapMetadata, RobotPose } from "@/lib/types/robot";
import type { PointCloudFrame, StreamStatus } from "@/lib/types/pointcloud";
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
  /** Committed / being-dragged goal marker. Same greens as the 2D canvas so a
   *  goal reads as the same thing in either view. */
  goal: number;
  goalDraft: number;
}

const THEMES: Record<"light" | "dark", Theme> = {
  light: {
    background: 0xf5f5f5,
    ground: 0xffffff,
    groundOpacity: 0.85,
    mapCloud: 0x333333,
    goal: 0x059669,
    goalDraft: 0x10b981,
  },
  dark: {
    background: 0x0a0a0a,
    ground: 0x404040,
    groundOpacity: 0.6,
    mapCloud: 0xffffff,
    goal: 0x10b981,
    goalDraft: 0x34d399,
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

// Goal marker, in metres. The 2D canvas draws its arrow in screen-space pixels
// because a metric arrow is a couple of pixels long at map scale; here the
// opposite holds — the marker sits on the ground in a perspective view, so it
// has to be a real object of roughly robot size or it stops reading as a place
// on the floor. Lifted off z=0 to keep it out of a z-fight with the ground.
const GOAL_RING_INNER_M = 0.26;
const GOAL_RING_OUTER_M = 0.34;
const GOAL_SHAFT_LEN_M = 0.5;
const GOAL_SHAFT_RADIUS_M = 0.035;
const GOAL_HEAD_LEN_M = 0.26;
const GOAL_HEAD_RADIUS_M = 0.1;
const GOAL_MARKER_Z_M = 0.05;

/** Drag distance (CSS px) below which the heading is not taken from the drag. */
const HEADING_DEADZONE_PX = 8;

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
 * Ring + arrow marking a goal pose, built pointing down +x so the group's
 * rotation.z is the goal heading. Unlit (MeshBasicMaterial) like everything
 * else in the scene except the robot itself, so it keeps its colour whichever
 * way it faces.
 */
function createGoalMarker(color: number, opacity: number): THREE.Group {
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
      new THREE.RingGeometry(GOAL_RING_INNER_M, GOAL_RING_OUTER_M, 32),
      material,
    ),
  );

  // Cylinder / cone run along +y by default; -90deg about z aims them down +x.
  const shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(
      GOAL_SHAFT_RADIUS_M,
      GOAL_SHAFT_RADIUS_M,
      GOAL_SHAFT_LEN_M,
      12,
    ),
    material,
  );
  shaft.rotation.z = -Math.PI / 2;
  shaft.position.x = GOAL_RING_OUTER_M + GOAL_SHAFT_LEN_M / 2;
  group.add(shaft);

  const head = new THREE.Mesh(
    new THREE.ConeGeometry(GOAL_HEAD_RADIUS_M, GOAL_HEAD_LEN_M, 16),
    material,
  );
  head.rotation.z = -Math.PI / 2;
  head.position.x = GOAL_RING_OUTER_M + GOAL_SHAFT_LEN_M + GOAL_HEAD_LEN_M / 2;
  group.add(head);

  group.visible = false;
  return group;
}

/** Move a goal marker to a pose (degrees), or hide it when there is none. */
function placeGoalMarker(marker: THREE.Group, goal: GoalPose | null) {
  marker.visible = goal !== null;
  if (!goal) return;
  marker.position.set(goal.x, goal.y, GOAL_MARKER_Z_M);
  marker.rotation.z = (goal.theta * Math.PI) / 180;
}

/**
 * Wire the OrbitControls buttons for a camera mode.
 *  - "move":  left-drag pans (moves the view), right-drag orbits.
 *  - "focus": left-drag orbits around the locked target; panning is disabled
 *    so the target stays pinned to the robot.
 * Middle button always dollies (zoom).
 *
 * `goalMode` overrides the left button entirely: a left-drag then has to
 * produce a goal, not move the camera. Mapping it to null (OrbitControls falls
 * through to its no-action default) rather than disabling the controls outright
 * keeps right-drag orbit and wheel zoom live, so the operator can still look
 * around while placing a goal.
 */
function applyCameraMode(
  controls: OrbitControls,
  mode: "move" | "focus",
  goalMode: boolean,
) {
  const orbit = mode === "focus";
  controls.enablePan = !orbit;
  controls.mouseButtons = {
    LEFT: goalMode ? null : orbit ? THREE.MOUSE.ROTATE : THREE.MOUSE.PAN,
    MIDDLE: THREE.MOUSE.DOLLY,
    RIGHT: THREE.MOUSE.ROTATE,
  };
}

// Fallback world framing when no 2D map is available (e.g. a raw body_cloud
// render test with no map_server running). Points arrive near the LIO odom
// origin, so a modest span centred on the origin frames them sensibly.
const DEFAULT_SPAN_M = 20;

interface PointCloudCanvasProps {
  /** 2D map metadata; when omitted the cloud renders with no ground plane. */
  meta?: MapMetadata;
  /** Ground-plane texture (base64 PNG data URI from GET /api/v1/map/image). */
  mapImageUrl?: string;
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
   * Camera interaction mode. "move" = free navigation, left-drag pans the
   * scene. "focus" = the camera locks onto the robot (target follows its pose
   * and stays centred), left-drag orbits around it. Defaults to "move".
   */
  cameraMode?: "move" | "focus";
  /**
   * Open the live body_cloud WebSocket. Defaults to true; the model-preview
   * route turns it off so a machine with no backend running doesn't sit in the
   * stream's 2 s reconnect loop, logging a failed socket forever.
   */
  liveStream?: boolean;
  /** Committed goal, drawn on the ground until the caller clears it. */
  goal?: GoalPose | null;
  /**
   * When true, a press-drag-release on the ground produces a goal (RViz style):
   * the press point is projected onto the z=0 map plane and the drag direction
   * gives the heading. Left-button camera motion is suspended for the duration.
   */
  goalMode?: boolean;
  /** Fired once on release with the dragged goal. */
  onGoalCommit?: (goal: GoalPose) => void;
  onStatus?: (status: StreamStatus) => void;
  className?: string;
}

export function PointCloudCanvas({
  meta,
  mapImageUrl,
  pose,
  joints,
  showMapCloud,
  cameraMode = "move",
  liveStream = true,
  goal = null,
  goalMode = false,
  onGoalCommit,
  onStatus,
  className,
}: PointCloudCanvasProps) {
  const containerRef = React.useRef<HTMLDivElement>(null);
  const { resolvedTheme } = useTheme();

  // Goal being dragged right now. Kept in state (not a ref) so the marker
  // effect below runs on every pointer move; the scene itself is untouched, so
  // this costs a marker transform per frame, not a rebuild.
  const [draft, setDraft] = React.useState<GoalPose | null>(null);

  // All mutable three.js objects live here so the pose / map-cloud effects can
  // reach into the scene without tearing it down.
  const sceneRef = React.useRef<{
    renderer: THREE.WebGLRenderer;
    scene: THREE.Scene;
    camera: THREE.PerspectiveCamera;
    controls: OrbitControls;
    liveGeom: THREE.BufferGeometry;
    mapPoints: THREE.Points | null;
    goalMarker: THREE.Group;
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
  const goalModeRef = React.useRef(goalMode);
  const targetPoseRef = React.useRef<SmoothPose | null>(null);
  const renderedPoseRef = React.useRef<SmoothPose | null>(null);

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
    applyCameraMode(controls, cameraModeRef.current, goalModeRef.current);
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

    // Goal markers: the committed one and the lighter one that follows the
    // drag. Both start hidden; the goal effect below places them and re-runs
    // after a scene rebuild, so a rebuild mid-goal does not lose the marker.
    const goalMarker = createGoalMarker(theme.goal, 1);
    const draftMarker = createGoalMarker(theme.goalDraft, 0.55);
    scene.add(goalMarker);
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
      if (!target) return;

      let cur = renderedPoseRef.current;
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

      robotGroup.position.set(cur.x, cur.y, cur.z);
      robotGroup.rotation.z = cur.yaw;
      robotGroup.visible = true;
      if (cameraModeRef.current === "focus") {
        controls.target.set(cur.x, cur.y, cur.z);
      }
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
      goalMarker,
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
    applyCameraMode(ctx.controls, cameraMode, goalModeRef.current);
    // Entering focus: frame the robot from a fixed offset behind and above it.
    if (cameraMode === "focus" && renderedPoseRef.current) {
      const p = renderedPoseRef.current;
      ctx.controls.target.set(p.x, p.y, p.z);
      ctx.camera.position.set(p.x, p.y - 8, p.z + 6);
    }
    ctx.controls.update();
  }, [cameraMode, meta, mapImageUrl, resolvedTheme]);

  // ---- Goal mode: hand the left button over ----------------------------
  // Separate from the effect above (which also re-frames the camera when focus
  // mode is entered — arming goal mode must not jolt the view). `cameraMode` is
  // a dep so the button mapping is re-asserted after that effect rewrites it.
  React.useEffect(() => {
    goalModeRef.current = goalMode;
    const ctx = sceneRef.current;
    if (!ctx) return;
    applyCameraMode(ctx.controls, cameraModeRef.current, goalMode);
  }, [goalMode, cameraMode, meta, mapImageUrl, resolvedTheme]);

  // ---- Goal markers (no scene rebuild) ---------------------------------
  React.useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    placeGoalMarker(ctx.goalMarker, goal);
    // The draft is only meaningful while goal mode is on: leaving the mode
    // mid-drag must not strand a marker on the map.
    placeGoalMarker(ctx.draftMarker, goalMode ? draft : null);
  }, [goal, draft, goalMode, meta, mapImageUrl, resolvedTheme]);

  // ---- Optional static map cloud (toggle) ------------------------------
  React.useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;

    if (!showMapCloud) {
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
    fetchMapPointCloud({ signal: abort.signal })
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
  }, [showMapCloud, resolvedTheme]);

  // ---- Goal picking -----------------------------------------------------
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

  /** Project a pointer position onto the z=0 map plane. */
  const pickGround = (event: React.PointerEvent) => {
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
    const hit = raycaster.ray.intersectPlane(GROUND_PLANE, new THREE.Vector3());
    // Misses when the ray runs parallel to the floor or points at the sky.
    return hit ? { wx: hit.x, wy: hit.y } : null;
  };

  /**
   * The planner can only plan inside the occupancy grid, so a press that lands
   * on floor beyond the map must not become a goal — the 2D canvas rejects the
   * letterbox margin for the same reason. With no 2D map loaded there is
   * nothing to bound against, so any ground point is accepted.
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

  const handlePointerDown = (event: React.PointerEvent) => {
    if (!goalMode || event.button !== 0) return;
    const hit = pickGround(event);
    if (!hit || !insideMap(hit.wx, hit.wy)) return;
    anchorRef.current = { ...hit, cx: event.clientX, cy: event.clientY };
    // Capture on the canvas, not on this container: OrbitControls captures the
    // same pointer on the canvas itself, and capturing further up the tree
    // would steal it and strand OrbitControls' pointerup handler.
    sceneRef.current?.renderer.domElement.setPointerCapture(event.pointerId);
    // Until the pointer moves, inherit the robot's current heading so a plain
    // click still yields a sane goal instead of snapping to 0deg.
    setDraft({ x: hit.wx, y: hit.wy, theta: pose?.theta ?? 0 });
  };

  const handlePointerMove = (event: React.PointerEvent) => {
    const anchor = anchorRef.current;
    if (!anchor || !goalMode) return;

    // Keep whatever heading the draft already has inside the deadzone.
    let theta = draft?.theta ?? pose?.theta ?? 0;
    const dragPx = Math.hypot(
      event.clientX - anchor.cx,
      event.clientY - anchor.cy,
    );
    if (dragPx >= HEADING_DEADZONE_PX) {
      // World-space angle from the anchor to wherever the drag now points at
      // the floor. Unlike the 2D canvas this cannot come from the screen delta:
      // the camera may be looking at the map from any azimuth (or from below),
      // so screen-right is not world +x.
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
    if (draft && goalMode) onGoalCommit?.(draft);
    setDraft(null);
  };

  return (
    <div
      ref={containerRef}
      className={cn(
        "relative h-full w-full overflow-hidden",
        goalMode && "cursor-crosshair touch-none",
        className,
      )}
      onPointerDown={handlePointerDown}
      onPointerMove={handlePointerMove}
      onPointerUp={handlePointerUp}
      onPointerCancel={handlePointerUp}
    />
  );
}
