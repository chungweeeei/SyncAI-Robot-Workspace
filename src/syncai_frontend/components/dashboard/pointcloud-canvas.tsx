"use client";

import * as React from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { MeshoptDecoder } from "three/examples/jsm/libs/meshopt_decoder.module.js";
import { useTheme } from "next-themes";

import { cn } from "@/lib/utils";
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
}

const THEMES: Record<"light" | "dark", Theme> = {
  light: {
    background: 0xf5f5f5,
    ground: 0xffffff,
    groundOpacity: 0.85,
    mapCloud: 0x333333,
  },
  dark: {
    background: 0x0a0a0a,
    ground: 0x404040,
    groundOpacity: 0.6,
    mapCloud: 0xffffff,
  },
};

// Point sizes (metres). The body cloud is the live focus; the map cloud sits a
// touch larger as a static spatial reference.
const LIVE_POINT_SIZE = 0.12;
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

// Time constant (seconds) of the easing applied to the reported pose. The feed
// is a 1 Hz snapshot polled over REST, so drawing it raw teleports the robot
// once a second; easing turns each update into a glide that reads as
// continuous motion. It is deliberately a filter rather than a replay buffer:
// smoothing costs a fraction of a second of lag but never renders a pose the
// robot has already left behind, which a buffer would. Worth lowering once the
// telemetry WebSocket raises the feed rate.
const POSE_EASE_TAU_S = 0.25;

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

/** Map a height to an RGB colour (blue = low, red = high) via an HSL sweep. */
function heightColor(z: number, out: THREE.Color): THREE.Color {
  const t = Math.min(1, Math.max(0, (z - Z_MIN) / (Z_MAX - Z_MIN)));
  // hue 240deg (blue) -> 0deg (red)
  return out.setHSL(((1 - t) * 240) / 360, 0.9, 0.55);
}

/**
 * Wire the OrbitControls buttons for a camera mode.
 *  - "move":  left-drag pans (moves the view), right-drag orbits.
 *  - "focus": left-drag orbits around the locked target; panning is disabled
 *    so the target stays pinned to the robot.
 * Middle button always dollies (zoom).
 */
function applyCameraMode(controls: OrbitControls, mode: "move" | "focus") {
  if (mode === "focus") {
    controls.enablePan = false;
    controls.mouseButtons = {
      LEFT: THREE.MOUSE.ROTATE,
      MIDDLE: THREE.MOUSE.DOLLY,
      RIGHT: THREE.MOUSE.ROTATE,
    };
  } else {
    controls.enablePan = true;
    controls.mouseButtons = {
      LEFT: THREE.MOUSE.PAN,
      MIDDLE: THREE.MOUSE.DOLLY,
      RIGHT: THREE.MOUSE.ROTATE,
    };
  }
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
  /** When true, also fetch and render the static localizer map cloud. */
  showMapCloud?: boolean;
  /**
   * Camera interaction mode. "move" = free navigation, left-drag pans the
   * scene. "focus" = the camera locks onto the robot (target follows its pose
   * and stays centred), left-drag orbits around it. Defaults to "move".
   */
  cameraMode?: "move" | "focus";
  onStatus?: (status: StreamStatus) => void;
  className?: string;
}

export function PointCloudCanvas({
  meta,
  mapImageUrl,
  pose,
  showMapCloud,
  cameraMode = "move",
  onStatus,
  className,
}: PointCloudCanvasProps) {
  const containerRef = React.useRef<HTMLDivElement>(null);
  const { resolvedTheme } = useTheme();

  // All mutable three.js objects live here so the pose / map-cloud effects can
  // reach into the scene without tearing it down.
  const sceneRef = React.useRef<{
    renderer: THREE.WebGLRenderer;
    scene: THREE.Scene;
    camera: THREE.PerspectiveCamera;
    controls: OrbitControls;
    liveGeom: THREE.BufferGeometry;
    mapPoints: THREE.Points | null;
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
  const targetPoseRef = React.useRef<SmoothPose | null>(null);
  const renderedPoseRef = React.useRef<SmoothPose | null>(null);

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
    applyCameraMode(controls, cameraModeRef.current);
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
      dispose,
    };

    return () => {
      dispose();
      sceneRef.current = null;
    };
  }, [meta, mapImageUrl, resolvedTheme]);

  // ---- Live body_cloud stream (independent of scene rebuilds) ----------
  // The WebSocket is opened once on mount and closed on unmount. Each frame is
  // written into whatever live geometry the scene-setup effect currently owns
  // (via sceneRef), so map loads and theme changes rebuild the scene without
  // tearing down the socket. Previously the stream lived in the setup effect,
  // so an async map load closed the still-connecting WS and logged
  // "WebSocket is closed before the connection is established".
  React.useEffect(() => {
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
  }, []);

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

  // ---- Camera mode (move / focus) --------------------------------------
  // Re-runs on a scene rebuild (meta / theme) too, so the mode survives a
  // renderer teardown.
  React.useEffect(() => {
    cameraModeRef.current = cameraMode;
    const ctx = sceneRef.current;
    if (!ctx) return;
    applyCameraMode(ctx.controls, cameraMode);
    // Entering focus: frame the robot from a fixed offset behind and above it.
    if (cameraMode === "focus" && renderedPoseRef.current) {
      const p = renderedPoseRef.current;
      ctx.controls.target.set(p.x, p.y, p.z);
      ctx.camera.position.set(p.x, p.y - 8, p.z + 6);
    }
    ctx.controls.update();
  }, [cameraMode, meta, mapImageUrl, resolvedTheme]);

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

  return (
    <div
      ref={containerRef}
      className={cn("relative h-full w-full overflow-hidden", className)}
    />
  );
}
