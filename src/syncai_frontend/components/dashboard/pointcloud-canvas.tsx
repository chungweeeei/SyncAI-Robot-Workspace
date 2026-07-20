"use client";

import * as React from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
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

interface Theme {
  background: number;
  robot: number;
  ground: number;
  groundOpacity: number;
}

const THEMES: Record<"light" | "dark", Theme> = {
  light: {
    background: 0xf5f5f5,
    robot: 0x2563eb,
    ground: 0xffffff,
    groundOpacity: 0.85,
  },
  dark: {
    background: 0x0a0a0a,
    robot: 0x3b82f6,
    ground: 0x404040,
    groundOpacity: 0.6,
  },
};

/** Map a height to an RGB colour (blue = low, red = high) via an HSL sweep. */
function heightColor(z: number, out: THREE.Color): THREE.Color {
  const t = Math.min(1, Math.max(0, (z - Z_MIN) / (Z_MAX - Z_MIN)));
  // hue 240deg (blue) -> 0deg (red)
  return out.setHSL(((1 - t) * 240) / 360, 0.9, 0.55);
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
  /** When set, also fetch and render the static map cloud for this map. */
  mapCloudName?: string;
  onStatus?: (status: StreamStatus) => void;
  className?: string;
}

export function PointCloudCanvas({
  meta,
  mapImageUrl,
  pose,
  mapCloudName,
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
    robot: THREE.Object3D;
    mapPoints: THREE.Points | null;
    dispose: () => void;
  } | null>(null);

  // Keep the latest onStatus without forcing the setup effect to re-run.
  const onStatusRef = React.useRef(onStatus);
  React.useEffect(() => {
    onStatusRef.current = onStatus;
  }, [onStatus]);

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
      new THREE.PointsMaterial({ size: 0.05, vertexColors: true }),
    );
    livePoints.frustumCulled = false;
    scene.add(livePoints);

    // Robot marker: a cone pointing along +x (heading) at pose height.
    const robot = new THREE.Mesh(
      new THREE.ConeGeometry(0.18, 0.6, 16),
      new THREE.MeshBasicMaterial({ color: theme.robot }),
    );
    // ConeGeometry points along +y; rotate so it points along +x (yaw 0).
    robot.rotation.z = -Math.PI / 2;
    const robotGroup = new THREE.Group();
    robotGroup.add(robot);
    // Hidden until the first pose arrives (a raw cloud test has no /robot/state).
    robotGroup.visible = false;
    scene.add(robotGroup);

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

    let raf = 0;
    const animate = () => {
      controls.update();
      renderer.render(scene, camera);
      raf = requestAnimationFrame(animate);
    };
    raf = requestAnimationFrame(animate);

    // Live stream: write frames straight into the GPU buffers.
    const color = new THREE.Color();
    const posAttr = liveGeom.getAttribute("position") as THREE.BufferAttribute;
    const colAttr = liveGeom.getAttribute("color") as THREE.BufferAttribute;
    const applyFrame = (frame: PointCloudFrame) => {
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
      liveGeom.setDrawRange(0, n);
    };

    const stream = createPointCloudStream({
      onFrame: applyFrame,
      onStatus: (s) => onStatusRef.current?.(s),
    });

    const dispose = () => {
      cancelAnimationFrame(raf);
      observer.disconnect();
      stream.close();
      controls.dispose();
      renderer.dispose();
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
      robot: robotGroup,
      mapPoints: null,
      dispose,
    };

    return () => {
      dispose();
      sceneRef.current = null;
    };
  }, [meta, mapImageUrl, resolvedTheme]);

  // ---- Pose updates (no scene rebuild) ---------------------------------
  React.useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx || !pose) return;
    ctx.robot.visible = true;
    ctx.robot.position.set(pose.x, pose.y, pose.z);
    ctx.robot.rotation.z = (pose.theta * Math.PI) / 180;
  }, [pose]);

  // ---- Optional static map cloud (toggle) ------------------------------
  React.useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;

    if (!mapCloudName) {
      if (ctx.mapPoints) {
        ctx.scene.remove(ctx.mapPoints);
        ctx.mapPoints.geometry.dispose();
        (ctx.mapPoints.material as THREE.Material).dispose();
        ctx.mapPoints = null;
      }
      return;
    }

    const abort = new AbortController();
    fetchMapPointCloud(mapCloudName, { signal: abort.signal })
      .then((frame) => {
        if (abort.signal.aborted || !sceneRef.current) return;
        const geom = new THREE.BufferGeometry();
        geom.setAttribute(
          "position",
          new THREE.BufferAttribute(frame.positions, 3),
        );
        const colors = new Float32Array(frame.count * 3);
        const color = new THREE.Color();
        for (let i = 0; i < frame.count; i++) {
          const j = i * 3;
          heightColor(frame.positions[j + 2], color);
          colors[j] = color.r;
          colors[j + 1] = color.g;
          colors[j + 2] = color.b;
        }
        geom.setAttribute("color", new THREE.BufferAttribute(colors, 3));
        const points = new THREE.Points(
          geom,
          new THREE.PointsMaterial({
            size: 0.03,
            vertexColors: true,
            opacity: 0.5,
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
  }, [mapCloudName]);

  return (
    <div
      ref={containerRef}
      className={cn("relative h-full w-full overflow-hidden", className)}
    />
  );
}
