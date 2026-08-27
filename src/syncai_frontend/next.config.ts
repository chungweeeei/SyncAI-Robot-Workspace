import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  // Production Dockerfile ships only .next/standalone (+ static assets); the
  // dev flow (`npm run dev` against the mounted workspace) is unaffected.
  output: "standalone",
  // Dev-only: the dashboard is served from the container's bridge IP
  // (e.g. http://172.18.0.6:3001), so the browser sends that Origin. Next 16
  // trusts only `localhost` by default and blocks other origins' dev/HMR
  // requests, which breaks the webpack-hmr WebSocket. Allow the container/LAN
  // origins we actually open the dashboard from.
  allowedDevOrigins: ["10.8.140.119"],
};

export default nextConfig;
